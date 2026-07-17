#include "hal/camera_impl.h"
#include "core/error.h"
#include "core/print.h"
#include <chrono>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/select.h>
#include <linux/videodev2.h>

namespace {

constexpr auto k_capture_timeout_ms = 1000;

// ioctl() is interruptible; a signal must not look like a hardware failure.
auto xioctl(int fd, unsigned long request, void* arg) -> int {
    int ret = 0;
    do {
        ret = ioctl(fd, request, arg);
    } while (ret == -1 && errno == EINTR);
    return ret;
}

auto wait_for_frame(int fd, int timeout_ms) -> std::expected<void, HardwareError> {
    if (fd < 0) {
        return std::unexpected(HardwareError::CameraCaptureFailed);
    }

    // An absolute deadline, not a per-iteration timeout: re-arming the full
    // timeout after every EINTR would let repeated signals extend the block
    // arbitrarily far beyond the stated bound.
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(timeout_ms);

    for (;;) {
        const auto remaining = deadline - std::chrono::steady_clock::now();
        if (remaining <= std::chrono::steady_clock::duration::zero()) {
            println(stderr, "[Camera] Capture timeout ({} ms)", timeout_ms);
            return std::unexpected(HardwareError::Timeout);
        }
        const auto remaining_us =
            std::chrono::duration_cast<std::chrono::microseconds>(remaining);

        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(fd, &fds);

        timeval tv{};
        tv.tv_sec = static_cast<time_t>(remaining_us.count() / 1'000'000);
        tv.tv_usec = static_cast<suseconds_t>(remaining_us.count() % 1'000'000);

        auto ret = select(fd + 1, &fds, nullptr, nullptr, &tv);
        if (ret < 0) {
            if (errno == EINTR) {
                continue;
            }
            println(stderr, "[Camera] select() failed: {}", strerror(errno));
            return std::unexpected(HardwareError::CameraCaptureFailed);
        }
        if (ret == 0) {
            println(stderr, "[Camera] Capture timeout ({} ms)", timeout_ms);
            return std::unexpected(HardwareError::Timeout);
        }
        return {};
    }
}

auto fourcc_to_string(uint32_t fourcc) -> std::string {
    return {static_cast<char>(fourcc & 0xFF),
            static_cast<char>((fourcc >> 8) & 0xFF),
            static_cast<char>((fourcc >> 16) & 0xFF),
            static_cast<char>((fourcc >> 24) & 0xFF)};
}

}

CameraImpl::CameraImpl(const std::string& device, int width, int height, int fps,
                       const SystemConfig::CameraControls& controls)
    : device_(device)
    , width_(width)
    , height_(height)
    , fps_(fps)
    , controls_(controls) {
}

CameraImpl::~CameraImpl() {
    close();
}

CameraImpl::CameraImpl(CameraImpl&& other) noexcept
    : device_(std::move(other.device_))
    , width_(other.width_)
    , height_(other.height_)
    , fps_(other.fps_)
    , controls_(other.controls_)
    , fd_(other.fd_)
    , frame_size_bytes_(other.frame_size_bytes_)
    , buffers_(std::move(other.buffers_))
    , streaming_(other.streaming_) {
    other.fd_ = -1;
    other.streaming_ = false;
    other.frame_size_bytes_ = 0;
    other.buffers_.clear();
}

auto CameraImpl::operator=(CameraImpl&& other) noexcept -> CameraImpl& {
    if (this != &other) {
        close();
        device_ = std::move(other.device_);
        width_ = other.width_;
        height_ = other.height_;
        fps_ = other.fps_;
        controls_ = other.controls_;
        fd_ = other.fd_;
        frame_size_bytes_ = other.frame_size_bytes_;
        buffers_ = std::move(other.buffers_);
        streaming_ = other.streaming_;
        other.fd_ = -1;
        other.streaming_ = false;
        other.frame_size_bytes_ = 0;
        other.buffers_.clear();
    }
    return *this;
}

auto CameraImpl::negotiate_format() -> std::expected<void, HardwareError> {
    v4l2_format fmt{};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = static_cast<uint32_t>(width_);
    fmt.fmt.pix.height = static_cast<uint32_t>(height_);
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_GREY;
    fmt.fmt.pix.field = V4L2_FIELD_NONE;

    if (xioctl(fd_, VIDIOC_S_FMT, &fmt) < 0) {
        println(stderr, "[Camera] VIDIOC_S_FMT failed on {}: {}", device_, strerror(errno));
        return std::unexpected(HardwareError::CameraOpenFailed);
    }

    // S_FMT reports what the driver actually accepted, which may differ from the
    // request. Silently accepting a substitution would feed the wrong geometry or
    // pixel layout to the detector, so treat a mismatch as an open failure.
    if (fmt.fmt.pix.width != static_cast<uint32_t>(width_) ||
        fmt.fmt.pix.height != static_cast<uint32_t>(height_)) {
        println(stderr, "[Camera] {} substituted {}x{} for the requested {}x{}",
                device_, fmt.fmt.pix.width, fmt.fmt.pix.height, width_, height_);
        return std::unexpected(HardwareError::CameraOpenFailed);
    }

    if (fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_GREY) {
        println(stderr, "[Camera] {} substituted pixel format {} for the requested GREY",
                device_, fourcc_to_string(fmt.fmt.pix.pixelformat));
        return std::unexpected(HardwareError::CameraOpenFailed);
    }

    // capture() memcpys the mapped buffer straight into a tightly-packed
    // width*height frame, so any row padding would shear the image: each row n
    // shifted by n*(bytesperline - width). A sheared frame still yields clean
    // blobs, still satisfies the epipolar gate (both frames shear identically),
    // and still triangulates to a confident 3D point — for a place nothing is.
    if (fmt.fmt.pix.bytesperline != static_cast<uint32_t>(width_)) {
        println(stderr, "[Camera] {} reports bytesperline {} for a {}-pixel row; "
                "row-padded buffers are not supported", device_,
                fmt.fmt.pix.bytesperline, width_);
        return std::unexpected(HardwareError::CameraOpenFailed);
    }

    const size_t expected = static_cast<size_t>(width_) * static_cast<size_t>(height_);
    if (fmt.fmt.pix.sizeimage < expected) {
        println(stderr, "[Camera] {} reports sizeimage {} below the {} bytes implied "
                "by {}x{} GREY", device_, fmt.fmt.pix.sizeimage, expected, width_, height_);
        return std::unexpected(HardwareError::CameraOpenFailed);
    }

    frame_size_bytes_ = fmt.fmt.pix.sizeimage;

    v4l2_streamparm parm{};
    parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    parm.parm.capture.timeperframe.numerator = 1;
    parm.parm.capture.timeperframe.denominator = static_cast<uint32_t>(fps_);

    if (xioctl(fd_, VIDIOC_S_PARM, &parm) < 0) {
        println(stderr, "[Camera] VIDIOC_S_PARM failed on {}: {}", device_, strerror(errno));
        return std::unexpected(HardwareError::CameraOpenFailed);
    }

    // Frame rate is a performance parameter, not a safety one: warn, don't fail.
    const auto& tpf = parm.parm.capture.timeperframe;
    if (tpf.numerator != 1 || tpf.denominator != static_cast<uint32_t>(fps_)) {
        println(stderr, "[Camera] {} negotiated {}/{} fps, requested {}",
                device_, tpf.denominator, tpf.numerator, fps_);
    }

    return {};
}

auto CameraImpl::init_mmap_buffers() -> std::expected<void, HardwareError> {
    v4l2_requestbuffers req{};
    req.count = k_buffer_count;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (xioctl(fd_, VIDIOC_REQBUFS, &req) < 0) {
        println(stderr, "[Camera] VIDIOC_REQBUFS failed on {}: {}", device_, strerror(errno));
        return std::unexpected(HardwareError::CameraOpenFailed);
    }

    if (req.count < 2) {
        println(stderr, "[Camera] {} granted only {} buffers, need at least 2",
                device_, req.count);
        return std::unexpected(HardwareError::CameraOpenFailed);
    }

    buffers_.resize(req.count);

    for (unsigned int i = 0; i < req.count; ++i) {
        v4l2_buffer buf{};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        if (xioctl(fd_, VIDIOC_QUERYBUF, &buf) < 0) {
            println(stderr, "[Camera] VIDIOC_QUERYBUF({}) failed on {}: {}",
                    i, device_, strerror(errno));
            return std::unexpected(HardwareError::CameraOpenFailed);
        }

        void* start = mmap(nullptr, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED,
                           fd_, buf.m.offset);
        if (start == MAP_FAILED) {
            println(stderr, "[Camera] mmap({}) failed on {}: {}",
                    i, device_, strerror(errno));
            return std::unexpected(HardwareError::CameraOpenFailed);
        }

        buffers_[i].start = start;
        buffers_[i].length = buf.length;

        if (xioctl(fd_, VIDIOC_QBUF, &buf) < 0) {
            println(stderr, "[Camera] VIDIOC_QBUF({}) failed on {}: {}",
                    i, device_, strerror(errno));
            return std::unexpected(HardwareError::CameraOpenFailed);
        }
    }

    return {};
}

auto CameraImpl::unmap_buffers() -> void {
    for (auto& b : buffers_) {
        if (b.start != nullptr && b.start != MAP_FAILED) {
            if (munmap(b.start, b.length) < 0) {
                println(stderr, "[Camera] munmap failed on {}: {}", device_, strerror(errno));
            }
        }
        b.start = nullptr;
        b.length = 0;
    }
    buffers_.clear();
}

auto CameraImpl::open(int /*device_index*/) -> std::expected<void, HardwareError> {
    fd_ = ::open(device_.c_str(), O_RDWR | O_CLOEXEC);
    if (fd_ < 0) {
        println(stderr, "[Camera] Failed to open {}: {}", device_, strerror(errno));
        return std::unexpected(HardwareError::CameraOpenFailed);
    }

    v4l2_capability cap{};
    if (xioctl(fd_, VIDIOC_QUERYCAP, &cap) < 0) {
        println(stderr, "[Camera] VIDIOC_QUERYCAP failed on {}: {}", device_, strerror(errno));
        close();
        return std::unexpected(HardwareError::CameraOpenFailed);
    }

    // A device advertising V4L2_CAP_DEVICE_CAPS reports per-node capabilities in
    // device_caps; the top-level capabilities field covers the whole device.
    const uint32_t caps = (cap.capabilities & V4L2_CAP_DEVICE_CAPS) != 0
                              ? cap.device_caps
                              : cap.capabilities;

    if ((caps & V4L2_CAP_VIDEO_CAPTURE) == 0) {
        println(stderr, "[Camera] {} does not support video capture", device_);
        close();
        return std::unexpected(HardwareError::CameraOpenFailed);
    }

    if ((caps & V4L2_CAP_STREAMING) == 0) {
        println(stderr, "[Camera] {} does not support streaming I/O", device_);
        close();
        return std::unexpected(HardwareError::CameraOpenFailed);
    }

    auto fmt_result = negotiate_format();
    if (!fmt_result.has_value()) {
        close();
        return std::unexpected(fmt_result.error());
    }

    apply_controls();

    auto buf_result = init_mmap_buffers();
    if (!buf_result.has_value()) {
        close();
        return std::unexpected(buf_result.error());
    }

    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(fd_, VIDIOC_STREAMON, &type) < 0) {
        println(stderr, "[Camera] VIDIOC_STREAMON failed on {}: {}", device_, strerror(errno));
        close();
        return std::unexpected(HardwareError::CameraOpenFailed);
    }
    streaming_ = true;

    println("[Camera] Opened {} at {}x{}@{} GREY, {} mmap buffers, {} bytes/frame",
            device_, width_, height_, fps_, buffers_.size(), frame_size_bytes_);

    return {};
}

auto CameraImpl::apply_controls() -> void {
    auto set_ctrl = [this](uint32_t id, int value, const char* name) {
        v4l2_control ctrl{};
        ctrl.id = id;
        ctrl.value = value;
        if (xioctl(fd_, VIDIOC_S_CTRL, &ctrl) < 0) {
            println(stderr, "[Camera] {} set failed on {}: {}", name, device_, strerror(errno));
        }
    };

    set_ctrl(V4L2_CID_EXPOSURE_AUTO, controls_.exposure_auto, "exposure_auto");
    set_ctrl(V4L2_CID_EXPOSURE_ABSOLUTE, controls_.exposure_absolute_us, "exposure_absolute");
    set_ctrl(V4L2_CID_BRIGHTNESS, controls_.brightness, "brightness");
    set_ctrl(V4L2_CID_GAMMA, controls_.gamma, "gamma");
    set_ctrl(V4L2_CID_SHARPNESS, controls_.sharpness, "sharpness");
    set_ctrl(V4L2_CID_GAIN, controls_.gain, "gain");
}

auto CameraImpl::capture(uint8_t* buffer, size_t size) -> std::expected<void, HardwareError> {
    if (fd_ < 0 || !streaming_ || buffer == nullptr) {
        return std::unexpected(HardwareError::CameraCaptureFailed);
    }

    auto ready = wait_for_frame(fd_, k_capture_timeout_ms);
    if (!ready.has_value()) {
        return std::unexpected(ready.error());
    }

    v4l2_buffer buf{};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;

    if (xioctl(fd_, VIDIOC_DQBUF, &buf) < 0) {
        if (errno == EAGAIN) {
            return std::unexpected(HardwareError::Timeout);
        }
        println(stderr, "[Camera] VIDIOC_DQBUF failed on {}: {}", device_, strerror(errno));
        return std::unexpected(HardwareError::CameraCaptureFailed);
    }

    if (buf.index >= buffers_.size()) {
        println(stderr, "[Camera] DQBUF returned out-of-range index {} on {}",
                buf.index, device_);
        return std::unexpected(HardwareError::CameraCaptureFailed);
    }

    // Requeue on every exit path below, or the driver runs out of buffers.
    auto requeue = [&]() -> std::expected<void, HardwareError> {
        if (xioctl(fd_, VIDIOC_QBUF, &buf) < 0) {
            println(stderr, "[Camera] VIDIOC_QBUF failed on {}: {}", device_, strerror(errno));
            return std::unexpected(HardwareError::CameraCaptureFailed);
        }
        return {};
    };

    if ((buf.flags & V4L2_BUF_FLAG_ERROR) != 0) {
        println(stderr, "[Camera] {} flagged a buffer error", device_);
        (void)requeue();
        return std::unexpected(HardwareError::CameraCaptureFailed);
    }

    if (buf.bytesused < size) {
        println(stderr, "[Camera] Capture incomplete on {}: {} of {} bytes",
                device_, buf.bytesused, size);
        (void)requeue();
        return std::unexpected(HardwareError::CameraCaptureFailed);
    }

    std::memcpy(buffer, buffers_[buf.index].start, size);

    return requeue();
}

auto CameraImpl::is_open() const -> bool {
    return fd_ >= 0;
}

void CameraImpl::close() {
    if (fd_ >= 0) {
        if (streaming_) {
            int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            (void)xioctl(fd_, VIDIOC_STREAMOFF, &type);
            streaming_ = false;
        }

        unmap_buffers();

        // Release the driver's buffer allocation before dropping the fd.
        v4l2_requestbuffers req{};
        req.count = 0;
        req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory = V4L2_MEMORY_MMAP;
        (void)xioctl(fd_, VIDIOC_REQBUFS, &req);

        ::close(fd_);
        fd_ = -1;
        frame_size_bytes_ = 0;
        println("[Camera] Closed {}", device_);
    } else {
        unmap_buffers();
    }
}
