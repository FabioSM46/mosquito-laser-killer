#include "hal/camera_impl.h"
#include "core/error.h"
#include "core/print.h"
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/select.h>
#include <linux/videodev2.h>

namespace {

constexpr auto k_capture_timeout_ms = 1000;

auto wait_for_frame(int fd, int timeout_ms) -> std::expected<void, HardwareError> {
    if (fd < 0) {
        return std::unexpected(HardwareError::CameraCaptureFailed);
    }

    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(fd, &fds);

    timeval tv{};
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;

    auto ret = select(fd + 1, &fds, nullptr, nullptr, &tv);
    if (ret < 0) {
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
    , fd_(other.fd_) {
    other.fd_ = -1;
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
        other.fd_ = -1;
    }
    return *this;
}

auto CameraImpl::open(int /*device_index*/) -> std::expected<void, HardwareError> {
    fd_ = ::open(device_.c_str(), O_RDWR);
    if (fd_ < 0) {
        println(stderr, "[Camera] Failed to open {}: {}", device_, strerror(errno));
        return std::unexpected(HardwareError::CameraOpenFailed);
    }

    v4l2_capability cap{};
    if (ioctl(fd_, VIDIOC_QUERYCAP, &cap) < 0) {
        println(stderr, "[Camera] VIDIOC_QUERYCAP failed: {}", strerror(errno));
        close();
        return std::unexpected(HardwareError::CameraOpenFailed);
    }

    v4l2_format fmt{};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = static_cast<uint32_t>(width_);
    fmt.fmt.pix.height = static_cast<uint32_t>(height_);
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_GREY;
    fmt.fmt.pix.field = V4L2_FIELD_NONE;

    if (ioctl(fd_, VIDIOC_S_FMT, &fmt) < 0) {
        println(stderr, "[Camera] VIDIOC_S_FMT failed: {}", strerror(errno));
        close();
        return std::unexpected(HardwareError::CameraOpenFailed);
    }

    v4l2_streamparm parm{};
    parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    parm.parm.capture.timeperframe.numerator = 1;
    parm.parm.capture.timeperframe.denominator = static_cast<uint32_t>(fps_);

    if (ioctl(fd_, VIDIOC_S_PARM, &parm) < 0) {
        println(stderr, "[Camera] VIDIOC_S_PARM failed: {}", strerror(errno));
        close();
        return std::unexpected(HardwareError::CameraOpenFailed);
    }

    apply_controls();

    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd_, VIDIOC_STREAMON, &type) < 0) {
        println(stderr, "[Camera] VIDIOC_STREAMON failed on {}: {}", device_, strerror(errno));
        close();
        return std::unexpected(HardwareError::CameraOpenFailed);
    }

    println("[Camera] Opened {} at {}x{}@{} (actual {}x{}@{}/{})",
            device_, width_, height_, fps_,
            fmt.fmt.pix.width, fmt.fmt.pix.height,
            parm.parm.capture.timeperframe.denominator,
            parm.parm.capture.timeperframe.numerator);

    return {};
}

auto CameraImpl::apply_controls() -> void {
    auto set_ctrl = [this](uint32_t id, int value, const char* name) {
        v4l2_control ctrl{};
        ctrl.id = id;
        ctrl.value = value;
        if (ioctl(fd_, VIDIOC_S_CTRL, &ctrl) < 0) {
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
    if (fd_ < 0) {
        return std::unexpected(HardwareError::CameraCaptureFailed);
    }

    auto ready = wait_for_frame(fd_, k_capture_timeout_ms);
    if (!ready.has_value()) {
        return std::unexpected(ready.error());
    }

    auto bytes_read = ::read(fd_, buffer, size);
    if (bytes_read < 0) {
        println(stderr, "[Camera] Capture read failed: {}", strerror(errno));
        return std::unexpected(HardwareError::CameraCaptureFailed);
    }

    if (bytes_read != static_cast<ssize_t>(size)) {
        println(stderr, "[Camera] Capture incomplete: read {} of {} bytes", bytes_read, size);
        return std::unexpected(HardwareError::CameraCaptureFailed);
    }

    return {};
}

auto CameraImpl::is_open() const -> bool {
    return fd_ >= 0;
}

void CameraImpl::close() {
    if (fd_ >= 0) {
        int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        (void)ioctl(fd_, VIDIOC_STREAMOFF, &type);
        ::close(fd_);
        fd_ = -1;
        println("[Camera] Closed {}", device_);
    }
}
