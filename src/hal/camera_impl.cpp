#include "hal/camera_impl.h"
#include "core/error.h"
#include "core/print.h"
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/videodev2.h>

CameraImpl::CameraImpl(const std::string& device) : device_(device) {
}

CameraImpl::~CameraImpl() {
    close();
}

CameraImpl::CameraImpl(CameraImpl&& other) noexcept
    : device_(std::move(other.device_)), fd_(other.fd_) {
    other.fd_ = -1;
}

auto CameraImpl::operator=(CameraImpl&& other) noexcept -> CameraImpl& {
    if (this != &other) {
        close();
        device_ = std::move(other.device_);
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
    fmt.fmt.pix.width = 640;
    fmt.fmt.pix.height = 480;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_GREY;
    fmt.fmt.pix.field = V4L2_FIELD_NONE;

    if (ioctl(fd_, VIDIOC_S_FMT, &fmt) < 0) {
        println(stderr, "[Camera] VIDIOC_S_FMT failed: {}", strerror(errno));
        close();
        return std::unexpected(HardwareError::CameraOpenFailed);
    }

    println("[Camera] Opened {} at {}x{}", device_, fmt.fmt.pix.width, fmt.fmt.pix.height);
    return {};
}

auto CameraImpl::capture(uint8_t* buffer, size_t size) -> std::expected<void, HardwareError> {
    if (fd_ < 0) {
        return std::unexpected(HardwareError::CameraCaptureFailed);
    }

    auto bytes_read = ::read(fd_, buffer, size);
    if (bytes_read < 0) {
        println(stderr, "[Camera] Capture read failed: {}", strerror(errno));
        return std::unexpected(HardwareError::CameraCaptureFailed);
    }

    return {};
}

auto CameraImpl::is_open() const -> bool {
    return fd_ >= 0;
}

void CameraImpl::close() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
        println("[Camera] Closed {}", device_);
    }
}
