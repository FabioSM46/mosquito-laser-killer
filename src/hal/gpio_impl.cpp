#include "hal/gpio_impl.h"
#include "core/error.h"
#include "core/print.h"
#include <fcntl.h>
#include <unistd.h>
#include <cstring>

GpioImpl::GpioImpl(unsigned int pin) : pin_(pin) {
}

GpioImpl::~GpioImpl() {
    cleanup();
}

GpioImpl::GpioImpl(GpioImpl&& other) noexcept
    : pin_(other.pin_), fd_(other.fd_), output_direction_(other.output_direction_) {
    other.fd_ = -1;
}

auto GpioImpl::operator=(GpioImpl&& other) noexcept -> GpioImpl& {
    if (this != &other) {
        cleanup();
        pin_ = other.pin_;
        fd_ = other.fd_;
        output_direction_ = other.output_direction_;
        other.fd_ = -1;
    }
    return *this;
}

auto GpioImpl::cleanup() -> void {
    if (fd_ >= 0) {
        if (output_direction_) {
            (void)write(false);
        }
        close(fd_);
        fd_ = -1;
    }
}

auto GpioImpl::set_direction_output() -> std::expected<void, HardwareError> {
    std::string path = "/sys/class/gpio/gpio" + std::to_string(pin_) + "/direction";

    int export_fd = open("/sys/class/gpio/export", O_WRONLY);
    if (export_fd >= 0) {
        std::string pin_str = std::to_string(pin_);
        (void)!::write(export_fd, pin_str.c_str(), pin_str.size());
        close(export_fd);
    }

    fd_ = open(path.c_str(), O_WRONLY);
    if (fd_ < 0) {
        println(stderr, "[GPIO {}] Failed to open direction file: {}", pin_, strerror(errno));
        return std::unexpected(HardwareError::GpioOpenFailed);
    }

    if (::write(fd_, "out", 3) < 0) {
        println(stderr, "[GPIO {}] Failed to set direction output: {}", pin_, strerror(errno));
        close(fd_);
        fd_ = -1;
        return std::unexpected(HardwareError::GpioWriteFailed);
    }
    close(fd_);

    std::string value_path = "/sys/class/gpio/gpio" + std::to_string(pin_) + "/value";
    fd_ = open(value_path.c_str(), O_WRONLY);
    if (fd_ < 0) {
        println(stderr, "[GPIO {}] Failed to open value file: {}", pin_, strerror(errno));
        return std::unexpected(HardwareError::GpioOpenFailed);
    }

    output_direction_ = true;
    auto result = write(false);
    if (!result.has_value()) {
        cleanup();
        return std::unexpected(result.error());
    }

    println("[GPIO {}] Initialized as output, forced LOW", pin_);
    return {};
}

auto GpioImpl::set_direction_input() -> std::expected<void, HardwareError> {
    cleanup();
    output_direction_ = false;

    std::string path = "/sys/class/gpio/gpio" + std::to_string(pin_) + "/value";
    fd_ = open(path.c_str(), O_RDONLY);
    if (fd_ < 0) {
        println(stderr, "[GPIO {}] Failed to open value for input: {}", pin_, strerror(errno));
        return std::unexpected(HardwareError::GpioOpenFailed);
    }

    return {};
}

auto GpioImpl::write(bool value) -> std::expected<void, HardwareError> {
    if (fd_ < 0 || !output_direction_) {
        println(stderr, "[GPIO {}] Write attempted on unconfigured pin", pin_);
        return std::unexpected(HardwareError::GpioWriteFailed);
    }

    const char* val = value ? "1" : "0";
    auto n = ::write(fd_, val, 1);
    if (n < 0) {
        println(stderr, "[GPIO {}] Write failed: {}", pin_, strerror(errno));
        return std::unexpected(HardwareError::GpioWriteFailed);
    }

    lseek(fd_, 0, SEEK_SET);
    return {};
}

auto GpioImpl::read() -> std::expected<bool, HardwareError> {
    if (fd_ < 0) {
        println(stderr, "[GPIO {}] Read attempted on unconfigured pin", pin_);
        return std::unexpected(HardwareError::GpioReadFailed);
    }

    char buf[2]{};
    lseek(fd_, 0, SEEK_SET);
    auto n = ::read(fd_, buf, 1);
    if (n < 0) {
        println(stderr, "[GPIO {}] Read failed: {}", pin_, strerror(errno));
        return std::unexpected(HardwareError::GpioReadFailed);
    }

    return buf[0] == '1';
}

auto GpioImpl::is_open() const -> bool {
    return fd_ >= 0;
}
