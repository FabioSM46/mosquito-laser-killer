#include "hal/gpio_impl.h"
#include "core/error.h"
#include "core/print.h"
#include <gpiod.hpp>
#include <cstring>

GpioImpl::GpioImpl(unsigned int pin) : pin_(pin) {
}

GpioImpl::~GpioImpl() {
    release();
}

GpioImpl::GpioImpl(GpioImpl&& other) noexcept
    : pin_(other.pin_)
    , chip_(std::move(other.chip_))
    , line_(std::move(other.line_))
    , output_direction_(other.output_direction_) {
    other.output_direction_ = false;
}

auto GpioImpl::operator=(GpioImpl&& other) noexcept -> GpioImpl& {
    if (this != &other) {
        release();
        pin_ = other.pin_;
        chip_ = std::move(other.chip_);
        line_ = std::move(other.line_);
        output_direction_ = other.output_direction_;
        other.output_direction_ = false;
    }
    return *this;
}

auto GpioImpl::release() -> void {
    if (line_) {
        if (output_direction_) {
            line_->set_value(0);
        }
        line_->release();
        line_.reset();
    }
    chip_.reset();
}

auto GpioImpl::set_direction_output() -> std::expected<void, HardwareError> {
    release();

    try {
        chip_ = std::make_unique<gpiod::chip>("/dev/gpiochip0");
        line_ = std::make_unique<gpiod::line>(chip_->get_line(pin_));

        line_->request({"mosquito-laser-killer",
                        gpiod::line_request::DIRECTION_OUTPUT,
                        0}, 0);
    } catch (const std::exception& e) {
        println(stderr, "[GPIO {}] Failed to configure as output: {}", pin_, e.what());
        release();
        return std::unexpected(HardwareError::GpioOpenFailed);
    }

    output_direction_ = true;

    line_->set_value(0);

    println("[GPIO {}] Initialized as output via gpiod, forced LOW", pin_);
    return {};
}

auto GpioImpl::set_direction_input() -> std::expected<void, HardwareError> {
    release();

    try {
        chip_ = std::make_unique<gpiod::chip>("/dev/gpiochip0");
        line_ = std::make_unique<gpiod::line>(chip_->get_line(pin_));

        line_->request({"mosquito-laser-killer",
                        gpiod::line_request::DIRECTION_INPUT,
                        0});
    } catch (const std::exception& e) {
        println(stderr, "[GPIO {}] Failed to configure as input: {}", pin_, e.what());
        release();
        return std::unexpected(HardwareError::GpioOpenFailed);
    }

    output_direction_ = false;
    return {};
}

auto GpioImpl::write(bool value) -> std::expected<void, HardwareError> {
    if (!line_ || !output_direction_) {
        println(stderr, "[GPIO {}] Write attempted on unconfigured pin", pin_);
        return std::unexpected(HardwareError::GpioWriteFailed);
    }

    try {
        line_->set_value(value ? 1 : 0);
    } catch (const std::exception& e) {
        println(stderr, "[GPIO {}] Write failed: {}", pin_, e.what());
        return std::unexpected(HardwareError::GpioWriteFailed);
    }

    return {};
}

auto GpioImpl::read() -> std::expected<bool, HardwareError> {
    if (!line_) {
        println(stderr, "[GPIO {}] Read attempted on unconfigured pin", pin_);
        return std::unexpected(HardwareError::GpioReadFailed);
    }

    try {
        int val = line_->get_value();
        return val != 0;
    } catch (const std::exception& e) {
        println(stderr, "[GPIO {}] Read failed: {}", pin_, e.what());
        return std::unexpected(HardwareError::GpioReadFailed);
    }
}

auto GpioImpl::is_open() const -> bool {
    return chip_ != nullptr && line_ != nullptr;
}
