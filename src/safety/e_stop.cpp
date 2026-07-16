#include "safety/e_stop.h"
#include "core/print.h"

EStop::EStop(std::unique_ptr<IGpio> gpio, unsigned int debounce_cycles)
    : gpio_(std::move(gpio)), debounce_cycles_(debounce_cycles) {
    if (!gpio_) {
        println(stderr, "[ESTOP] FATAL: null GPIO interface");
    }
}

auto EStop::initialize() -> std::expected<void, HardwareError> {
    if (!gpio_) {
        return std::unexpected(HardwareError::GpioOpenFailed);
    }

    auto dir_result = gpio_->set_direction_input();
    if (!dir_result.has_value()) {
        println(stderr, "[ESTOP] Failed to set GPIO direction: {}",
                     to_string(dir_result.error()));
        return std::unexpected(dir_result.error());
    }

    initialized_ = true;
    println("[ESTOP] Initialized, debounce: {} cycles", debounce_cycles_);
    return {};
}

auto EStop::update() -> void {
    if (!initialized_ || !gpio_) {
        return;
    }

    auto read_result = gpio_->read();
    if (!read_result.has_value()) {
        // Fail-safe: treat GPIO faults as E-stop pressed.
        println(stderr, "[ESTOP] GPIO read failed: {} — forcing PRESSED",
                     to_string(read_result.error()));
        pressed_ = true;
        pressed_counter_ = debounce_cycles_;
        return;
    }

    bool pin_low = !read_result.value();

    if (pin_low) {
        if (pressed_counter_ < debounce_cycles_) {
            pressed_counter_++;
        }
    } else {
        if (pressed_counter_ > 0) {
            pressed_counter_--;
        }
    }

    bool prev_pressed = pressed_;

    if (pressed_counter_ >= debounce_cycles_) {
        pressed_ = true;
    } else if (pressed_counter_ == 0) {
        pressed_ = false;
    }

    if (prev_pressed != pressed_) {
        println(stderr, "[ESTOP] E-STOP {} (counter={})",
                     pressed_ ? "PRESSED" : "RELEASED", pressed_counter_);
    }
}

auto EStop::is_pressed() const -> bool {
    return initialized_ && pressed_;
}

auto EStop::is_initialized() const -> bool {
    return initialized_;
}
