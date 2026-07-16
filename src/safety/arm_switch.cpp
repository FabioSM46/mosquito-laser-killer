#include "safety/arm_switch.h"
#include "core/print.h"

ArmSwitch::ArmSwitch(std::unique_ptr<IGpio> gpio, unsigned int debounce_cycles)
    : gpio_(std::move(gpio)), debounce_cycles_(debounce_cycles) {
    if (!gpio_) {
        println(stderr, "[ARM] FATAL: null GPIO interface");
    }
}

auto ArmSwitch::initialize() -> std::expected<void, HardwareError> {
    if (!gpio_) {
        return std::unexpected(HardwareError::GpioOpenFailed);
    }

    auto dir_result = gpio_->set_direction_input();
    if (!dir_result.has_value()) {
        println(stderr, "[ARM] Failed to set GPIO direction: {}",
                     to_string(dir_result.error()));
        return std::unexpected(dir_result.error());
    }

    initialized_ = true;
    println("[ARM] Initialized, debounce: {} cycles", debounce_cycles_);
    return {};
}

auto ArmSwitch::update() -> void {
    if (!initialized_ || !gpio_) {
        return;
    }

    auto read_result = gpio_->read();
    if (!read_result.has_value()) {
        // Fail-safe: treat GPIO faults as disarmed.
        println(stderr, "[ARM] GPIO read failed: {} — forcing DISARMED",
                     to_string(read_result.error()));
        armed_ = false;
        armed_counter_ = 0;
        return;
    }

    bool pin_high = read_result.value();

    if (pin_high) {
        if (armed_counter_ < debounce_cycles_) {
            armed_counter_++;
        }
    } else {
        if (armed_counter_ > 0) {
            armed_counter_--;
        }
    }

    bool prev_armed = armed_;

    if (armed_counter_ >= debounce_cycles_) {
        armed_ = true;
    } else if (armed_counter_ == 0) {
        armed_ = false;
    }

    if (prev_armed != armed_) {
        println("[ARM] Switch {} (counter={})",
                     armed_ ? "ARMED" : "DISARMED", armed_counter_);
    }
}

auto ArmSwitch::is_armed() const -> bool {
    return initialized_ && armed_;
}
