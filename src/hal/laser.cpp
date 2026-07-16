#include "hal/laser.h"
#include "hal/igpio.h"
#include "core/error.h"
#include "core/print.h"

Laser::Laser(std::unique_ptr<IGpio> gpio, unsigned int pin, double max_pulse_ms)
    : gpio_(std::move(gpio))
    , max_pulse_ms_(max_pulse_ms) {
    println("[LASER] Initializing on GPIO pin {}...", pin);

    if (!gpio_) {
        println(stderr, "[LASER] FATAL: null GPIO interface");
        emergency_shutdown_ = true;
        return;
    }

    auto dir_result = gpio_->set_direction_output();
    if (!dir_result.has_value()) {
        println(stderr, "[LASER] FATAL: failed to set GPIO direction: {}",
                     to_string(dir_result.error()));
        emergency_shutdown_ = true;
        return;
    }

    auto write_result = gpio_->write(false);
    if (!write_result.has_value()) {
        println(stderr, "[LASER] FATAL: failed to force pin LOW on init: {}",
                     to_string(write_result.error()));
        emergency_shutdown_ = true;
        return;
    }

    firing_ = false;
    println("[LASER] Initialized, pin forced LOW (safe state), max_pulse={:.0f}ms",
                 max_pulse_ms_);
}

Laser::~Laser() {
    if (gpio_) {
        auto result = gpio_->write(false);
        if (!result.has_value()) {
            println(stderr, "[LASER] Destructor: failed to force pin LOW: {}",
                         to_string(result.error()));
        }
    }
    println("[LASER] Shutdown complete, pin LOW confirmed");
}

auto Laser::fire(bool enable) -> std::expected<void, HardwareError> {
    if (emergency_shutdown_ && enable) {
        println(stderr, "[LASER] Fire rejected: emergency shutdown active");
        return std::unexpected(HardwareError::LaserEmergencyShutdown);
    }

    if (!gpio_) {
        println(stderr, "[LASER] Fire rejected: no GPIO interface");
        return std::unexpected(HardwareError::GpioWriteFailed);
    }

    if (enable && firing_) {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration<double, std::milli>(now - pulse_start_);
        if (elapsed.count() >= max_pulse_ms_) {
            println(stderr, "[LASER] Max pulse exceeded ({:.0f}ms), forcing OFF",
                         max_pulse_ms_);
            auto off = gpio_->write(false);
            firing_ = false;
            if (!off.has_value()) {
                emergency_shutdown_ = true;
                return std::unexpected(off.error());
            }
            return std::unexpected(HardwareError::LaserEmergencyShutdown);
        }
    }

    auto result = gpio_->write(enable);
    if (!result.has_value()) {
        println(stderr, "[LASER] GPIO write failed for fire({}): {}",
                     enable, to_string(result.error()));
        emergency_shutdown_ = true;
        return std::unexpected(result.error());
    }

    if (enable && !firing_) {
        pulse_start_ = std::chrono::steady_clock::now();
    }

    firing_ = enable;

    if (enable) {
        println("[LASER] FIRING");
    } else {
        println("[LASER] OFF");
    }

    return {};
}

auto Laser::enforce_max_pulse(std::chrono::steady_clock::time_point now) -> void {
    if (!firing_ || emergency_shutdown_ || !gpio_) {
        return;
    }

    auto elapsed = std::chrono::duration<double, std::milli>(now - pulse_start_);
    if (elapsed.count() < max_pulse_ms_) {
        return;
    }

    println(stderr, "[LASER] enforce_max_pulse: duration {:.0f}ms exceeded, forcing OFF",
                 max_pulse_ms_);
    auto result = gpio_->write(false);
    firing_ = false;
    if (!result.has_value()) {
        emergency_shutdown_ = true;
        println(stderr, "[LASER] enforce_max_pulse write failed: {}",
                     to_string(result.error()));
    }
}

auto Laser::emergency_shutdown() -> std::expected<void, HardwareError> {
    println(stderr, "[LASER] EMERGENCY SHUTDOWN TRIGGERED");

    emergency_shutdown_ = true;

    if (gpio_) {
        auto result = gpio_->write(false);
        if (!result.has_value()) {
            println(stderr, "[LASER] Emergency shutdown: failed to force pin LOW: {}",
                         to_string(result.error()));
            return std::unexpected(result.error());
        }
    }

    firing_ = false;
    println(stderr, "[LASER] Emergency shutdown complete, pin forced LOW");
    return {};
}

auto Laser::is_firing() const -> bool {
    return firing_;
}

auto Laser::is_initialized() const -> bool {
    return gpio_ != nullptr && !emergency_shutdown_;
}
