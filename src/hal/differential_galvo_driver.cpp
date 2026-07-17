#include "hal/differential_galvo_driver.h"
#include "core/error.h"
#include "core/print.h"

DifferentialGalvoDriver::DifferentialGalvoDriver(std::unique_ptr<IDac> dac_x,
                                                 std::unique_ptr<IDac> dac_y)
    : dac_x_(std::move(dac_x)), dac_y_(std::move(dac_y)) {
    if (!dac_x_ || !dac_y_) {
        println(stderr, "[GALVO] FATAL: null DAC interface");
        return;
    }

    if (!dac_x_->is_initialized() || !dac_y_->is_initialized()) {
        println(stderr, "[GALVO] FATAL: DAC not initialized");
        return;
    }

    initialized_ = true;

    auto zero_result = zero();
    if (!zero_result.has_value()) {
        initialized_ = false;
        println(stderr, "[GALVO] Failed to zero on init: {}",
                     to_string(zero_result.error()));
        return;
    }

    println("[GALVO] Differential driver initialized");
    println("[GALVO] X axis on DAC0 (A=positive, B=inverted), Y axis on DAC1 (A=positive, B=inverted)");
    println("[GALVO] Center position: DAC=2048 on both channels -> 0V differential");
    println("[GALVO] Full range: DAC=0->4095 maps to -5V->+5V differential");
}

DifferentialGalvoDriver::~DifferentialGalvoDriver() {
    // Only confirm what was actually achieved — see the note in ~Laser.
    if (!dac_x_ || !dac_y_) {
        println(stderr, "[GALVO] Shutdown: DAC missing, galvo position UNKNOWN");
        return;
    }

    auto result = zero();
    if (!result.has_value()) {
        println(stderr, "[GALVO] Shutdown: FAILED to zero: {} — "
                     "GALVO POSITION UNKNOWN", to_string(result.error()));
        return;
    }

    println("[GALVO] Shutdown complete, both axes at center (0V differential)");
}

auto DifferentialGalvoDriver::set_position(uint16_t x, uint16_t y)
    -> std::expected<void, HardwareError> {
    if (!initialized_) {
        println(stderr, "[GALVO] Position rejected: not initialized");
        return std::unexpected(HardwareError::DacInvalidValue);
    }

    if (x > dac_max_ || y > dac_max_) {
        println(stderr, "[GALVO] Position out of 12-bit range: x={} y={}", x, y);
        return std::unexpected(HardwareError::DacInvalidValue);
    }

    uint16_t x_complement = dac_max_ - x;
    uint16_t y_complement = dac_max_ - y;

    auto x_result = dac_x_->write({x, x_complement});
    if (!x_result.has_value()) {
        println(stderr, "[GALVO] X-axis DAC write failed: {}",
                     to_string(x_result.error()));
        return std::unexpected(x_result.error());
    }

    auto y_result = dac_y_->write({y, y_complement});
    if (!y_result.has_value()) {
        println(stderr, "[GALVO] Y-axis DAC write failed: {}",
                     to_string(y_result.error()));
        return std::unexpected(y_result.error());
    }

    return {};
}

auto DifferentialGalvoDriver::zero() -> std::expected<void, HardwareError> {
    uint16_t center = 2048;
    return set_position(center, center);
}

auto DifferentialGalvoDriver::is_initialized() const -> bool {
    return initialized_;
}
