#pragma once

#include <expected>
#include <chrono>
#include "core/error.h"

class ILaser {
public:
    virtual ~ILaser() = default;

    [[nodiscard]] virtual auto fire(bool enable) -> std::expected<void, HardwareError> = 0;
    [[nodiscard]] virtual auto emergency_shutdown() -> std::expected<void, HardwareError> = 0;
    [[nodiscard]] virtual auto is_firing() const -> bool = 0;
    [[nodiscard]] virtual auto is_initialized() const -> bool = 0;

    // Defense-in-depth pulse limit, independent of the firing sequencer: force the
    // pin OFF if it has been HIGH longer than the configured maximum. Part of the
    // interface because the control loop must be able to call it on any ILaser,
    // and because a guard that cannot be mocked cannot be tested.
    virtual auto enforce_max_pulse(std::chrono::steady_clock::time_point now) -> void = 0;
};
