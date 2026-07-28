#pragma once

#include <chrono>
#include <expected>
#include <cstdint>
#include <optional>
#include "core/error.h"
#include "core/types.h"

class ICamera {
public:
    virtual ~ICamera() = default;

    [[nodiscard]] virtual auto open(int device_index) -> std::expected<void, HardwareError> = 0;
    [[nodiscard]] virtual auto capture(uint8_t* buffer, size_t size)
        -> std::expected<void, HardwareError> = 0;
    [[nodiscard]] virtual auto is_open() const -> bool = 0;
    virtual void close() = 0;

    // Exposure timestamp of the most recent successful capture() (driver
    // clock where available, steady_clock fallback). On the interface because
    // capture_step() records both cameras' stamps to make the stereo pair's
    // temporal skew measurable (§4.12) — and a value the step depends on must
    // be mockable, for the same reason ILaser carries enforce_max_pulse().
    [[nodiscard]] virtual auto last_frame_timestamp() const
        -> std::chrono::steady_clock::time_point = 0;
};
