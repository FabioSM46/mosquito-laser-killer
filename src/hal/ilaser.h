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
};
