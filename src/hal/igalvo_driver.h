#pragma once

#include <expected>
#include <cstdint>
#include "core/error.h"

class IGalvoDriver {
public:
    virtual ~IGalvoDriver() = default;

    [[nodiscard]] virtual auto set_position(uint16_t x, uint16_t y)
        -> std::expected<void, HardwareError> = 0;

    [[nodiscard]] virtual auto zero() -> std::expected<void, HardwareError> = 0;

    [[nodiscard]] virtual auto is_initialized() const -> bool = 0;
};
