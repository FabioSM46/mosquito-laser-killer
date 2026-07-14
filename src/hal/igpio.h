#pragma once

#include <expected>
#include <cstdint>
#include "core/error.h"

class IGpio {
public:
    virtual ~IGpio() = default;

    [[nodiscard]] virtual auto write(bool value) -> std::expected<void, HardwareError> = 0;
    [[nodiscard]] virtual auto read() -> std::expected<bool, HardwareError> = 0;
    [[nodiscard]] virtual auto set_direction_output() -> std::expected<void, HardwareError> = 0;
    [[nodiscard]] virtual auto set_direction_input() -> std::expected<void, HardwareError> = 0;
    [[nodiscard]] virtual auto is_open() const -> bool = 0;
};
