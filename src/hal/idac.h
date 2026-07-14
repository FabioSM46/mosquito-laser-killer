#pragma once

#include <expected>
#include <cstdint>
#include "core/error.h"
#include "core/types.h"

class IDac {
public:
    virtual ~IDac() = default;

    [[nodiscard]] virtual auto write(DacValues values) -> std::expected<void, HardwareError> = 0;
    [[nodiscard]] virtual auto zero() -> std::expected<void, HardwareError> = 0;
    [[nodiscard]] virtual auto is_initialized() const -> bool = 0;
};
