#pragma once

#include <expected>
#include <cstdint>
#include <vector>
#include <span>
#include "core/error.h"

class ISpi {
public:
    virtual ~ISpi() = default;

    [[nodiscard]] virtual auto transfer(std::span<const uint8_t> tx,
                                        std::span<uint8_t> rx)
        -> std::expected<void, HardwareError> = 0;

    [[nodiscard]] virtual auto write16(uint16_t value) -> std::expected<void, HardwareError> = 0;
    [[nodiscard]] virtual auto is_open() const -> bool = 0;
};
