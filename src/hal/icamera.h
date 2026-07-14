#pragma once

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
};
