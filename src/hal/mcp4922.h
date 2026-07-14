#pragma once

#include "hal/idac.h"
#include <memory>

class ISpi;

class MCP4922 final : public IDac {
public:
    explicit MCP4922(std::unique_ptr<ISpi> spi);
    ~MCP4922() override;

    MCP4922(const MCP4922&) = delete;
    auto operator=(const MCP4922&) -> MCP4922& = delete;
    MCP4922(MCP4922&&) noexcept = default;
    auto operator=(MCP4922&&) noexcept -> MCP4922& = default;

    [[nodiscard]] auto write(DacValues values) -> std::expected<void, HardwareError> override;
    [[nodiscard]] auto zero() -> std::expected<void, HardwareError> override;
    [[nodiscard]] auto is_initialized() const -> bool override;

private:
    std::unique_ptr<ISpi> spi_;

    static constexpr uint16_t DAC_A_WRITE = 0b0011'0000'0000'0000;
    static constexpr uint16_t DAC_B_WRITE = 0b1011'0000'0000'0000;
    static constexpr uint16_t CHAN_MASK   = 0b0000'1111'1111'1111;

    [[nodiscard]] auto write_channel(uint16_t command, uint16_t value)
        -> std::expected<void, HardwareError>;
};
