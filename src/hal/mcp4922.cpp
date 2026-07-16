#include "hal/mcp4922.h"
#include "hal/ispi.h"
#include "core/error.h"
#include "core/print.h"

MCP4922::MCP4922(std::unique_ptr<ISpi> spi) : spi_(std::move(spi)) {
    if (spi_) {
        auto result = zero();
        if (!result.has_value()) {
            println(stderr, "[MCP4922] Failed to zero DAC on init: {}",
                         to_string(result.error()));
        } else {
            println("[MCP4922] Initialized, both channels at mid-scale (2048)");
        }
    }
}

MCP4922::~MCP4922() {
    if (spi_) {
        auto result = zero();
        if (!result.has_value()) {
            println(stderr, "[MCP4922] Failed to zero DAC on shutdown: {}",
                         to_string(result.error()));
        }
    }
}

auto MCP4922::write(DacValues values) -> std::expected<void, HardwareError> {
    if (values.channel_a > 4095 || values.channel_b > 4095) {
        println(stderr, "[MCP4922] DAC value out of range (0-4095): A={} B={}",
                     values.channel_a, values.channel_b);
        return std::unexpected(HardwareError::DacInvalidValue);
    }

    auto result_a = write_channel(DAC_A_WRITE, values.channel_a);
    if (!result_a.has_value()) {
        return result_a;
    }

    auto result_b = write_channel(DAC_B_WRITE, values.channel_b);
    if (!result_b.has_value()) {
        return result_b;
    }

    return {};
}

auto MCP4922::zero() -> std::expected<void, HardwareError> {
    // Mid-scale on both channels → 0 V differential at mid common-mode (safe center).
    constexpr uint16_t kCenter = 2048;
    return write({kCenter, kCenter});
}

auto MCP4922::is_initialized() const -> bool {
    return spi_ != nullptr;
}

auto MCP4922::write_channel(uint16_t command, uint16_t value)
    -> std::expected<void, HardwareError> {
    uint16_t word = command | (value & CHAN_MASK);

    if (!spi_) {
        return std::unexpected(HardwareError::SpiTransferFailed);
    }

    return spi_->write16(word);
}
