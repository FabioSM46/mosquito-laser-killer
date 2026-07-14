#pragma once

#include "hal/ispi.h"
#include <string>
#include <cstdint>

class SpiImpl final : public ISpi {
public:
    SpiImpl(const std::string& device, uint32_t speed_hz);
    ~SpiImpl() override;

    SpiImpl(const SpiImpl&) = delete;
    auto operator=(const SpiImpl&) -> SpiImpl& = delete;
    SpiImpl(SpiImpl&&) noexcept;
    auto operator=(SpiImpl&&) noexcept -> SpiImpl&;

    [[nodiscard]] auto transfer(std::span<const uint8_t> tx, std::span<uint8_t> rx)
        -> std::expected<void, HardwareError> override;
    [[nodiscard]] auto write16(uint16_t value) -> std::expected<void, HardwareError> override;
    [[nodiscard]] auto is_open() const -> bool override;

private:
    std::string device_;
    uint32_t speed_hz_{};
    int fd_{-1};

    auto cleanup() -> void;
};
