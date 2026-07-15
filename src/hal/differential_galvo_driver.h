#pragma once

#include "hal/igalvo_driver.h"
#include "hal/idac.h"
#include <memory>

class DifferentialGalvoDriver final : public IGalvoDriver {
public:
    DifferentialGalvoDriver(std::unique_ptr<IDac> dac_x, std::unique_ptr<IDac> dac_y);
    ~DifferentialGalvoDriver() override;

    DifferentialGalvoDriver(const DifferentialGalvoDriver&) = delete;
    auto operator=(const DifferentialGalvoDriver&) -> DifferentialGalvoDriver& = delete;
    DifferentialGalvoDriver(DifferentialGalvoDriver&&) noexcept = default;
    auto operator=(DifferentialGalvoDriver&&) noexcept -> DifferentialGalvoDriver& = default;

    [[nodiscard]] auto set_position(uint16_t x, uint16_t y)
        -> std::expected<void, HardwareError> override;

    [[nodiscard]] auto zero() -> std::expected<void, HardwareError> override;

    [[nodiscard]] auto is_initialized() const -> bool override;

private:
    std::unique_ptr<IDac> dac_x_;
    std::unique_ptr<IDac> dac_y_;
    bool initialized_{false};

    static constexpr uint16_t dac_max_{4095};
};
