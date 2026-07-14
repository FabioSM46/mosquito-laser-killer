#pragma once

#include "hal/igpio.h"
#include <string>
#include <memory>

namespace gpiod {
class chip;
class line;
}

class GpioImpl final : public IGpio {
public:
    explicit GpioImpl(unsigned int pin);
    ~GpioImpl() override;

    GpioImpl(const GpioImpl&) = delete;
    auto operator=(const GpioImpl&) -> GpioImpl& = delete;
    GpioImpl(GpioImpl&&) noexcept;
    auto operator=(GpioImpl&&) noexcept -> GpioImpl&;

    [[nodiscard]] auto write(bool value) -> std::expected<void, HardwareError> override;
    [[nodiscard]] auto read() -> std::expected<bool, HardwareError> override;
    [[nodiscard]] auto set_direction_output() -> std::expected<void, HardwareError> override;
    [[nodiscard]] auto set_direction_input() -> std::expected<void, HardwareError> override;
    [[nodiscard]] auto is_open() const -> bool override;

private:
    unsigned int pin_{};
    std::unique_ptr<gpiod::chip> chip_;
    std::unique_ptr<gpiod::line> line_;
    bool output_direction_{false};

    auto release() -> void;
};
