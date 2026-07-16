#pragma once

#include "hal/igpio.h"
#include "core/error.h"
#include <memory>
#include <expected>

class ArmSwitch {
public:
    explicit ArmSwitch(std::unique_ptr<IGpio> gpio, unsigned int debounce_cycles = 6);
    ~ArmSwitch() = default;

    ArmSwitch(const ArmSwitch&) = delete;
    auto operator=(const ArmSwitch&) -> ArmSwitch& = delete;

    [[nodiscard]] auto initialize() -> std::expected<void, HardwareError>;

    auto update() -> void;

    [[nodiscard]] auto is_armed() const -> bool;

private:
    std::unique_ptr<IGpio> gpio_;
    unsigned int debounce_cycles_;
    unsigned int armed_counter_{0};
    bool armed_{false};
    bool initialized_{false};
};
