#pragma once

#include "hal/igpio.h"
#include "core/error.h"
#include <memory>
#include <expected>

class EStop {
public:
    explicit EStop(std::unique_ptr<IGpio> gpio, unsigned int debounce_cycles = 3);
    ~EStop() = default;

    EStop(const EStop&) = delete;
    auto operator=(const EStop&) -> EStop& = delete;

    [[nodiscard]] auto initialize() -> std::expected<void, HardwareError>;

    auto update() -> void;

    [[nodiscard]] auto is_pressed() const -> bool;

    [[nodiscard]] auto is_initialized() const -> bool;

private:
    std::unique_ptr<IGpio> gpio_;
    unsigned int debounce_cycles_;
    unsigned int pressed_counter_{0};
    bool pressed_{false};
    bool initialized_{false};
};
