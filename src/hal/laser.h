#pragma once

#include "hal/ilaser.h"
#include <memory>
#include <chrono>

class IGpio;

class Laser final : public ILaser {
public:
    explicit Laser(std::unique_ptr<IGpio> gpio, unsigned int pin,
                   double max_pulse_ms = 100.0);
    ~Laser() override;

    Laser(const Laser&) = delete;
    auto operator=(const Laser&) -> Laser& = delete;
    Laser(Laser&&) noexcept = delete;
    auto operator=(Laser&&) noexcept -> Laser& = delete;

    [[nodiscard]] auto fire(bool enable) -> std::expected<void, HardwareError> override;
    [[nodiscard]] auto emergency_shutdown() -> std::expected<void, HardwareError> override;
    [[nodiscard]] auto is_firing() const -> bool override;
    [[nodiscard]] auto is_initialized() const -> bool override;

    // Defense-in-depth pulse limit: force OFF if pin has been HIGH too long.
    auto enforce_max_pulse(std::chrono::steady_clock::time_point now) -> void;

private:
    std::unique_ptr<IGpio> gpio_;
    bool firing_{false};
    bool emergency_shutdown_{false};
    double max_pulse_ms_{100.0};
    std::chrono::steady_clock::time_point pulse_start_{};
};
