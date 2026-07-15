#pragma once

#include "core/types.h"
#include "hal/ilaser.h"
#include "hal/igalvo_driver.h"
#include "control/coordinate_mapper.h"
#include <chrono>
#include <atomic>
#include "core/print.h"

class FiringController {
public:
    FiringController(ILaser& laser,
                     IGalvoDriver& galvo,
                     CoordinateMapper& mapper,
                     double max_pulse_ms = 100.0,
                     double cooldown_s = 10.0,
                     double settle_ms = 3.0);
    ~FiringController() = default;

    FiringController(const FiringController&) = delete;
    auto operator=(const FiringController&) -> FiringController& = delete;

    [[nodiscard]] auto may_fire() const -> bool;

    auto set_target(const Point3D& position) -> void;
    auto clear_target() -> void;

    [[nodiscard]] auto execute_cycle(std::chrono::steady_clock::time_point now)
        -> bool;

    auto emergency_stop() -> void;

private:
    ILaser& laser_;
    IGalvoDriver& galvo_;
    CoordinateMapper& mapper_;

    double max_pulse_ms_;
    double cooldown_s_;
    double settle_delay_ms_;

    std::optional<Point3D> current_target_{};
    bool target_valid_{false};

    std::chrono::steady_clock::time_point cooldown_until_{};
    std::chrono::steady_clock::time_point pulse_start_{};
    std::chrono::steady_clock::time_point galvo_command_time_{};

    bool galvo_settled_{false};
    bool pulse_active_{false};
    bool emergency_stop_{false};
    bool target_just_set_{false};

    [[nodiscard]] auto enforce_timing_limits(std::chrono::steady_clock::time_point now)
        -> bool;
};
