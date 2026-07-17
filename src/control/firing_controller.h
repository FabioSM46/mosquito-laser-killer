#pragma once

#include "core/types.h"
#include "hal/ilaser.h"
#include "hal/igalvo_driver.h"
#include "control/coordinate_mapper.h"
#include <chrono>
#include "core/print.h"

// Sequences the laser fire path behind every timing and state gate.
//
// Every method that can affect the laser takes `now` explicitly. The controller
// reads no clock of its own: a safety component whose behaviour depends on a
// hidden clock cannot be tested deterministically, and the gaps between what the
// tests exercise and what runs are exactly where a Class 4 laser gets loose.
class FiringController {
public:
    // Blanking applied at construction so the system cannot fire the instant it
    // boots, before the operator has control of the arm switch.
    static constexpr auto k_startup_blanking = std::chrono::seconds(1);

    FiringController(ILaser& laser,
                     IGalvoDriver& galvo,
                     CoordinateMapper& mapper,
                     double max_pulse_ms,
                     double cooldown_s,
                     double settle_ms,
                     std::chrono::steady_clock::time_point start_time);
    ~FiringController() = default;

    FiringController(const FiringController&) = delete;
    auto operator=(const FiringController&) -> FiringController& = delete;

    [[nodiscard]] auto may_fire(std::chrono::steady_clock::time_point now) const -> bool;
    [[nodiscard]] auto is_firing() const -> bool;
    [[nodiscard]] auto is_armed() const -> bool;

    // True once a hardware fault has latched the controller off. The owner must
    // poll this and drive the system to SAFE_HALT: the controller deliberately
    // holds no state machine, so it cannot halt the system itself.
    [[nodiscard]] auto is_halted() const -> bool { return emergency_stop_; }

    auto set_armed(bool armed, std::chrono::steady_clock::time_point now) -> void;
    auto set_target(const Point3D& position,
                    std::chrono::steady_clock::time_point now) -> void;
    auto clear_target(std::chrono::steady_clock::time_point now) -> void;
    auto disarm(std::chrono::steady_clock::time_point now) -> void;

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
    bool armed_{false};

    std::chrono::steady_clock::time_point cooldown_until_{};
    std::chrono::steady_clock::time_point pulse_start_{};
    std::chrono::steady_clock::time_point galvo_command_time_{};

    bool galvo_settled_{false};
    bool pulse_active_{false};
    bool emergency_stop_{false};
    bool target_just_set_{false};

    [[nodiscard]] auto enforce_timing_limits(std::chrono::steady_clock::time_point now)
        -> bool;

    auto start_cooldown(std::chrono::steady_clock::time_point now) -> void;
    auto abort_active_pulse(const char* reason,
                            std::chrono::steady_clock::time_point now) -> void;
    auto force_laser_off_and_halt(const char* reason) -> void;
    auto end_pulse(std::chrono::steady_clock::time_point now) -> bool;
};
