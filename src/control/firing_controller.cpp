#include "control/firing_controller.h"
#include "core/error.h"

FiringController::FiringController(ILaser& laser,
                                   IGalvoDriver& galvo,
                                   CoordinateMapper& mapper,
                                   double max_pulse_ms,
                                   double cooldown_s,
                                   double settle_ms,
                                   std::chrono::steady_clock::time_point start_time)
    : laser_(laser)
    , galvo_(galvo)
    , mapper_(mapper)
    , max_pulse_ms_(max_pulse_ms)
    , cooldown_s_(cooldown_s)
    , settle_delay_ms_(settle_ms)
    , cooldown_until_(start_time + k_startup_blanking) {
    println("[FIRING] Controller initialized: max_pulse={:.0f}ms, cooldown={:.0f}s, "
                 "settle={:.1f}ms", max_pulse_ms_, cooldown_s_, settle_delay_ms_);
}

auto FiringController::may_fire(std::chrono::steady_clock::time_point now) const -> bool {
    // Cooldown / emergency only. Arm is a separate structural gate checked
    // before set_target and before starting a pulse.
    if (emergency_stop_) {
        return false;
    }

    if (now < cooldown_until_) {
        return false;
    }

    return true;
}

auto FiringController::is_firing() const -> bool {
    return pulse_active_;
}

auto FiringController::is_armed() const -> bool {
    return armed_ && !emergency_stop_;
}

auto FiringController::set_armed(bool armed, std::chrono::steady_clock::time_point now)
    -> void {
    if (armed == armed_) {
        return;
    }

    if (!armed) {
        disarm(now);
        return;
    }

    if (emergency_stop_) {
        return;
    }

    armed_ = true;
    println("[FIRING] Armed");
}

auto FiringController::set_target(const Point3D& position,
                                  std::chrono::steady_clock::time_point now) -> void {
    if (emergency_stop_ || !armed_) {
        return;
    }

    if (pulse_active_) {
        abort_active_pulse("Target changed while pulse active, aborting pulse", now);
    }

    bool position_changed = !current_target_.has_value() ||
                            current_target_->x != position.x ||
                            current_target_->y != position.y ||
                            current_target_->z != position.z;

    if (position_changed) {
        galvo_settled_ = false;
        target_just_set_ = true;
    }

    current_target_ = position;
    target_valid_ = true;
}

auto FiringController::clear_target(std::chrono::steady_clock::time_point now) -> void {
    abort_active_pulse("Target lost while pulse active, aborting pulse", now);

    current_target_.reset();
    target_valid_ = false;
    galvo_settled_ = false;
}

auto FiringController::disarm(std::chrono::steady_clock::time_point now) -> void {
    println("[FIRING] Disarming (immediate laser OFF)");

    abort_active_pulse("Disarming while pulse active", now);

    armed_ = false;
    current_target_.reset();
    target_valid_ = false;
    galvo_settled_ = false;

    println("[FIRING] Disarmed, ready for re-arm");
}

auto FiringController::start_cooldown(std::chrono::steady_clock::time_point now) -> void {
    cooldown_until_ = now + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(cooldown_s_));
}

auto FiringController::abort_active_pulse(const char* reason,
                                          std::chrono::steady_clock::time_point now)
    -> void {
    if (!pulse_active_) {
        return;
    }

    println("[FIRING] {}", reason);

    auto result = laser_.fire(false);
    if (!result.has_value()) {
        println(stderr, "[FIRING] Pulse abort laser off failed: {}",
                     to_string(result.error()));
        force_laser_off_and_halt("Pulse abort laser off failed");
        return;
    }
    pulse_active_ = false;

    // Same teardown as end_pulse: an aborted pulse must not leave the galvo
    // marked settled on a target the caller is about to replace.
    galvo_settled_ = false;
    target_valid_ = false;
    current_target_.reset();

    start_cooldown(now);
    println("[FIRING] Cooldown started after pulse abort ({}s)", cooldown_s_);
}

auto FiringController::force_laser_off_and_halt(const char* reason) -> void {
    println(stderr, "[FIRING] {}", reason);
    emergency_stop_ = true;
    pulse_active_ = false;
    target_valid_ = false;
    galvo_settled_ = false;
    armed_ = false;
    current_target_.reset();
    (void)laser_.emergency_shutdown();
}

auto FiringController::end_pulse(std::chrono::steady_clock::time_point now) -> bool {
    println("[FIRING] Max pulse duration ({:.0f}ms) reached, forcing laser OFF",
                 max_pulse_ms_);
    auto off_result = laser_.fire(false);
    if (!off_result.has_value()) {
        println(stderr, "[FIRING] Laser off failed: {}",
                     to_string(off_result.error()));
        force_laser_off_and_halt("Laser off failed after max pulse");
        return false;
    }

    pulse_active_ = false;
    start_cooldown(now);
    println("[FIRING] Cooldown active for {}s", cooldown_s_);
    galvo_settled_ = false;
    target_valid_ = false;
    current_target_.reset();

    return true;
}

auto FiringController::execute_cycle(std::chrono::steady_clock::time_point now) -> bool {
    if (emergency_stop_) {
        auto result = laser_.fire(false);
        if (!result.has_value()) {
            (void)laser_.emergency_shutdown();
        }
        return false;
    }

    // Pulse duration is checked first so a stuck control path still ends the pulse
    // before any galvo motion or new fire attempt.
    if (pulse_active_) {
        auto pulse_duration = std::chrono::duration<double, std::milli>(now - pulse_start_);
        if (pulse_duration.count() >= max_pulse_ms_) {
            return end_pulse(now);
        }
        // Motion blanking: never command galvos while the laser is ON.
        return false;
    }

    if (!enforce_timing_limits(now)) {
        return false;
    }

    if (!armed_) {
        return false;
    }

    if (target_valid_ && current_target_.has_value()) {
        auto dac_result = mapper_.map_to_dac(current_target_.value());

        if (dac_result.has_value()) {
            auto values = dac_result.value();
            auto write_result = galvo_.set_position(values.channel_a, values.channel_b);
            if (!write_result.has_value()) {
                println(stderr, "[FIRING] Galvo set_position failed: {}",
                             to_string(write_result.error()));
                force_laser_off_and_halt("Galvo set_position failed");
                return false;
            }

            if (target_just_set_) {
                galvo_command_time_ = now;
                galvo_settled_ = false;
                target_just_set_ = false;
            }
        } else {
            galvo_settled_ = false;
            target_valid_ = false;
            return false;
        }
    }

    if (!galvo_settled_ && target_valid_) {
        auto elapsed = std::chrono::duration<double, std::milli>(now - galvo_command_time_);
        // Strictly greater: galvo_command_time_ is stamped in this same cycle, so
        // a >= comparison would call a zero-elapsed galvo "settled" and fire while
        // the mirrors are still slewing.
        if (elapsed.count() > settle_delay_ms_) {
            galvo_settled_ = true;
        }
    }

    if (armed_ && target_valid_ && galvo_settled_ && may_fire(now) && !pulse_active_) {
        auto fire_result = laser_.fire(true);
        if (!fire_result.has_value()) {
            println(stderr, "[FIRING] Laser fire failed: {}",
                         to_string(fire_result.error()));
            force_laser_off_and_halt("Laser fire failed");
            return false;
        }

        pulse_active_ = true;
        pulse_start_ = now;
    }

    return false;
}

auto FiringController::enforce_timing_limits(std::chrono::steady_clock::time_point now)
    -> bool {
    if (now < cooldown_until_) {
        auto result = laser_.fire(false);
        if (!result.has_value()) {
            println(stderr, "[FIRING] Cooldown laser off failed: {}",
                         to_string(result.error()));
            force_laser_off_and_halt("Cooldown laser off failed");
            return false;
        }
    }

    return true;
}

auto FiringController::emergency_stop() -> void {
    println(stderr, "[FIRING] EMERGENCY STOP");

    emergency_stop_ = true;
    armed_ = false;
    target_valid_ = false;
    galvo_settled_ = false;
    pulse_active_ = false;
    current_target_.reset();

    auto result = laser_.emergency_shutdown();
    if (!result.has_value()) {
        println(stderr, "[FIRING] Emergency stop: laser shutdown failed: {}",
                     to_string(result.error()));
    }

    println(stderr, "[FIRING] Emergency stop complete, laser OFF");
}
