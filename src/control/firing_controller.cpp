#include "control/firing_controller.h"
#include "core/error.h"
#include <thread>

FiringController::FiringController(ILaser& laser,
                                   IGalvoDriver& galvo,
                                   CoordinateMapper& mapper,
                                   double max_pulse_ms,
                                   double cooldown_s,
                                   double settle_ms)
    : laser_(laser)
    , galvo_(galvo)
    , mapper_(mapper)
    , max_pulse_ms_(max_pulse_ms)
    , cooldown_s_(cooldown_s)
    , settle_delay_ms_(settle_ms) {
    cooldown_until_ = std::chrono::steady_clock::now() + std::chrono::seconds(1);
    println("[FIRING] Controller initialized: max_pulse={:.0f}ms, cooldown={:.0f}s, "
                 "settle={:.1f}ms", max_pulse_ms_, cooldown_s_, settle_delay_ms_);
}

auto FiringController::may_fire() const -> bool {
    auto now = std::chrono::steady_clock::now();

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

auto FiringController::set_target(const Point3D& position) -> void {
    if (emergency_stop_) {
        println(stderr, "[FIRING] Target rejected: emergency stop active");
        return;
    }

    if (pulse_active_) {
        abort_active_pulse("Target changed while pulse active, aborting pulse");
    }

    current_target_ = position;
    target_valid_ = true;
    galvo_settled_ = false;
    target_just_set_ = true;
}

auto FiringController::clear_target() -> void {
    abort_active_pulse("Target lost while pulse active, aborting pulse");

    current_target_.reset();
    target_valid_ = false;
    galvo_settled_ = false;
}

auto FiringController::disarm() -> void {
    println("[FIRING] Disarming (immediate laser OFF)");

    abort_active_pulse("Disarming while pulse active");

    current_target_.reset();
    target_valid_ = false;
    galvo_settled_ = false;

    println("[FIRING] Disarmed, ready for re-arm");
}

auto FiringController::abort_active_pulse(const char* reason) -> void {
    if (!pulse_active_) {
        return;
    }

    println("[FIRING] {}", reason);

    auto result = laser_.fire(false);
    if (!result.has_value()) {
        println(stderr, "[FIRING] Pulse abort laser off failed: {}",
                     to_string(result.error()));
    }
    pulse_active_ = false;

    auto now = std::chrono::steady_clock::now();
    cooldown_until_ = now + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(cooldown_s_));
    println("[FIRING] Cooldown started after pulse abort ({}s)", cooldown_s_);
}

auto FiringController::execute_cycle(std::chrono::steady_clock::time_point now) -> bool {
    if (emergency_stop_) {
        auto result = laser_.fire(false);
        if (!result.has_value()) {
            println(stderr, "[FIRING] Emergency: laser off failed: {}",
                         to_string(result.error()));
        }
        return false;
    }

    if (!enforce_timing_limits(now)) {
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
                emergency_stop_ = true;
                laser_.emergency_shutdown();
                return false;
            }

            if (galvo_settled_ || target_just_set_) {
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
        if (elapsed.count() >= settle_delay_ms_) {
            galvo_settled_ = true;
        }
    }

    if (target_valid_ && galvo_settled_ && may_fire() && !pulse_active_) {
        auto fire_result = laser_.fire(true);
        if (!fire_result.has_value()) {
            println(stderr, "[FIRING] Laser fire failed: {}",
                         to_string(fire_result.error()));
            emergency_stop_ = true;
            laser_.emergency_shutdown();
            return false;
        }

        pulse_active_ = true;
        pulse_start_ = now;
        println("[FIRING] Pulse started at target=({:.3f},{:.3f},{:.3f})",
                     current_target_->x, current_target_->y, current_target_->z);
    }

    if (pulse_active_) {
        auto pulse_duration = std::chrono::duration<double, std::milli>(now - pulse_start_);

        if (pulse_duration.count() >= max_pulse_ms_) {
            println("[FIRING] Max pulse duration ({:.0f}ms) reached, forcing laser OFF",
                         max_pulse_ms_);
            auto off_result = laser_.fire(false);
            if (!off_result.has_value()) {
                println(stderr, "[FIRING] Laser off failed: {}",
                             to_string(off_result.error()));
                emergency_stop_ = true;
                return false;
            }

            pulse_active_ = false;
            cooldown_until_ = now + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<double>(cooldown_s_));
            println("[FIRING] Cooldown active for {}s", cooldown_s_);
            galvo_settled_ = false;
            target_valid_ = false;
            current_target_.reset();

            return true;
        }
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
        }
    }

    return true;
}

auto FiringController::emergency_stop() -> void {
    println(stderr, "[FIRING] EMERGENCY STOP");

    emergency_stop_ = true;
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
