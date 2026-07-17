#include "control/control_loop.h"
#include "core/print.h"

auto control_step(ControlDeps& deps,
                  const std::optional<TargetCommand>& cmd,
                  std::chrono::steady_clock::time_point heartbeat,
                  std::chrono::steady_clock::time_point now) -> ControlOutcome {
    // Defense in depth: bound the pulse from the HAL even if the sequencer below
    // is wedged. This runs before anything that could return early.
    deps.laser.enforce_max_pulse(now);

    deps.watchdog.feed(heartbeat);
    if (!deps.watchdog.check(now)) {
        println(stderr, "[CONTROL] Watchdog triggered, halting");
        deps.firing_controller.emergency_stop();
        (void)deps.state_machine.transition(SystemState::SAFE_HALT);
        return ControlOutcome::Halt;
    }

    deps.e_stop.update();
    if (deps.e_stop.is_pressed()) {
        println(stderr, "[CONTROL] E-STOP PRESSED, halting");
        deps.firing_controller.emergency_stop();
        (void)deps.state_machine.transition(SystemState::SAFE_HALT);
        return ControlOutcome::Halt;
    }

    deps.arm_switch.update();
    const bool is_armed = deps.arm_switch.is_armed();
    const auto state_before = deps.state_machine.current();

    // Structural arm gate: the controller refuses targets and fire when disarmed.
    deps.firing_controller.set_armed(is_armed, now);

    if (!is_armed && state_before != SystemState::IDLE &&
        state_before != SystemState::SAFE_HALT &&
        state_before != SystemState::COOLDOWN) {
        println("[CONTROL] Arm switch OFF, disarming");
        if (state_before == SystemState::FIRING) {
            (void)deps.state_machine.transition(SystemState::COOLDOWN);
        } else {
            (void)deps.state_machine.transition(SystemState::IDLE);
        }
    }

    if (is_armed && state_before == SystemState::IDLE) {
        println("[CONTROL] Arm switch ON, arming");
        (void)deps.state_machine.transition(SystemState::ARMED);
    }

    if (cmd.has_value()) {
        const auto current_state = deps.state_machine.current();
        const bool has_target = cmd->target_valid && cmd->target_position.has_value();

        if (!is_armed) {
            deps.firing_controller.clear_target(now);
        } else if (current_state == SystemState::FIRING) {
            // Hold aim while firing (motion blanking). Abort only on loss.
            if (!has_target) {
                (void)deps.state_machine.transition(SystemState::COOLDOWN);
                deps.firing_controller.clear_target(now);
            }
        } else if (has_target && (current_state == SystemState::ARMED ||
                                  current_state == SystemState::TRACKING)) {
            if (current_state == SystemState::ARMED) {
                (void)deps.state_machine.transition(SystemState::TRACKING);
            }
            deps.firing_controller.set_target(cmd->target_position.value(), now);
        } else {
            if (current_state == SystemState::TRACKING) {
                (void)deps.state_machine.transition(SystemState::ARMED);
            }
            deps.firing_controller.clear_target(now);
        }
    }

    const bool pulse_was_active = deps.firing_controller.is_firing();
    const bool pulse_ended = deps.firing_controller.execute_cycle(now);

    if (pulse_ended && deps.state_machine.current() == SystemState::FIRING) {
        (void)deps.state_machine.transition(SystemState::COOLDOWN);
    } else if (!pulse_was_active && deps.firing_controller.is_firing() &&
               deps.state_machine.current() == SystemState::TRACKING) {
        (void)deps.state_machine.transition(SystemState::FIRING);
    }

    if (deps.state_machine.current() == SystemState::COOLDOWN &&
        !deps.firing_controller.is_firing() &&
        deps.firing_controller.may_fire(now)) {
        (void)deps.state_machine.transition(SystemState::IDLE);
        if (is_armed) {
            (void)deps.state_machine.transition(SystemState::ARMED);
        }
    }

    // A hardware fault latches the controller off but cannot halt the system by
    // itself — it holds no state machine. Without this poll the loop would spin
    // forever with the laser dead while the operator's readout still said ARMED.
    if (deps.firing_controller.is_halted()) {
        println(stderr, "[CONTROL] Firing controller latched off by a hardware "
                "fault, halting");
        (void)deps.state_machine.transition(SystemState::SAFE_HALT);
        return ControlOutcome::Halt;
    }

    if (deps.state_machine.current() == SystemState::SAFE_HALT) {
        println(stderr, "[CONTROL] SAFE_HALT detected, breaking control loop");
        return ControlOutcome::Halt;
    }

    return ControlOutcome::Continue;
}
