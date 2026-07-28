#pragma once

#include "core/types.h"
#include "hal/ilaser.h"
#include "safety/system_state.h"
#include "safety/watchdog.h"
#include "safety/arm_switch.h"
#include "safety/e_stop.h"
#include "control/firing_controller.h"
#include <chrono>
#include <optional>

// One iteration of the control thread, extracted from main so the ordering of the
// safety checks is testable. Previously this logic lived inline in a lambda in
// main(), and the test suite hand-copied it — which meant the copy, not the real
// loop, was what got tested, and the two had already drifted apart.
struct ControlDeps {
    SystemStateMachine& state_machine;
    FiringController& firing_controller;
    Watchdog& watchdog;
    ArmSwitch& arm_switch;
    EStop& e_stop;
    ILaser& laser;
};

enum class ControlOutcome {
    Continue,
    Halt,
};

// Fixed control-thread period, deliberately NOT derived from target_fps. The
// control cycle bounds the max-pulse overshoot (§4.1's "+ one control cycle"),
// the e-stop/arm debounce wall time (debounce cycles × period), and the
// watchdog poll cadence — and a performance knob must never retune a safety
// interlock (§4.4; the watchdog timeout was decoupled for the same reason).
// 5 ms keeps the real pulse bound at ~105 ms and the e-stop debounce at
// 3 × 5 = 15 ms regardless of the configured camera rate.
inline constexpr auto k_control_period = std::chrono::microseconds(5000);

// Runs the guards in their required order:
//   enforce_max_pulse -> watchdog -> e-stop -> arm -> target -> execute -> halt check
//
// `cmd` is the freshest target command, or nullopt if none arrived this cycle.
// `heartbeat` is the producer's last-published timestamp. `now` drives every
// timing decision; nothing here reads a clock of its own.
[[nodiscard]] auto control_step(ControlDeps& deps,
                                const std::optional<TargetCommand>& cmd,
                                std::chrono::steady_clock::time_point heartbeat,
                                std::chrono::steady_clock::time_point now)
    -> ControlOutcome;
