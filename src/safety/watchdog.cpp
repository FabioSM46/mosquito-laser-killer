#include "safety/watchdog.h"
#include "core/error.h"

Watchdog::Watchdog(SystemStateMachine& state_machine,
                   ILaser& laser,
                   IGalvoDriver& galvo,
                   uint32_t missed_threshold)
    : state_machine_(state_machine)
    , laser_(laser)
    , galvo_(galvo)
    , missed_threshold_(missed_threshold) {
    auto now = std::chrono::steady_clock::now();
    last_heartbeat_.store(now, std::memory_order_release);
    println("[WATCHDOG] Initialized, threshold: {} missed cycles", missed_threshold_);
}

auto Watchdog::feed(std::chrono::steady_clock::time_point heartbeat) -> void {
    last_heartbeat_.store(heartbeat, std::memory_order_release);

    auto current_missed = missed_count_.load(std::memory_order_acquire);
    if (current_missed > 0) {
        missed_count_.store(0, std::memory_order_release);
    }
}

auto Watchdog::check(std::chrono::steady_clock::time_point now) -> bool {
    if (triggered_.load(std::memory_order_acquire)) {
        return false;
    }

    auto last = last_heartbeat_.load(std::memory_order_acquire);

    if (last == std::chrono::steady_clock::time_point::min()) {
        return true;
    }

    auto elapsed = now - last;
    auto frame_period = std::chrono::microseconds(8333);
    auto expected_cycles = static_cast<uint32_t>(
        elapsed / frame_period);

    if (expected_cycles > 0) {
        auto current_missed = missed_count_.load(std::memory_order_acquire);
        current_missed += expected_cycles;
        missed_count_.store(current_missed, std::memory_order_release);

        if (current_missed >= missed_threshold_) {
            println(stderr, "[WATCHDOG] Heartbeat lost for {} cycles (threshold: {}). "
                         "Triggering SAFE_HALT.", current_missed, missed_threshold_);
            trigger_safe_halt();
            return false;
        }

        last_heartbeat_.store(now, std::memory_order_release);
    }

    return true;
}

auto Watchdog::trigger_safe_halt() -> void {
    bool expected = false;
    if (!triggered_.compare_exchange_strong(expected, true)) {
        return;
    }

    println(stderr, "[WATCHDOG] Force laser EMERGENCY SHUTDOWN");
    auto laser_result = laser_.emergency_shutdown();
    if (!laser_result.has_value()) {
        println(stderr, "[WATCHDOG] Laser emergency shutdown failed: {}",
                     to_string(laser_result.error()));
    }

    println(stderr, "[WATCHDOG] Commanding galvos to (0V differential)");
    auto galvo_result = galvo_.zero();
    if (!galvo_result.has_value()) {
        println(stderr, "[WATCHDOG] Galvo zero failed: {}",
                     to_string(galvo_result.error()));
    }

    (void)state_machine_.transition(SystemState::SAFE_HALT);
}
