#include "safety/watchdog.h"
#include "core/error.h"

Watchdog::Watchdog(SystemStateMachine& state_machine,
                   ILaser& laser,
                   IGalvoDriver& galvo,
                   std::chrono::milliseconds timeout,
                   std::chrono::milliseconds startup_grace)
    : state_machine_(state_machine)
    , laser_(laser)
    , galvo_(galvo)
    , timeout_(std::chrono::duration_cast<std::chrono::steady_clock::duration>(timeout))
    , startup_grace_(
          std::chrono::duration_cast<std::chrono::steady_clock::duration>(startup_grace)) {
    println("[WATCHDOG] Initialized, timeout: {}ms, startup grace: {}ms",
            timeout.count(), startup_grace.count());
}

auto Watchdog::feed(std::chrono::steady_clock::time_point heartbeat) -> void {
    auto prev = last_heartbeat_.load(std::memory_order_acquire);

    // Strictly-newer check: the control thread forwards the producer's atomic
    // unconditionally every cycle, so a value that has not advanced means the
    // producer has not run. Accepting it would defeat the timeout entirely.
    while (heartbeat > prev) {
        if (last_heartbeat_.compare_exchange_weak(prev, heartbeat,
                                                  std::memory_order_release,
                                                  std::memory_order_acquire)) {
            return;
        }
    }
}

auto Watchdog::time_since_heartbeat(std::chrono::steady_clock::time_point now) const
    -> std::chrono::steady_clock::duration {
    auto last = last_heartbeat_.load(std::memory_order_acquire);
    if (last == std::chrono::steady_clock::time_point::min()) {
        return std::chrono::steady_clock::duration::max();
    }
    return now - last;
}

auto Watchdog::check(std::chrono::steady_clock::time_point now) -> bool {
    if (triggered_.load(std::memory_order_acquire)) {
        return false;
    }

    auto expected_first = std::chrono::steady_clock::time_point::min();
    first_check_.compare_exchange_strong(expected_first, now,
                                         std::memory_order_release,
                                         std::memory_order_acquire);
    auto first = first_check_.load(std::memory_order_acquire);

    auto last = last_heartbeat_.load(std::memory_order_acquire);

    if (last == std::chrono::steady_clock::time_point::min()) {
        // No heartbeat has ever arrived. Allow a bounded window for the capture
        // and processing threads to come up, then fail closed — an unbounded
        // grace period would mean a producer that never starts is never noticed.
        if (now - first > startup_grace_) {
            println(stderr, "[WATCHDOG] No heartbeat within startup grace ({}ms). "
                    "Triggering SAFE_HALT.",
                    std::chrono::duration_cast<std::chrono::milliseconds>(startup_grace_)
                        .count());
            trigger_safe_halt();
            return false;
        }
        return true;
    }

    auto elapsed = now - last;
    if (elapsed > timeout_) {
        println(stderr, "[WATCHDOG] Heartbeat stale for {}ms (timeout: {}ms). "
                "Triggering SAFE_HALT.",
                std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count(),
                std::chrono::duration_cast<std::chrono::milliseconds>(timeout_).count());
        trigger_safe_halt();
        return false;
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
