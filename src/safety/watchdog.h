#pragma once

#include "safety/system_state.h"
#include "hal/ilaser.h"
#include "hal/igalvo_driver.h"
#include <chrono>
#include <atomic>
#include "core/print.h"

// Heartbeat watchdog for the processing thread.
//
// The timeout is an ABSOLUTE duration, deliberately independent of target_fps:
// target_fps is a performance knob, and deriving the safety timeout from it means
// raising the frame rate silently tightens a Class 4 interlock.
class Watchdog {
public:
    static constexpr auto k_default_timeout = std::chrono::milliseconds(25);
    static constexpr auto k_default_startup_grace = std::chrono::milliseconds(5000);

    Watchdog(SystemStateMachine& state_machine,
             ILaser& laser,
             IGalvoDriver& galvo,
             std::chrono::milliseconds timeout = k_default_timeout,
             std::chrono::milliseconds startup_grace = k_default_startup_grace);

    Watchdog(const Watchdog&) = delete;
    auto operator=(const Watchdog&) -> Watchdog& = delete;

    // Accepts a heartbeat only if it is strictly newer than the last one seen.
    // The control thread forwards the same atomic every cycle, so re-feeding an
    // unchanged value must not reset the timer — otherwise a producer that never
    // ran would keep the watchdog alive forever.
    auto feed(std::chrono::steady_clock::time_point heartbeat) -> void;

    [[nodiscard]] auto check(std::chrono::steady_clock::time_point now) -> bool;

    // steady_clock::duration::max() until the first real heartbeat arrives.
    [[nodiscard]] auto time_since_heartbeat(
        std::chrono::steady_clock::time_point now) const -> std::chrono::steady_clock::duration;

    [[nodiscard]] auto has_heartbeat() const -> bool {
        return last_heartbeat_.load(std::memory_order_acquire) !=
               std::chrono::steady_clock::time_point::min();
    }

    [[nodiscard]] auto triggered() const -> bool {
        return triggered_.load(std::memory_order_acquire);
    }

private:
    SystemStateMachine& state_machine_;
    ILaser& laser_;
    IGalvoDriver& galvo_;
    std::chrono::steady_clock::duration timeout_;
    std::chrono::steady_clock::duration startup_grace_;

    std::atomic<std::chrono::steady_clock::time_point> last_heartbeat_{
        std::chrono::steady_clock::time_point::min()};
    // Stamped on the first check() so the grace window is driven by the injected
    // clock rather than construction time.
    std::atomic<std::chrono::steady_clock::time_point> first_check_{
        std::chrono::steady_clock::time_point::min()};
    std::atomic<bool> triggered_{false};

    auto trigger_safe_halt() -> void;
};
