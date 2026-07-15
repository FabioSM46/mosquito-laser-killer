#pragma once

#include "safety/system_state.h"
#include "hal/ilaser.h"
#include "hal/igalvo_driver.h"
#include <chrono>
#include <atomic>
#include "core/print.h"

class Watchdog {
public:
    Watchdog(SystemStateMachine& state_machine,
             ILaser& laser,
             IGalvoDriver& galvo,
             uint32_t missed_threshold = 3,
             int target_fps = 120);

    Watchdog(const Watchdog&) = delete;
    auto operator=(const Watchdog&) -> Watchdog& = delete;

    auto feed(std::chrono::steady_clock::time_point heartbeat) -> void;

    [[nodiscard]] auto check(std::chrono::steady_clock::time_point now) -> bool;

    [[nodiscard]] auto missed_count() const -> uint32_t {
        return missed_count_.load(std::memory_order_acquire);
    }

private:
    SystemStateMachine& state_machine_;
    ILaser& laser_;
    IGalvoDriver& galvo_;
    uint32_t missed_threshold_;
    std::chrono::microseconds frame_period_;

    std::atomic<std::chrono::steady_clock::time_point> last_heartbeat_{
        std::chrono::steady_clock::time_point::min()};
    std::atomic<uint32_t> missed_count_{0};
    std::atomic<bool> triggered_{false};

    auto trigger_safe_halt() -> void;
};
