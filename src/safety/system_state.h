#pragma once

#include <cstdint>
#include <string_view>
#include <atomic>
#include "core/print.h"

enum class SystemState : uint8_t {
    INIT = 0,
    IDLE,
    ARMED,
    TRACKING,
    FIRING,
    COOLDOWN,
    SAFE_HALT,
};

[[nodiscard]] constexpr auto to_string(SystemState s) -> std::string_view {
    switch (s) {
    case SystemState::INIT:      return "INIT";
    case SystemState::IDLE:      return "IDLE";
    case SystemState::ARMED:     return "ARMED";
    case SystemState::TRACKING:  return "TRACKING";
    case SystemState::FIRING:    return "FIRING";
    case SystemState::COOLDOWN:  return "COOLDOWN";
    case SystemState::SAFE_HALT: return "SAFE_HALT";
    }
    return "UNKNOWN";
}

class SystemStateMachine {
public:
    SystemStateMachine() {
        state_.store(SystemState::INIT, std::memory_order_release);
    }

    SystemStateMachine(const SystemStateMachine&) = delete;
    auto operator=(const SystemStateMachine&) -> SystemStateMachine& = delete;

    [[nodiscard]] auto current() const -> SystemState {
        return state_.load(std::memory_order_acquire);
    }

    [[nodiscard]] auto transition(SystemState target) -> bool;

private:
    std::atomic<SystemState> state_{SystemState::INIT};

    [[nodiscard]] static auto is_valid_transition(SystemState from, SystemState to) -> bool {
        switch (from) {
        case SystemState::INIT:
            return to == SystemState::IDLE || to == SystemState::SAFE_HALT;
        case SystemState::IDLE:
            return to == SystemState::ARMED || to == SystemState::SAFE_HALT;
        case SystemState::ARMED:
            return to == SystemState::TRACKING || to == SystemState::IDLE ||
                   to == SystemState::SAFE_HALT;
        case SystemState::TRACKING:
            // IDLE allowed for arm-switch disarm (operator gate).
            return to == SystemState::FIRING || to == SystemState::ARMED ||
                   to == SystemState::IDLE || to == SystemState::SAFE_HALT;
        case SystemState::FIRING:
            return to == SystemState::COOLDOWN || to == SystemState::SAFE_HALT;
        case SystemState::COOLDOWN:
            return to == SystemState::IDLE || to == SystemState::SAFE_HALT;
        case SystemState::SAFE_HALT:
            return false;
        }
        return false;
    }
};
