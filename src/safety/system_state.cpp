#include "safety/system_state.h"

auto SystemStateMachine::transition(SystemState target) -> bool {
    auto current_state = state_.load(std::memory_order_acquire);

    // SAFE_HALT terminality is enforced here too: is_valid_transition returns
    // false for the entire SAFE_HALT row, so no separate re-check is needed.
    if (!is_valid_transition(current_state, target)) {
        println(stderr, "[STATE] Invalid transition rejected: {} -> {}",
                     to_string(current_state), to_string(target));
        return false;
    }

    state_.store(target, std::memory_order_release);
    println("[STATE] {} -> {}", to_string(current_state), to_string(target));

    if (target == SystemState::SAFE_HALT) {
        println(stderr, "[STATE] SYSTEM HAS ENTERED SAFE_HALT - RESTART REQUIRED");
    }

    return true;
}
