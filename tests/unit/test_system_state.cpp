#include <gtest/gtest.h>
#include <array>
#include <vector>
#include <algorithm>
#include "safety/system_state.h"

using namespace testing;

namespace {

constexpr std::array k_all_states{
    SystemState::INIT,     SystemState::IDLE,     SystemState::ARMED,
    SystemState::TRACKING, SystemState::FIRING,   SystemState::COOLDOWN,
    SystemState::SAFE_HALT,
};

// The complete transition table, written out independently of the switch in
// system_state.h. Enumerating it here rather than deriving it means a change to
// the implementation has to be justified against an explicit list, instead of the
// test silently agreeing with whatever the code now does.
auto allowed_targets(SystemState from) -> std::vector<SystemState> {
    switch (from) {
    case SystemState::INIT:
        return {SystemState::IDLE, SystemState::SAFE_HALT};
    case SystemState::IDLE:
        return {SystemState::ARMED, SystemState::SAFE_HALT};
    case SystemState::ARMED:
        return {SystemState::TRACKING, SystemState::IDLE, SystemState::SAFE_HALT};
    case SystemState::TRACKING:
        return {SystemState::FIRING, SystemState::ARMED, SystemState::IDLE,
                SystemState::SAFE_HALT};
    case SystemState::FIRING:
        return {SystemState::COOLDOWN, SystemState::SAFE_HALT};
    case SystemState::COOLDOWN:
        return {SystemState::IDLE, SystemState::SAFE_HALT};
    case SystemState::SAFE_HALT:
        return {};  // terminal
    }
    return {};
}

// Walks the machine into `target` using only legal transitions.
auto drive_to(SystemStateMachine& sm, SystemState target) -> bool {
    switch (target) {
    case SystemState::INIT:
        return sm.current() == SystemState::INIT;
    case SystemState::IDLE:
        return sm.transition(SystemState::IDLE);
    case SystemState::ARMED:
        return sm.transition(SystemState::IDLE) && sm.transition(SystemState::ARMED);
    case SystemState::TRACKING:
        return drive_to(sm, SystemState::ARMED) && sm.transition(SystemState::TRACKING);
    case SystemState::FIRING:
        return drive_to(sm, SystemState::TRACKING) && sm.transition(SystemState::FIRING);
    case SystemState::COOLDOWN:
        return drive_to(sm, SystemState::FIRING) && sm.transition(SystemState::COOLDOWN);
    case SystemState::SAFE_HALT:
        return sm.transition(SystemState::SAFE_HALT);
    }
    return false;
}

}

// Exhaustive: every one of the 7x7 (from, to) pairs, so no transition is covered
// by accident and none is left untested. The previous suite checked 2 of ~20
// invalid transitions and missed ARMED -> FIRING — skipping TRACKING entirely,
// which is the one transition that would let the system fire without ever having
// confirmed a target.
TEST(SystemStateMachineTest, EveryTransitionMatchesTheTable) {
    for (const auto from : k_all_states) {
        const auto allowed = allowed_targets(from);

        for (const auto to : k_all_states) {
            SystemStateMachine sm;
            ASSERT_TRUE(drive_to(sm, from))
                << "could not reach " << to_string(from);
            ASSERT_EQ(sm.current(), from);

            const bool expected =
                std::find(allowed.begin(), allowed.end(), to) != allowed.end();
            const bool actual = sm.transition(to);

            EXPECT_EQ(actual, expected)
                << to_string(from) << " -> " << to_string(to)
                << ": expected " << (expected ? "allowed" : "rejected")
                << ", got " << (actual ? "allowed" : "rejected");

            // A rejected transition must not move the machine.
            EXPECT_EQ(sm.current(), expected ? to : from)
                << "state changed unexpectedly on "
                << to_string(from) << " -> " << to_string(to);
        }
    }
}

TEST(SystemStateMachineTest, EveryStateCanReachSafeHaltExceptSafeHaltItself) {
    for (const auto from : k_all_states) {
        if (from == SystemState::SAFE_HALT) {
            continue;
        }
        SystemStateMachine sm;
        ASSERT_TRUE(drive_to(sm, from)) << "could not reach " << to_string(from);

        EXPECT_TRUE(sm.transition(SystemState::SAFE_HALT))
            << "SAFE_HALT unreachable from " << to_string(from);
        EXPECT_EQ(sm.current(), SystemState::SAFE_HALT);
    }
}

TEST(SystemStateMachineTest, InitialStateIsInit) {
    SystemStateMachine sm;
    EXPECT_EQ(sm.current(), SystemState::INIT);
}

TEST(SystemStateMachineTest, ValidTransitionInitToIdle) {
    SystemStateMachine sm;
    EXPECT_TRUE(sm.transition(SystemState::IDLE));
    EXPECT_EQ(sm.current(), SystemState::IDLE);
}

TEST(SystemStateMachineTest, ValidTransitionIdleToArmed) {
    SystemStateMachine sm;
    (void)sm.transition(SystemState::IDLE);
    EXPECT_TRUE(sm.transition(SystemState::ARMED));
    EXPECT_EQ(sm.current(), SystemState::ARMED);
}

TEST(SystemStateMachineTest, ValidTransitionArmedToTracking) {
    SystemStateMachine sm;
    (void)sm.transition(SystemState::IDLE);
    (void)sm.transition(SystemState::ARMED);
    EXPECT_TRUE(sm.transition(SystemState::TRACKING));
    EXPECT_EQ(sm.current(), SystemState::TRACKING);
}

TEST(SystemStateMachineTest, ValidTransitionTrackingToFiring) {
    SystemStateMachine sm;
    (void)sm.transition(SystemState::IDLE);
    (void)sm.transition(SystemState::ARMED);
    (void)sm.transition(SystemState::TRACKING);
    EXPECT_TRUE(sm.transition(SystemState::FIRING));
    EXPECT_EQ(sm.current(), SystemState::FIRING);
}

TEST(SystemStateMachineTest, ValidTransitionFiringToCooldown) {
    SystemStateMachine sm;
    (void)sm.transition(SystemState::IDLE);
    (void)sm.transition(SystemState::ARMED);
    (void)sm.transition(SystemState::TRACKING);
    (void)sm.transition(SystemState::FIRING);
    EXPECT_TRUE(sm.transition(SystemState::COOLDOWN));
    EXPECT_EQ(sm.current(), SystemState::COOLDOWN);
}

TEST(SystemStateMachineTest, ValidTransitionCooldownToIdle) {
    SystemStateMachine sm;
    (void)sm.transition(SystemState::IDLE);
    (void)sm.transition(SystemState::ARMED);
    (void)sm.transition(SystemState::TRACKING);
    (void)sm.transition(SystemState::FIRING);
    (void)sm.transition(SystemState::COOLDOWN);
    EXPECT_TRUE(sm.transition(SystemState::IDLE));
    EXPECT_EQ(sm.current(), SystemState::IDLE);
}

TEST(SystemStateMachineTest, DirectInitToArmedIsInvalid) {
    SystemStateMachine sm;
    EXPECT_FALSE(sm.transition(SystemState::ARMED));
    EXPECT_EQ(sm.current(), SystemState::INIT);
}

TEST(SystemStateMachineTest, DirectIdleToFiringIsInvalid) {
    SystemStateMachine sm;
    (void)sm.transition(SystemState::IDLE);
    EXPECT_FALSE(sm.transition(SystemState::FIRING));
    EXPECT_EQ(sm.current(), SystemState::IDLE);
}

TEST(SystemStateMachineTest, AnyStateCanTransitionToSafeHalt) {
    SystemStateMachine sm;

    (void)sm.transition(SystemState::IDLE);
    (void)sm.transition(SystemState::ARMED);
    EXPECT_TRUE(sm.transition(SystemState::SAFE_HALT));
    EXPECT_EQ(sm.current(), SystemState::SAFE_HALT);
}

TEST(SystemStateMachineTest, SafeHaltIsIrreversible) {
    SystemStateMachine sm;
    (void)sm.transition(SystemState::IDLE);
    (void)sm.transition(SystemState::SAFE_HALT);
    EXPECT_EQ(sm.current(), SystemState::SAFE_HALT);

    EXPECT_FALSE(sm.transition(SystemState::IDLE));
    EXPECT_FALSE(sm.transition(SystemState::ARMED));
    EXPECT_FALSE(sm.transition(SystemState::TRACKING));
    EXPECT_FALSE(sm.transition(SystemState::FIRING));
    EXPECT_FALSE(sm.transition(SystemState::COOLDOWN));
    EXPECT_FALSE(sm.transition(SystemState::INIT));

    EXPECT_EQ(sm.current(), SystemState::SAFE_HALT);
}

TEST(SystemStateMachineTest, TrackingToArmedIsValid) {
    SystemStateMachine sm;
    (void)sm.transition(SystemState::IDLE);
    (void)sm.transition(SystemState::ARMED);
    (void)sm.transition(SystemState::TRACKING);
    EXPECT_TRUE(sm.transition(SystemState::ARMED));
    EXPECT_EQ(sm.current(), SystemState::ARMED);
}

TEST(SystemStateMachineTest, TrackingToIdleIsValid) {
    SystemStateMachine sm;
    (void)sm.transition(SystemState::IDLE);
    (void)sm.transition(SystemState::ARMED);
    (void)sm.transition(SystemState::TRACKING);
    EXPECT_TRUE(sm.transition(SystemState::IDLE));
    EXPECT_EQ(sm.current(), SystemState::IDLE);
}
