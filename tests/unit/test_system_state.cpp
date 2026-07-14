#include <gtest/gtest.h>
#include "safety/system_state.h"

using namespace testing;

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
