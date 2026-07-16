#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <chrono>
#include <memory>

#include "mocks/mock_gpio.h"
#include "mocks/mock_galvo_driver.h"
#include "mocks/mock_laser.h"
#include "safety/system_state.h"
#include "safety/watchdog.h"
#include "hal/laser.h"
#include "core/error.h"

using namespace testing;
using namespace std::chrono_literals;

class WatchdogTest : public Test {
protected:
    void SetUp() override {
        mock_laser_ = std::make_unique<NiceMock<MockLaser>>();
        mock_galvo_ = std::make_unique<NiceMock<MockGalvoDriver>>();
    }

    std::unique_ptr<MockLaser> mock_laser_;
    std::unique_ptr<MockGalvoDriver> mock_galvo_;
};

TEST_F(WatchdogTest, HealthyHeartbeatDoesNotTrigger) {
    SystemStateMachine sm;
    sm.transition(SystemState::IDLE);

    Watchdog wd(sm, *mock_laser_, *mock_galvo_, 3);

    auto now = std::chrono::steady_clock::now();
    wd.feed(now);

    auto result = wd.check(now + 8ms);
    EXPECT_TRUE(result);
    EXPECT_EQ(sm.current(), SystemState::IDLE);
}

TEST_F(WatchdogTest, ThreeMissedCyclesTriggersSafeHalt) {
    SystemStateMachine sm;
    sm.transition(SystemState::IDLE);

    EXPECT_CALL(*mock_laser_, emergency_shutdown())
        .WillOnce(Return(std::expected<void, HardwareError>{}));
    EXPECT_CALL(*mock_galvo_, zero())
        .WillOnce(Return(std::expected<void, HardwareError>{}));

    Watchdog wd(sm, *mock_laser_, *mock_galvo_, 3);

    auto initial_time = std::chrono::steady_clock::now();
    wd.feed(initial_time);

    auto result = wd.check(initial_time + 25ms);
    EXPECT_FALSE(result);
    EXPECT_EQ(sm.current(), SystemState::SAFE_HALT);
}

TEST_F(WatchdogTest, OneOrTwoMissedCyclesDoesNotTrigger) {
    SystemStateMachine sm;
    sm.transition(SystemState::IDLE);

    Watchdog wd(sm, *mock_laser_, *mock_galvo_, 3);

    auto now = std::chrono::steady_clock::now();
    wd.feed(now);

    auto result1 = wd.check(now + 16ms);
    EXPECT_TRUE(result1);
    EXPECT_EQ(sm.current(), SystemState::IDLE);
}

TEST_F(WatchdogTest, FeedResetsMissedCount) {
    SystemStateMachine sm;
    (void)sm.transition(SystemState::IDLE);

    Watchdog wd(sm, *mock_laser_, *mock_galvo_, 3);

    auto t0 = std::chrono::steady_clock::now();
    wd.feed(t0);

    wd.check(t0 + 17ms);
    EXPECT_EQ(wd.missed_count(), 2);

    wd.feed(t0 + 20ms);
    EXPECT_EQ(wd.missed_count(), 0);

    auto result = wd.check(t0 + 30ms);
    EXPECT_TRUE(result);
}

TEST_F(WatchdogTest, SafeHaltIsIrreversible) {
    SystemStateMachine sm;
    sm.transition(SystemState::IDLE);

    EXPECT_CALL(*mock_laser_, emergency_shutdown())
        .WillOnce(Return(std::expected<void, HardwareError>{}));
    EXPECT_CALL(*mock_galvo_, zero())
        .WillOnce(Return(std::expected<void, HardwareError>{}));

    Watchdog wd(sm, *mock_laser_, *mock_galvo_, 3);

    auto now = std::chrono::steady_clock::now();
    wd.feed(now);
    wd.check(now + 25ms);

    EXPECT_EQ(sm.current(), SystemState::SAFE_HALT);

    EXPECT_FALSE(sm.transition(SystemState::IDLE));
    EXPECT_EQ(sm.current(), SystemState::SAFE_HALT);
}

TEST_F(WatchdogTest, CheckReturnsFalseWhenAlreadyTriggered) {
    SystemStateMachine sm;
    sm.transition(SystemState::IDLE);

    EXPECT_CALL(*mock_laser_, emergency_shutdown())
        .WillOnce(Return(std::expected<void, HardwareError>{}));
    EXPECT_CALL(*mock_galvo_, zero())
        .WillOnce(Return(std::expected<void, HardwareError>{}));

    Watchdog wd(sm, *mock_laser_, *mock_galvo_, 3);

    auto now = std::chrono::steady_clock::now();
    wd.feed(now);
    wd.check(now + 25ms);

    auto second_result = wd.check(now + 50ms);
    EXPECT_FALSE(second_result);
}
