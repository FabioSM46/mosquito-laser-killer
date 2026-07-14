#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <chrono>
#include <thread>
#include <memory>
#include <atomic>

#include "mocks/mock_gpio.h"
#include "mocks/mock_dac.h"
#include "mocks/mock_laser.h"
#include "safety/system_state.h"
#include "safety/watchdog.h"
#include "core/error.h"

using namespace testing;
using namespace std::chrono_literals;

class WatchdogJitterStressTest : public Test {
protected:
    void SetUp() override {
        mock_laser_ = std::make_unique<NiceMock<MockLaser>>();
        mock_dac_ = std::make_unique<NiceMock<MockDac>>();
    }

    std::unique_ptr<MockLaser> mock_laser_;
    std::unique_ptr<MockDac> mock_dac_;
};

TEST_F(WatchdogJitterStressTest, TwentyMsJitterDoesNotTrigger) {
    SystemStateMachine sm;
    (void)sm.transition(SystemState::IDLE);

    Watchdog wd(sm, *mock_laser_, *mock_dac_, 3);

    auto t0 = std::chrono::steady_clock::now();
    wd.feed(t0);

    auto result = wd.check(t0 + 20ms);
    EXPECT_TRUE(result);
    EXPECT_EQ(sm.current(), SystemState::IDLE);
}

TEST_F(WatchdogJitterStressTest, ThirtyMsJitterTriggersSafeHalt) {
    SystemStateMachine sm;
    (void)sm.transition(SystemState::IDLE);

    EXPECT_CALL(*mock_laser_, emergency_shutdown())
        .WillOnce(Return(std::expected<void, HardwareError>{}));
    EXPECT_CALL(*mock_dac_, zero())
        .WillOnce(Return(std::expected<void, HardwareError>{}));

    Watchdog wd(sm, *mock_laser_, *mock_dac_, 3);

    auto t0 = std::chrono::steady_clock::now();
    wd.feed(t0);

    auto result = wd.check(t0 + 30ms);
    EXPECT_FALSE(result);
    EXPECT_EQ(sm.current(), SystemState::SAFE_HALT);
}

TEST_F(WatchdogJitterStressTest, FiftyMsJitterDefinitelyTriggersSafeHalt) {
    SystemStateMachine sm;
    (void)sm.transition(SystemState::IDLE);

    EXPECT_CALL(*mock_laser_, emergency_shutdown())
        .WillOnce(Return(std::expected<void, HardwareError>{}));
    EXPECT_CALL(*mock_dac_, zero())
        .WillOnce(Return(std::expected<void, HardwareError>{}));

    Watchdog wd(sm, *mock_laser_, *mock_dac_, 3);

    auto t0 = std::chrono::steady_clock::now();
    wd.feed(t0);

    auto result = wd.check(t0 + 50ms);
    EXPECT_FALSE(result);
    EXPECT_EQ(sm.current(), SystemState::SAFE_HALT);
}

TEST_F(WatchdogJitterStressTest, IntermittentJitterWithFeedsDoesNotTrigger) {
    SystemStateMachine sm;
    (void)sm.transition(SystemState::IDLE);

    Watchdog wd(sm, *mock_laser_, *mock_dac_, 3);

    auto t0 = std::chrono::steady_clock::now();

    wd.feed(t0);
    wd.check(t0 + 9ms);

    wd.feed(t0 + 12ms);
    wd.check(t0 + 21ms);

    wd.feed(t0 + 24ms);
    auto result = wd.check(t0 + 34ms);

    EXPECT_TRUE(result);
    EXPECT_EQ(sm.current(), SystemState::IDLE);
}

TEST_F(WatchdogJitterStressTest, FourthMissTriggersSafeHalt) {
    SystemStateMachine sm;
    (void)sm.transition(SystemState::IDLE);

    EXPECT_CALL(*mock_laser_, emergency_shutdown())
        .WillOnce(Return(std::expected<void, HardwareError>{}));
    EXPECT_CALL(*mock_dac_, zero())
        .WillOnce(Return(std::expected<void, HardwareError>{}));

    Watchdog wd(sm, *mock_laser_, *mock_dac_, 3);

    auto t0 = std::chrono::steady_clock::now();
    wd.feed(t0);

    wd.check(t0 + 9ms);
    wd.check(t0 + 18ms);

    auto result = wd.check(t0 + 27ms);
    EXPECT_FALSE(result);
    EXPECT_EQ(sm.current(), SystemState::SAFE_HALT);
}
