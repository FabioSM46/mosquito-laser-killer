#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <chrono>
#include <memory>
#include <thread>

#include "mocks/mock_gpio.h"
#include "hal/laser.h"
#include "core/error.h"

using namespace testing;

namespace {
constexpr auto kOk = std::expected<void, HardwareError>{};
constexpr auto kWriteFail =
    std::unexpected(HardwareError::GpioWriteFailed);
}

class LaserSafetyTest : public Test {
protected:
    void SetUp() override {
        mock_gpio_ = std::make_unique<MockGpio>();
    }

    std::unique_ptr<MockGpio> mock_gpio_;
};

TEST_F(LaserSafetyTest, ConstructorForcesPinLow) {
    EXPECT_CALL(*mock_gpio_, set_direction_output())
        .WillOnce(Return(std::expected<void, HardwareError>{}));
    EXPECT_CALL(*mock_gpio_, write(false))
        .Times(2)
        .WillRepeatedly(Return(std::expected<void, HardwareError>{}));

    auto laser = std::make_unique<Laser>(std::move(mock_gpio_), 18);

    EXPECT_FALSE(laser->is_firing());
}

TEST_F(LaserSafetyTest, ConstructorHandlesDirectionFailure) {
    EXPECT_CALL(*mock_gpio_, set_direction_output())
        .WillOnce(Return(
            std::unexpected(HardwareError::GpioOpenFailed)));

    auto laser = std::make_unique<Laser>(std::move(mock_gpio_), 18);

    auto result = laser->fire(true);
    EXPECT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), HardwareError::LaserEmergencyShutdown);
}

TEST_F(LaserSafetyTest, ConstructorHandlesInitWriteFailure) {
    EXPECT_CALL(*mock_gpio_, set_direction_output())
        .WillOnce(Return(std::expected<void, HardwareError>{}));
    EXPECT_CALL(*mock_gpio_, write(false))
        .WillOnce(Return(
            std::unexpected(HardwareError::GpioWriteFailed)))
        .WillOnce(Return(std::expected<void, HardwareError>{}));

    auto laser = std::make_unique<Laser>(std::move(mock_gpio_), 18);

    auto result = laser->fire(true);
    EXPECT_FALSE(result.has_value());
}

TEST_F(LaserSafetyTest, FireEnablesPin) {
    EXPECT_CALL(*mock_gpio_, set_direction_output())
        .WillOnce(Return(std::expected<void, HardwareError>{}));
    EXPECT_CALL(*mock_gpio_, write(false))
        .Times(3)
        .WillRepeatedly(Return(std::expected<void, HardwareError>{}));
    EXPECT_CALL(*mock_gpio_, write(true))
        .WillOnce(Return(std::expected<void, HardwareError>{}));

    auto laser = std::make_unique<Laser>(std::move(mock_gpio_), 18);

    auto result = laser->fire(true);
    EXPECT_TRUE(result.has_value());
    EXPECT_TRUE(laser->is_firing());

    auto off_result = laser->fire(false);
    EXPECT_TRUE(off_result.has_value());
    EXPECT_FALSE(laser->is_firing());
}

TEST_F(LaserSafetyTest, EmergencyShutdownForcesPinLow) {
    EXPECT_CALL(*mock_gpio_, set_direction_output())
        .WillOnce(Return(std::expected<void, HardwareError>{}));
    EXPECT_CALL(*mock_gpio_, write(false))
        .Times(AtLeast(2))
        .WillRepeatedly(Return(std::expected<void, HardwareError>{}));

    auto laser = std::make_unique<Laser>(std::move(mock_gpio_), 18);

    auto result = laser->emergency_shutdown();
    EXPECT_TRUE(result.has_value());

    auto fire_result = laser->fire(true);
    EXPECT_FALSE(fire_result.has_value());
    EXPECT_EQ(fire_result.error(), HardwareError::LaserEmergencyShutdown);
}

TEST_F(LaserSafetyTest, FireWriteFailureTriggersEmergencyShutdown) {
    EXPECT_CALL(*mock_gpio_, set_direction_output())
        .WillOnce(Return(std::expected<void, HardwareError>{}));
    EXPECT_CALL(*mock_gpio_, write(false))
        .WillOnce(Return(std::expected<void, HardwareError>{}))
        .WillOnce(Return(std::expected<void, HardwareError>{}))
        .WillOnce(Return(std::expected<void, HardwareError>{}));
    EXPECT_CALL(*mock_gpio_, write(true))
        .WillOnce(Return(
            std::unexpected(HardwareError::GpioWriteFailed)));

    auto laser = std::make_unique<Laser>(std::move(mock_gpio_), 18);

    auto result = laser->fire(true);
    EXPECT_FALSE(result.has_value());

    auto second_result = laser->fire(false);
    EXPECT_TRUE(second_result.has_value());
    EXPECT_FALSE(laser->is_firing());
}

TEST_F(LaserSafetyTest, EnforceMaxPulseForcesPinLow) {
    EXPECT_CALL(*mock_gpio_, set_direction_output())
        .WillOnce(Return(std::expected<void, HardwareError>{}));
    EXPECT_CALL(*mock_gpio_, write(false))
        .Times(AtLeast(2))
        .WillRepeatedly(Return(std::expected<void, HardwareError>{}));
    EXPECT_CALL(*mock_gpio_, write(true))
        .WillOnce(Return(std::expected<void, HardwareError>{}));

    auto laser = std::make_unique<Laser>(std::move(mock_gpio_), 18, 10.0);

    ASSERT_TRUE(laser->fire(true).has_value());
    EXPECT_TRUE(laser->is_firing());

    auto later = std::chrono::steady_clock::now() + std::chrono::milliseconds(20);
    laser->enforce_max_pulse(later);
    EXPECT_FALSE(laser->is_firing());
}

// §4.1 names three software mechanisms that can end a pulse; this is
// Laser::fire()'s own re-entry check. A second fire(true) after the limit has
// elapsed must force the pin LOW and refuse — never extend the pulse.
// fire() reads its own clock, so this test spends real time; sleep_for
// guarantees AT LEAST the requested time, which is the safe direction against
// a 10ms limit.
TEST_F(LaserSafetyTest, ReentrantFireBeyondMaxPulseForcesPinLow) {
    EXPECT_CALL(*mock_gpio_, set_direction_output()).WillOnce(Return(kOk));
    EXPECT_CALL(*mock_gpio_, write(false))
        .Times(AtLeast(2))
        .WillRepeatedly(Return(kOk));
    EXPECT_CALL(*mock_gpio_, write(true)).WillOnce(Return(kOk));

    auto laser = std::make_unique<Laser>(std::move(mock_gpio_), 18, 10.0);
    ASSERT_TRUE(laser->fire(true).has_value());

    std::this_thread::sleep_for(std::chrono::milliseconds(15));

    auto result = laser->fire(true);
    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), HardwareError::LaserEmergencyShutdown);
    EXPECT_FALSE(laser->is_firing());
}

// If the forced-OFF write of the re-entry check fails, the pin may still be
// HIGH: is_firing() must keep saying so, and the laser must latch emergency
// shutdown rather than carry on.
TEST_F(LaserSafetyTest, ReentrantMaxPulseWriteFailureKeepsFiringFlagAndLatches) {
    EXPECT_CALL(*mock_gpio_, set_direction_output()).WillOnce(Return(kOk));
    EXPECT_CALL(*mock_gpio_, write(false))
        .Times(3)                       // ctor, failed forced-OFF, destructor
        .WillOnce(Return(kOk))
        .WillOnce(Return(kWriteFail))
        .WillOnce(Return(kOk));
    EXPECT_CALL(*mock_gpio_, write(true)).WillOnce(Return(kOk));

    auto laser = std::make_unique<Laser>(std::move(mock_gpio_), 18, 10.0);
    ASSERT_TRUE(laser->fire(true).has_value());

    std::this_thread::sleep_for(std::chrono::milliseconds(15));

    auto result = laser->fire(true);
    ASSERT_FALSE(result.has_value());
    EXPECT_TRUE(laser->is_firing()) << "pin state unknown must not read as OFF";

    auto again = laser->fire(true);
    ASSERT_FALSE(again.has_value());
    EXPECT_EQ(again.error(), HardwareError::LaserEmergencyShutdown);
}

TEST_F(LaserSafetyTest, EnforceMaxPulseWriteFailureKeepsFiringFlagAndLatches) {
    EXPECT_CALL(*mock_gpio_, set_direction_output()).WillOnce(Return(kOk));
    EXPECT_CALL(*mock_gpio_, write(false))
        .Times(3)                       // ctor, failed forced-OFF, destructor
        .WillOnce(Return(kOk))
        .WillOnce(Return(kWriteFail))
        .WillOnce(Return(kOk));
    EXPECT_CALL(*mock_gpio_, write(true)).WillOnce(Return(kOk));

    auto laser = std::make_unique<Laser>(std::move(mock_gpio_), 18, 10.0);
    ASSERT_TRUE(laser->fire(true).has_value());

    laser->enforce_max_pulse(std::chrono::steady_clock::now() +
                             std::chrono::milliseconds(20));

    EXPECT_TRUE(laser->is_firing()) << "pin state unknown must not read as OFF";
    auto result = laser->fire(true);
    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), HardwareError::LaserEmergencyShutdown);
}

// The firing controller re-forces fire(false) every control cycle through
// cooldown and startup blanking. The log went transition-only, but the WRITE
// must remain unconditional belt-and-braces against a latched pin.
TEST_F(LaserSafetyTest, RepeatedFireOffStillWritesTheDefensiveLow) {
    EXPECT_CALL(*mock_gpio_, set_direction_output()).WillOnce(Return(kOk));
    EXPECT_CALL(*mock_gpio_, write(false))
        .Times(4)                       // ctor + two explicit OFFs + destructor
        .WillRepeatedly(Return(kOk));

    auto laser = std::make_unique<Laser>(std::move(mock_gpio_), 18);
    ASSERT_TRUE(laser->fire(false).has_value());
    ASSERT_TRUE(laser->fire(false).has_value());
}
