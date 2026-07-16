#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <chrono>
#include <memory>

#include "mocks/mock_gpio.h"
#include "mocks/mock_spi.h"
#include "mocks/mock_dac.h"
#include "mocks/mock_laser.h"
#include "hal/laser.h"
#include "core/error.h"

using namespace testing;

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
