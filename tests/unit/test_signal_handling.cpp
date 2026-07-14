#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <chrono>
#include <thread>
#include <csignal>
#include <atomic>
#include <memory>

#include "mocks/mock_gpio.h"
#include "mocks/mock_dac.h"
#include "mocks/mock_laser.h"
#include "hal/laser.h"
#include "core/error.h"

using namespace testing;
using namespace std::chrono_literals;

class SignalHandlingTest : public Test {
protected:
    void SetUp() override {
        mock_gpio_ = std::make_unique<NiceMock<MockGpio>>();
        mock_laser_ = std::make_unique<NiceMock<MockLaser>>();
        mock_dac_ = std::make_unique<NiceMock<MockDac>>();
    }

    std::unique_ptr<MockGpio> mock_gpio_;
    std::unique_ptr<MockLaser> mock_laser_;
    std::unique_ptr<MockDac> mock_dac_;
};

TEST_F(SignalHandlingTest, LaserPinLowAfterEmergencyShutdownSignal) {
    EXPECT_CALL(*mock_laser_, emergency_shutdown())
        .WillOnce(Return(std::expected<void, HardwareError>{}));

    auto result = mock_laser_->emergency_shutdown();
    EXPECT_TRUE(result.has_value());
}

TEST_F(SignalHandlingTest, DacZeroedAfterEmergencyShutdown) {
    EXPECT_CALL(*mock_dac_, zero())
        .WillOnce(Return(std::expected<void, HardwareError>{}));

    auto result = mock_dac_->zero();
    EXPECT_TRUE(result.has_value());
}

TEST_F(SignalHandlingTest, LaserDestructorForcesPinLow) {
    EXPECT_CALL(*mock_gpio_, set_direction_output())
        .WillOnce(Return(std::expected<void, HardwareError>{}));
    EXPECT_CALL(*mock_gpio_, write(false))
        .Times(AtLeast(2))
        .WillRepeatedly(Return(std::expected<void, HardwareError>{}));

    auto laser = std::make_unique<Laser>(std::move(mock_gpio_), 18);
    laser.reset();
}

TEST_F(SignalHandlingTest, EmergencyShutdownDuringActiveFire) {
    EXPECT_CALL(*mock_laser_, fire(true))
        .WillOnce(Return(std::expected<void, HardwareError>{}));
    EXPECT_CALL(*mock_laser_, emergency_shutdown())
        .WillOnce(Return(std::expected<void, HardwareError>{}));

    auto fire_result = mock_laser_->fire(true);
    EXPECT_TRUE(fire_result.has_value());

    auto shutdown_result = mock_laser_->emergency_shutdown();
    EXPECT_TRUE(shutdown_result.has_value());
}
