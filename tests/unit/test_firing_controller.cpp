#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <chrono>
#include <thread>
#include <memory>

#include "mocks/mock_gpio.h"
#include "mocks/mock_spi.h"
#include "mocks/mock_dac.h"
#include "mocks/mock_laser.h"
#include "core/types.h"
#include "core/error.h"
#include "safety/bounding_box.h"
#include "control/coordinate_mapper.h"
#include "control/firing_controller.h"

using namespace testing;
using namespace std::chrono_literals;

class FiringControllerTest : public Test {
protected:
    void SetUp() override {
        mock_laser_ = std::make_unique<NiceMock<MockLaser>>();
        mock_dac_ = std::make_unique<NiceMock<MockDac>>();

        SystemConfig::BoundingBox bb;
        bb.x_min = -2.0;
        bb.x_max = 2.0;
        bb.y_min = -2.0;
        bb.y_max = 2.0;
        bb.z_min = 0.1;
        bb.z_max = 10.0;

        SystemConfig::GalvoLimits gl;
        gl.angle_x_min_deg = -25.0;
        gl.angle_x_max_deg = 25.0;
        gl.angle_y_min_deg = -25.0;
        gl.angle_y_max_deg = 25.0;

        bbox_ = std::make_unique<BoundingBox3D>(bb);
        mapper_ = std::make_unique<CoordinateMapper>(*bbox_, gl);
        controller_ = std::make_unique<FiringController>(
            *mock_laser_, *mock_dac_, *mapper_,
            100.0, 10.0, 3.0);
    }

    std::unique_ptr<MockLaser> mock_laser_;
    std::unique_ptr<MockDac> mock_dac_;
    std::unique_ptr<BoundingBox3D> bbox_;
    std::unique_ptr<CoordinateMapper> mapper_;
    std::unique_ptr<FiringController> controller_;
};

TEST_F(FiringControllerTest, MayFireInitiallyFalseDueToStartupCooldown) {
    EXPECT_FALSE(controller_->may_fire());
}

TEST_F(FiringControllerTest, MayFireAfterStartupCooldown) {
    std::this_thread::sleep_for(1100ms);
    EXPECT_TRUE(controller_->may_fire());
}

TEST_F(FiringControllerTest, SetTargetDoesNotFireImmediately) {
    EXPECT_CALL(*mock_dac_, write(_))
        .WillRepeatedly(Return(std::expected<void, HardwareError>{}));

    controller_->set_target({0.0, 0.0, 1.0});

    auto now = std::chrono::steady_clock::now();
    controller_->execute_cycle(now);
}

TEST_F(FiringControllerTest, DacWriteBeforeFire) {
    EXPECT_CALL(*mock_dac_, write(_))
        .Times(AtLeast(1))
        .WillRepeatedly(Return(std::expected<void, HardwareError>{}));
    EXPECT_CALL(*mock_laser_, fire(true))
        .WillOnce(Return(std::expected<void, HardwareError>{}));
    EXPECT_CALL(*mock_laser_, fire(false))
        .WillRepeatedly(Return(std::expected<void, HardwareError>{}));

    controller_->set_target({0.0, 0.0, 1.0});

    std::this_thread::sleep_for(1100ms);

    auto now = std::chrono::steady_clock::now();
    (void)controller_->execute_cycle(now);
    (void)controller_->execute_cycle(now + 4ms);
}

TEST_F(FiringControllerTest, EmergencyStopForcesLaserOff) {
    EXPECT_CALL(*mock_laser_, fire(false))
        .WillRepeatedly(Return(std::expected<void, HardwareError>{}));

    controller_->emergency_stop();

    auto now = std::chrono::steady_clock::now();
    auto result = controller_->execute_cycle(now);

    EXPECT_FALSE(result);

    controller_->set_target({0.0, 0.0, 1.0});
    auto cycle_result = controller_->execute_cycle(now + 1ms);
    EXPECT_FALSE(cycle_result);
}

TEST_F(FiringControllerTest, ClearTargetPreventsFiring) {
    EXPECT_CALL(*mock_dac_, write(_))
        .WillRepeatedly(Return(std::expected<void, HardwareError>{}));

    controller_->set_target({0.0, 0.0, 1.0});
    controller_->clear_target();

    auto now = std::chrono::steady_clock::now();
    auto result = controller_->execute_cycle(now);
    EXPECT_FALSE(result);
}

TEST_F(FiringControllerTest, LaserForceDisabledWhenNotMayFire) {
    EXPECT_CALL(*mock_laser_, fire(false))
        .WillRepeatedly(Return(std::expected<void, HardwareError>{}));

    auto now = std::chrono::steady_clock::now();
    controller_->execute_cycle(now);
}
