#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <chrono>
#include <thread>
#include <memory>

#include "mocks/mock_gpio.h"
#include "mocks/mock_spi.h"
#include "mocks/mock_dac.h"
#include "mocks/mock_galvo_driver.h"
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
        mock_galvo_ = std::make_unique<NiceMock<MockGalvoDriver>>();

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
            *mock_laser_, *mock_galvo_, *mapper_,
            100.0, 10.0, 3.0);
    }

    std::unique_ptr<MockLaser> mock_laser_;
    std::unique_ptr<MockGalvoDriver> mock_galvo_;
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
    EXPECT_CALL(*mock_galvo_, set_position(_, _))
        .WillRepeatedly(Return(std::expected<void, HardwareError>{}));

    controller_->set_target({0.0, 0.0, 1.0});

    auto now = std::chrono::steady_clock::now();
    controller_->execute_cycle(now);
}

TEST_F(FiringControllerTest, DacWriteBeforeFire) {
    EXPECT_CALL(*mock_galvo_, set_position(_, _))
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
    EXPECT_CALL(*mock_galvo_, set_position(_, _))
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

TEST_F(FiringControllerTest, PulseEndsAfterMaxDuration) {
    FiringController fc(*mock_laser_, *mock_galvo_, *mapper_, 20.0, 100.0, 3.0);

    EXPECT_CALL(*mock_galvo_, set_position(_, _))
        .WillRepeatedly(Return(std::expected<void, HardwareError>{}));
    EXPECT_CALL(*mock_laser_, fire(_))
        .WillRepeatedly(Return(std::expected<void, HardwareError>{}));

    std::this_thread::sleep_for(1100ms);

    auto t0 = std::chrono::steady_clock::now();
    fc.set_target({0.0, 0.0, 1.0});

    (void)fc.execute_cycle(t0 + std::chrono::milliseconds(4));
    (void)fc.execute_cycle(t0 + std::chrono::milliseconds(8));

    auto result = fc.execute_cycle(t0 + std::chrono::milliseconds(30));
    EXPECT_TRUE(result);
    EXPECT_FALSE(fc.may_fire());
}

TEST_F(FiringControllerTest, CooldownPreventsReFireAfterPulse) {
    EXPECT_CALL(*mock_galvo_, set_position(_, _))
        .WillRepeatedly(Return(std::expected<void, HardwareError>{}));
    EXPECT_CALL(*mock_laser_, fire(_))
        .WillRepeatedly(Return(std::expected<void, HardwareError>{}));

    std::this_thread::sleep_for(1100ms);

    auto t0 = std::chrono::steady_clock::now();
    controller_->set_target({0.0, 0.0, 1.0});

    (void)controller_->execute_cycle(t0 + std::chrono::milliseconds(4));
    (void)controller_->execute_cycle(t0 + std::chrono::milliseconds(8));

    auto pulse_end = controller_->execute_cycle(t0 + std::chrono::milliseconds(110));
    EXPECT_TRUE(pulse_end);

    EXPECT_FALSE(controller_->may_fire());

    controller_->set_target({0.5, 0.0, 1.0});
    auto fire_result = controller_->execute_cycle(t0 + std::chrono::milliseconds(115));
    EXPECT_FALSE(fire_result);
}

TEST_F(FiringControllerTest, DisarmTurnsOffLaserImmediately) {
    EXPECT_CALL(*mock_galvo_, set_position(_, _))
        .WillRepeatedly(Return(std::expected<void, HardwareError>{}));
    EXPECT_CALL(*mock_laser_, fire(true))
        .WillOnce(Return(std::expected<void, HardwareError>{}));
    EXPECT_CALL(*mock_laser_, fire(false))
        .WillRepeatedly(Return(std::expected<void, HardwareError>{}));

    std::this_thread::sleep_for(1100ms);

    auto t0 = std::chrono::steady_clock::now();
    controller_->set_target({0.0, 0.0, 1.0});

    (void)controller_->execute_cycle(t0 + std::chrono::milliseconds(4));
    (void)controller_->execute_cycle(t0 + std::chrono::milliseconds(8));

    controller_->disarm();

    auto result = controller_->execute_cycle(t0 + std::chrono::milliseconds(12));
    EXPECT_FALSE(result);
}

TEST_F(FiringControllerTest, ClearTargetAbortsPulseAndCooldown) {
    EXPECT_CALL(*mock_galvo_, set_position(_, _))
        .WillRepeatedly(Return(std::expected<void, HardwareError>{}));
    EXPECT_CALL(*mock_laser_, fire(_))
        .WillRepeatedly(Return(std::expected<void, HardwareError>{}));

    std::this_thread::sleep_for(1100ms);

    auto t0 = std::chrono::steady_clock::now();
    controller_->set_target({0.0, 0.0, 1.0});

    (void)controller_->execute_cycle(t0 + std::chrono::milliseconds(4));
    (void)controller_->execute_cycle(t0 + std::chrono::milliseconds(8));

    controller_->clear_target();

    EXPECT_FALSE(controller_->may_fire());
}
