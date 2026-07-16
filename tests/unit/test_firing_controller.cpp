#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <chrono>
#include <memory>
#include <thread>

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
        // cooldown_s=0 skips the 1s production startup blanking for unit tests.
        controller_ = std::make_unique<FiringController>(
            *mock_laser_, *mock_galvo_, *mapper_,
            100.0, 0.0, 3.0);
    }

    auto make_controller(double max_pulse_ms, double cooldown_s, double settle_ms)
        -> std::unique_ptr<FiringController> {
        return std::make_unique<FiringController>(
            *mock_laser_, *mock_galvo_, *mapper_,
            max_pulse_ms, cooldown_s, settle_ms);
    }

    std::unique_ptr<MockLaser> mock_laser_;
    std::unique_ptr<MockGalvoDriver> mock_galvo_;
    std::unique_ptr<BoundingBox3D> bbox_;
    std::unique_ptr<CoordinateMapper> mapper_;
    std::unique_ptr<FiringController> controller_;
};

TEST_F(FiringControllerTest, MayFireTrueWithZeroCooldown) {
    EXPECT_TRUE(controller_->may_fire());
}

TEST_F(FiringControllerTest, MayFireInitiallyFalseWithProductionCooldown) {
    auto fc = make_controller(100.0, 10.0, 3.0);
    EXPECT_FALSE(fc->may_fire());
}

TEST_F(FiringControllerTest, SetTargetRejectedWhenDisarmed) {
    EXPECT_CALL(*mock_galvo_, set_position(_, _)).Times(0);
    EXPECT_CALL(*mock_laser_, fire(true)).Times(0);

    controller_->set_target({0.0, 0.0, 1.0});

    auto now = std::chrono::steady_clock::now();
    (void)controller_->execute_cycle(now);
    (void)controller_->execute_cycle(now + 4ms);
    EXPECT_FALSE(controller_->is_firing());
}

TEST_F(FiringControllerTest, SetTargetDoesNotFireImmediately) {
    EXPECT_CALL(*mock_galvo_, set_position(_, _))
        .WillRepeatedly(Return(std::expected<void, HardwareError>{}));
    EXPECT_CALL(*mock_laser_, fire(true)).Times(0);

    controller_->set_armed(true);
    controller_->set_target({0.0, 0.0, 1.0});

    auto now = std::chrono::steady_clock::now();
    controller_->execute_cycle(now);
}

TEST_F(FiringControllerTest, DacWriteBeforeFire) {
    controller_->set_armed(true);
    controller_->set_target({0.0, 0.0, 1.0});

    {
        InSequence seq;
        // Galvo commanded each pre-fire cycle until settle completes, then laser.
        EXPECT_CALL(*mock_galvo_, set_position(_, _))
            .Times(2)
            .WillRepeatedly(Return(std::expected<void, HardwareError>{}));
        EXPECT_CALL(*mock_laser_, fire(true))
            .WillOnce(Return(std::expected<void, HardwareError>{}));
    }

    auto now = std::chrono::steady_clock::now();
    (void)controller_->execute_cycle(now);
    (void)controller_->execute_cycle(now + 4ms);
    EXPECT_TRUE(controller_->is_firing());
    // Motion blanking: no further galvo writes while pulse active.
    EXPECT_CALL(*mock_galvo_, set_position(_, _)).Times(0);
    (void)controller_->execute_cycle(now + 8ms);
}

TEST_F(FiringControllerTest, NoGalvoWriteWhileLaserFiring) {
    controller_->set_armed(true);
    controller_->set_target({0.0, 0.0, 1.0});

    EXPECT_CALL(*mock_galvo_, set_position(_, _))
        .Times(2)
        .WillRepeatedly(Return(std::expected<void, HardwareError>{}));
    EXPECT_CALL(*mock_laser_, fire(true))
        .WillOnce(Return(std::expected<void, HardwareError>{}));

    auto now = std::chrono::steady_clock::now();
    (void)controller_->execute_cycle(now);
    (void)controller_->execute_cycle(now + 4ms);
    ASSERT_TRUE(controller_->is_firing());

    Mock::VerifyAndClearExpectations(mock_galvo_.get());
    EXPECT_CALL(*mock_galvo_, set_position(_, _)).Times(0);
    (void)controller_->execute_cycle(now + 8ms);
    (void)controller_->execute_cycle(now + 12ms);
    (void)controller_->execute_cycle(now + 16ms);
    EXPECT_TRUE(controller_->is_firing());
}

TEST_F(FiringControllerTest, EmergencyStopForcesLaserOff) {
    EXPECT_CALL(*mock_laser_, fire(false))
        .WillRepeatedly(Return(std::expected<void, HardwareError>{}));
    EXPECT_CALL(*mock_laser_, emergency_shutdown())
        .WillOnce(Return(std::expected<void, HardwareError>{}));

    controller_->emergency_stop();

    auto now = std::chrono::steady_clock::now();
    auto result = controller_->execute_cycle(now);

    EXPECT_FALSE(result);

    controller_->set_armed(true);
    controller_->set_target({0.0, 0.0, 1.0});
    auto cycle_result = controller_->execute_cycle(now + 1ms);
    EXPECT_FALSE(cycle_result);
}

TEST_F(FiringControllerTest, ClearTargetPreventsFiring) {
    EXPECT_CALL(*mock_galvo_, set_position(_, _))
        .WillRepeatedly(Return(std::expected<void, HardwareError>{}));

    controller_->set_armed(true);
    controller_->set_target({0.0, 0.0, 1.0});
    controller_->clear_target();

    auto now = std::chrono::steady_clock::now();
    auto result = controller_->execute_cycle(now);
    EXPECT_FALSE(result);
}

TEST_F(FiringControllerTest, LaserForceDisabledWhenNotMayFire) {
    auto fc = make_controller(100.0, 10.0, 3.0);
    EXPECT_CALL(*mock_laser_, fire(false))
        .WillRepeatedly(Return(std::expected<void, HardwareError>{}));

    auto now = std::chrono::steady_clock::now();
    fc->execute_cycle(now);
}

TEST_F(FiringControllerTest, PulseEndsAfterMaxDuration) {
    auto fc = make_controller(20.0, 0.0, 3.0);

    EXPECT_CALL(*mock_galvo_, set_position(_, _))
        .WillRepeatedly(Return(std::expected<void, HardwareError>{}));
    EXPECT_CALL(*mock_laser_, fire(_))
        .WillRepeatedly(Return(std::expected<void, HardwareError>{}));

    fc->set_armed(true);

    auto t0 = std::chrono::steady_clock::now();
    fc->set_target({0.0, 0.0, 1.0});

    (void)fc->execute_cycle(t0);
    (void)fc->execute_cycle(t0 + 4ms);
    ASSERT_TRUE(fc->is_firing());

    auto result = fc->execute_cycle(t0 + 30ms);
    EXPECT_TRUE(result);
    EXPECT_FALSE(fc->is_firing());
}

TEST_F(FiringControllerTest, CooldownPreventsReFireAfterPulse) {
    auto fc = make_controller(100.0, 10.0, 3.0);

    EXPECT_CALL(*mock_galvo_, set_position(_, _))
        .WillRepeatedly(Return(std::expected<void, HardwareError>{}));
    EXPECT_CALL(*mock_laser_, fire(_))
        .WillRepeatedly(Return(std::expected<void, HardwareError>{}));

    // Production cooldown has 1s startup blanking.
    std::this_thread::sleep_for(1050ms);
    fc->set_armed(true);

    auto t0 = std::chrono::steady_clock::now();
    fc->set_target({0.0, 0.0, 1.0});

    (void)fc->execute_cycle(t0);
    (void)fc->execute_cycle(t0 + 4ms);
    ASSERT_TRUE(fc->is_firing());

    auto pulse_end = fc->execute_cycle(t0 + 110ms);
    EXPECT_TRUE(pulse_end);
    EXPECT_FALSE(fc->may_fire());

    fc->set_target({0.5, 0.0, 1.0});
    auto fire_result = fc->execute_cycle(t0 + 115ms);
    EXPECT_FALSE(fire_result);
    EXPECT_FALSE(fc->is_firing());
}

TEST_F(FiringControllerTest, DisarmTurnsOffLaserImmediately) {
    EXPECT_CALL(*mock_galvo_, set_position(_, _))
        .WillRepeatedly(Return(std::expected<void, HardwareError>{}));
    EXPECT_CALL(*mock_laser_, fire(true))
        .WillOnce(Return(std::expected<void, HardwareError>{}));
    EXPECT_CALL(*mock_laser_, fire(false))
        .WillRepeatedly(Return(std::expected<void, HardwareError>{}));

    controller_->set_armed(true);

    auto t0 = std::chrono::steady_clock::now();
    controller_->set_target({0.0, 0.0, 1.0});

    (void)controller_->execute_cycle(t0);
    (void)controller_->execute_cycle(t0 + 4ms);
    ASSERT_TRUE(controller_->is_firing());

    controller_->disarm();
    EXPECT_FALSE(controller_->is_firing());
    EXPECT_FALSE(controller_->is_armed());

    auto result = controller_->execute_cycle(t0 + 12ms);
    EXPECT_FALSE(result);
}

TEST_F(FiringControllerTest, ClearTargetAbortsPulseAndCooldown) {
    auto fc = make_controller(100.0, 10.0, 3.0);
    EXPECT_CALL(*mock_galvo_, set_position(_, _))
        .WillRepeatedly(Return(std::expected<void, HardwareError>{}));
    EXPECT_CALL(*mock_laser_, fire(_))
        .WillRepeatedly(Return(std::expected<void, HardwareError>{}));

    std::this_thread::sleep_for(1050ms);
    fc->set_armed(true);

    auto t0 = std::chrono::steady_clock::now();
    fc->set_target({0.0, 0.0, 1.0});

    (void)fc->execute_cycle(t0);
    (void)fc->execute_cycle(t0 + 4ms);
    ASSERT_TRUE(fc->is_firing());

    fc->clear_target();

    EXPECT_FALSE(fc->is_firing());
    EXPECT_FALSE(fc->may_fire());
}

TEST_F(FiringControllerTest, SetTargetDuringPulseAbortsPulseAndCooldown) {
    auto fc = make_controller(100.0, 10.0, 3.0);
    EXPECT_CALL(*mock_galvo_, set_position(_, _))
        .WillRepeatedly(Return(std::expected<void, HardwareError>{}));
    EXPECT_CALL(*mock_laser_, fire(_))
        .WillRepeatedly(Return(std::expected<void, HardwareError>{}));

    std::this_thread::sleep_for(1050ms);
    fc->set_armed(true);

    auto t0 = std::chrono::steady_clock::now();
    fc->set_target({0.0, 0.0, 1.0});

    (void)fc->execute_cycle(t0);
    (void)fc->execute_cycle(t0 + 4ms);

    EXPECT_TRUE(fc->is_firing());

    fc->set_target({0.5, 0.0, 1.0});

    EXPECT_FALSE(fc->is_firing());
    EXPECT_FALSE(fc->may_fire());
}

TEST_F(FiringControllerTest, IsFiringFalseInitially) {
    EXPECT_FALSE(controller_->is_firing());
}

TEST_F(FiringControllerTest, SetArmedFalseDisarmsAndBlocksFire) {
    EXPECT_CALL(*mock_galvo_, set_position(_, _))
        .WillRepeatedly(Return(std::expected<void, HardwareError>{}));
    EXPECT_CALL(*mock_laser_, fire(true))
        .WillOnce(Return(std::expected<void, HardwareError>{}));
    EXPECT_CALL(*mock_laser_, fire(false))
        .WillRepeatedly(Return(std::expected<void, HardwareError>{}));

    controller_->set_armed(true);
    controller_->set_target({0.0, 0.0, 1.0});

    auto t0 = std::chrono::steady_clock::now();
    (void)controller_->execute_cycle(t0);
    (void)controller_->execute_cycle(t0 + 4ms);
    ASSERT_TRUE(controller_->is_firing());

    controller_->set_armed(false);
    EXPECT_FALSE(controller_->is_firing());
    EXPECT_FALSE(controller_->is_armed());

    controller_->set_target({0.0, 0.0, 1.0});
    (void)controller_->execute_cycle(t0 + 8ms);
    (void)controller_->execute_cycle(t0 + 12ms);
    EXPECT_FALSE(controller_->is_firing());
}

TEST_F(FiringControllerTest, LaserOffFailureTriggersEmergencyHalt) {
    auto fc = make_controller(20.0, 0.0, 3.0);

    EXPECT_CALL(*mock_galvo_, set_position(_, _))
        .WillRepeatedly(Return(std::expected<void, HardwareError>{}));
    EXPECT_CALL(*mock_laser_, fire(true))
        .WillOnce(Return(std::expected<void, HardwareError>{}));
    EXPECT_CALL(*mock_laser_, fire(false))
        .WillOnce(Return(std::unexpected(HardwareError::GpioWriteFailed)));
    EXPECT_CALL(*mock_laser_, emergency_shutdown())
        .WillOnce(Return(std::expected<void, HardwareError>{}));

    fc->set_armed(true);

    auto t0 = std::chrono::steady_clock::now();
    fc->set_target({0.0, 0.0, 1.0});
    (void)fc->execute_cycle(t0);
    (void)fc->execute_cycle(t0 + 4ms);
    ASSERT_TRUE(fc->is_firing());

    auto result = fc->execute_cycle(t0 + 30ms);
    EXPECT_FALSE(result);
    EXPECT_FALSE(fc->is_firing());
    EXPECT_FALSE(fc->may_fire());
}
