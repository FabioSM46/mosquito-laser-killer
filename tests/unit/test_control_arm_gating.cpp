#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <chrono>
#include <memory>

#include "mocks/mock_galvo_driver.h"
#include "mocks/mock_laser.h"
#include "core/types.h"
#include "core/error.h"
#include "safety/bounding_box.h"
#include "safety/system_state.h"
#include "control/coordinate_mapper.h"
#include "control/firing_controller.h"

using namespace testing;
using namespace std::chrono_literals;

// Mirrors the control-thread arm/target gating in main.cpp without hardware.
class ControlArmGatingTest : public Test {
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
            *mock_laser_, *mock_galvo_, *mapper_, 100.0, 0.0, 3.0);

        ASSERT_TRUE(sm_.transition(SystemState::IDLE));
    }

    auto apply_arm_and_target(bool is_armed, bool target_valid) -> void {
        controller_->set_armed(is_armed);
        auto cs = sm_.current();

        if (!is_armed && cs != SystemState::IDLE &&
            cs != SystemState::SAFE_HALT && cs != SystemState::COOLDOWN) {
            if (cs == SystemState::FIRING) {
                (void)sm_.transition(SystemState::COOLDOWN);
            } else {
                (void)sm_.transition(SystemState::IDLE);
            }
        }

        if (is_armed && cs == SystemState::IDLE) {
            (void)sm_.transition(SystemState::ARMED);
        }

        auto current_state = sm_.current();
        if (!is_armed) {
            controller_->clear_target();
        } else if (current_state == SystemState::FIRING) {
            if (!target_valid) {
                (void)sm_.transition(SystemState::COOLDOWN);
                controller_->clear_target();
            }
        } else if (target_valid &&
                   (current_state == SystemState::ARMED ||
                    current_state == SystemState::TRACKING)) {
            if (current_state == SystemState::ARMED) {
                (void)sm_.transition(SystemState::TRACKING);
            }
            controller_->set_target({0.0, 0.0, 1.0});
        } else {
            if (current_state == SystemState::TRACKING) {
                (void)sm_.transition(SystemState::ARMED);
            }
            controller_->clear_target();
        }
    }

    std::unique_ptr<MockLaser> mock_laser_;
    std::unique_ptr<MockGalvoDriver> mock_galvo_;
    std::unique_ptr<BoundingBox3D> bbox_;
    std::unique_ptr<CoordinateMapper> mapper_;
    std::unique_ptr<FiringController> controller_;
    SystemStateMachine sm_;
};

TEST_F(ControlArmGatingTest, TrackingDisarmGoesIdleAndCannotFire) {
    EXPECT_CALL(*mock_galvo_, set_position(_, _))
        .WillRepeatedly(Return(std::expected<void, HardwareError>{}));
    EXPECT_CALL(*mock_laser_, fire(true)).Times(0);

    apply_arm_and_target(true, true);
    EXPECT_EQ(sm_.current(), SystemState::TRACKING);
    EXPECT_TRUE(controller_->is_armed());

    apply_arm_and_target(false, true);
    EXPECT_EQ(sm_.current(), SystemState::IDLE);
    EXPECT_FALSE(controller_->is_armed());

    auto now = std::chrono::steady_clock::now();
    (void)controller_->execute_cycle(now);
    (void)controller_->execute_cycle(now + 4ms);
    EXPECT_FALSE(controller_->is_firing());
}

TEST_F(ControlArmGatingTest, DisarmThenRearmWorks) {
    EXPECT_CALL(*mock_galvo_, set_position(_, _))
        .WillRepeatedly(Return(std::expected<void, HardwareError>{}));
    EXPECT_CALL(*mock_laser_, fire(true))
        .WillOnce(Return(std::expected<void, HardwareError>{}));

    apply_arm_and_target(true, true);
    EXPECT_EQ(sm_.current(), SystemState::TRACKING);

    apply_arm_and_target(false, true);
    EXPECT_EQ(sm_.current(), SystemState::IDLE);

    apply_arm_and_target(true, true);
    EXPECT_EQ(sm_.current(), SystemState::TRACKING);
    EXPECT_TRUE(controller_->is_armed());

    auto now = std::chrono::steady_clock::now();
    (void)controller_->execute_cycle(now);
    (void)controller_->execute_cycle(now + 4ms);
    EXPECT_TRUE(controller_->is_firing());
}

TEST_F(ControlArmGatingTest, TargetWhileDisarmedNeverArmsController) {
    EXPECT_CALL(*mock_laser_, fire(true)).Times(0);

    apply_arm_and_target(false, true);
    EXPECT_EQ(sm_.current(), SystemState::IDLE);
    EXPECT_FALSE(controller_->is_armed());

    auto now = std::chrono::steady_clock::now();
    (void)controller_->execute_cycle(now);
    (void)controller_->execute_cycle(now + 4ms);
    EXPECT_FALSE(controller_->is_firing());
}
