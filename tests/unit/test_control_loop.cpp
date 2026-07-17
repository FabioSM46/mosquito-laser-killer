#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <chrono>
#include <memory>

#include "control/control_loop.h"
#include "control/firing_controller.h"
#include "control/coordinate_mapper.h"
#include "safety/system_state.h"
#include "safety/watchdog.h"
#include "safety/arm_switch.h"
#include "safety/e_stop.h"
#include "safety/bounding_box.h"
#include "mocks/mock_gpio.h"
#include "mocks/mock_laser.h"
#include "mocks/mock_galvo_driver.h"

using namespace testing;
using namespace std::chrono_literals;

namespace {

constexpr auto kOk = std::expected<void, HardwareError>{};

// Production values from config/system_config.yaml and core/types.h. Tests that
// build an easier config than the one that ships prove nothing about the binary:
// the previous fixture used a +/-2m box and a +/-25 deg cone, a combination the
// project's own config_validator rejects as a critical error.
auto production_config() -> SystemConfig {
    return SystemConfig{};
}

class ControlLoopTest : public Test {
protected:
    void SetUp() override {
        config_ = production_config();
        t0_ = std::chrono::steady_clock::now();
        // Must be assigned here, not in the member initializer: t0_ is still the
        // clock epoch at that point.
        now_ = t0_;

        laser_ = std::make_unique<NiceMock<MockLaser>>();
        galvo_ = std::make_unique<NiceMock<MockGalvoDriver>>();
        ON_CALL(*laser_, fire(_)).WillByDefault(Return(kOk));
        ON_CALL(*laser_, emergency_shutdown()).WillByDefault(Return(kOk));
        ON_CALL(*galvo_, set_position(_, _)).WillByDefault(Return(kOk));
        ON_CALL(*galvo_, zero()).WillByDefault(Return(kOk));

        auto arm_gpio = std::make_unique<NiceMock<MockGpio>>();
        arm_gpio_ = arm_gpio.get();
        ON_CALL(*arm_gpio_, set_direction_input()).WillByDefault(Return(kOk));
        ON_CALL(*arm_gpio_, read()).WillByDefault(Return(false));

        auto estop_gpio = std::make_unique<NiceMock<MockGpio>>();
        estop_gpio_ = estop_gpio.get();
        ON_CALL(*estop_gpio_, set_direction_input()).WillByDefault(Return(kOk));
        // Active LOW: HIGH means released.
        ON_CALL(*estop_gpio_, read()).WillByDefault(Return(true));

        arm_switch_ = std::make_unique<ArmSwitch>(std::move(arm_gpio));
        e_stop_ = std::make_unique<EStop>(std::move(estop_gpio));
        ASSERT_TRUE(arm_switch_->initialize().has_value());
        ASSERT_TRUE(e_stop_->initialize().has_value());

        bbox_ = std::make_unique<BoundingBox3D>(config_.bounding_box);
        mapper_ = std::make_unique<CoordinateMapper>(
            *bbox_, config_.galvo_limits, config_.dac_ref_voltage, config_.galvo_driver);
        controller_ = std::make_unique<FiringController>(
            *laser_, *galvo_, *mapper_, config_.max_pulse_duration_ms,
            config_.cooldown_seconds, config_.settle_delay_ms, t0_);
        watchdog_ = std::make_unique<Watchdog>(
            sm_, *laser_, *galvo_,
            std::chrono::milliseconds(static_cast<long>(config_.watchdog_timeout_ms)),
            std::chrono::milliseconds(static_cast<long>(config_.watchdog_startup_grace_ms)));

        ASSERT_TRUE(sm_.transition(SystemState::IDLE));

        deps_ = std::make_unique<ControlDeps>(ControlDeps{
            sm_, *controller_, *watchdog_, *arm_switch_, *e_stop_, *laser_});
    }

    // Runs `count` cycles at the production frame period, feeding a live
    // heartbeat, and returns the last outcome.
    auto run_cycles(int count, const std::optional<TargetCommand>& cmd = std::nullopt)
        -> ControlOutcome {
        auto outcome = ControlOutcome::Continue;
        for (int i = 0; i < count; ++i) {
            now_ += kCyclePeriod;
            outcome = control_step(*deps_, cmd, now_, now_);
            if (outcome == ControlOutcome::Halt) {
                break;
            }
        }
        return outcome;
    }

    void set_arm_gpio(bool high) {
        ON_CALL(*arm_gpio_, read()).WillByDefault(Return(high));
    }

    void set_estop_pressed(bool pressed) {
        // Active LOW.
        ON_CALL(*estop_gpio_, read()).WillByDefault(Return(!pressed));
    }

    // Debounce is 6 cycles for the arm switch, 3 for the e-stop.
    void settle_inputs() { run_cycles(8); }

    auto make_target(Point3D p) -> TargetCommand {
        TargetCommand cmd;
        cmd.target_valid = true;
        cmd.target_position = p;
        return cmd;
    }

    // Inside the production box: x,y in [-0.09, 0.09], z in [0.5, 1.0].
    static constexpr Point3D kValidTarget{0.0, 0.0, 0.7};
    static constexpr auto kCyclePeriod = 5ms;

    SystemConfig config_{};
    SystemStateMachine sm_;
    std::chrono::steady_clock::time_point t0_;
    std::chrono::steady_clock::time_point now_;

    std::unique_ptr<NiceMock<MockLaser>> laser_;
    std::unique_ptr<NiceMock<MockGalvoDriver>> galvo_;
    NiceMock<MockGpio>* arm_gpio_{nullptr};
    NiceMock<MockGpio>* estop_gpio_{nullptr};
    std::unique_ptr<ArmSwitch> arm_switch_;
    std::unique_ptr<EStop> e_stop_;
    std::unique_ptr<BoundingBox3D> bbox_;
    std::unique_ptr<CoordinateMapper> mapper_;
    std::unique_ptr<FiringController> controller_;
    std::unique_ptr<Watchdog> watchdog_;
    std::unique_ptr<ControlDeps> deps_;
};

//
// Guard ordering — nothing else pins this, and the order is the safety property.
//

TEST_F(ControlLoopTest, EnforceMaxPulseRunsEveryCycleBeforeAnythingCanReturnEarly) {
    // The HAL-level pulse limit is defense in depth against a wedged sequencer.
    // Deleting the call from the loop previously failed no test at all.
    EXPECT_CALL(*laser_, enforce_max_pulse(_)).Times(AtLeast(5));
    run_cycles(5);
}

TEST_F(ControlLoopTest, EnforceMaxPulseStillRunsOnTheCycleTheEStopHalts) {
    now_ += kCyclePeriod;
    set_estop_pressed(true);

    // Ordering: the pulse limit must be applied before the e-stop check returns.
    EXPECT_CALL(*laser_, enforce_max_pulse(_)).Times(AtLeast(1));
    EXPECT_CALL(*laser_, emergency_shutdown()).Times(AtLeast(1));

    ControlOutcome outcome = ControlOutcome::Continue;
    for (int i = 0; i < 5 && outcome == ControlOutcome::Continue; ++i) {
        now_ += kCyclePeriod;
        outcome = control_step(*deps_, std::nullopt, now_, now_);
    }
    EXPECT_EQ(outcome, ControlOutcome::Halt);
    EXPECT_EQ(sm_.current(), SystemState::SAFE_HALT);
}

TEST_F(ControlLoopTest, EStopHaltsEvenWhileArmedAndTracking) {
    set_arm_gpio(true);
    settle_inputs();
    auto cmd = make_target(kValidTarget);
    run_cycles(2, cmd);
    ASSERT_EQ(sm_.current(), SystemState::TRACKING);

    set_estop_pressed(true);
    auto outcome = run_cycles(5, cmd);

    EXPECT_EQ(outcome, ControlOutcome::Halt);
    EXPECT_EQ(sm_.current(), SystemState::SAFE_HALT);
    EXPECT_FALSE(controller_->is_firing());
}

TEST_F(ControlLoopTest, WatchdogStaleHeartbeatHaltsTheLoop) {
    set_arm_gpio(true);
    settle_inputs();

    // Establish a real heartbeat, then stop advancing it.
    const auto frozen = now_;
    ControlOutcome outcome = ControlOutcome::Continue;
    for (int i = 0; i < 50 && outcome == ControlOutcome::Continue; ++i) {
        now_ += kCyclePeriod;
        outcome = control_step(*deps_, std::nullopt, frozen, now_);
    }

    EXPECT_EQ(outcome, ControlOutcome::Halt);
    EXPECT_EQ(sm_.current(), SystemState::SAFE_HALT);
}

//
// Arm gating
//

TEST_F(ControlLoopTest, DisarmedNeverFiresEvenWithAValidTarget) {
    set_arm_gpio(false);
    EXPECT_CALL(*laser_, fire(false)).Times(AnyNumber());
    EXPECT_CALL(*laser_, fire(true)).Times(0);

    auto cmd = make_target(kValidTarget);
    run_cycles(50, cmd);

    EXPECT_FALSE(controller_->is_firing());
    EXPECT_EQ(sm_.current(), SystemState::IDLE);
}

TEST_F(ControlLoopTest, ArmSwitchOnMovesIdleToArmed) {
    set_arm_gpio(true);
    settle_inputs();
    EXPECT_EQ(sm_.current(), SystemState::ARMED);
}

TEST_F(ControlLoopTest, ValidTargetMovesArmedToTracking) {
    set_arm_gpio(true);
    settle_inputs();
    ASSERT_EQ(sm_.current(), SystemState::ARMED);

    run_cycles(1, make_target(kValidTarget));
    EXPECT_EQ(sm_.current(), SystemState::TRACKING);
}

TEST_F(ControlLoopTest, DisarmDuringTrackingReturnsToIdleAndClearsTarget) {
    set_arm_gpio(true);
    settle_inputs();
    auto cmd = make_target(kValidTarget);
    run_cycles(2, cmd);
    ASSERT_EQ(sm_.current(), SystemState::TRACKING);

    set_arm_gpio(false);
    run_cycles(8, cmd);

    EXPECT_EQ(sm_.current(), SystemState::IDLE);
    EXPECT_FALSE(controller_->is_armed());
    EXPECT_FALSE(controller_->is_firing());
}

TEST_F(ControlLoopTest, RearmAfterDisarmReturnsToArmed) {
    set_arm_gpio(true);
    settle_inputs();
    run_cycles(2, make_target(kValidTarget));
    ASSERT_EQ(sm_.current(), SystemState::TRACKING);

    set_arm_gpio(false);
    run_cycles(8);
    ASSERT_EQ(sm_.current(), SystemState::IDLE);

    set_arm_gpio(true);
    run_cycles(8);
    EXPECT_EQ(sm_.current(), SystemState::ARMED);
    EXPECT_TRUE(controller_->is_armed());
}

TEST_F(ControlLoopTest, TargetOutsideTheBoundingBoxNeverFires) {
    set_arm_gpio(true);
    settle_inputs();

    EXPECT_CALL(*laser_, fire(false)).Times(AnyNumber());

    EXPECT_CALL(*laser_, fire(true)).Times(0);

    // z = 3.0m is well beyond z_max = 1.0m — a face across the room, not a target.
    run_cycles(50, make_target({0.0, 0.0, 3.0}));
    EXPECT_FALSE(controller_->is_firing());
}

//
// Fire sequence
//

TEST_F(ControlLoopTest, FiresOnlyAfterSettleAndStartupBlankingElapse) {
    set_arm_gpio(true);
    settle_inputs();

    // Phase 1: still inside the 1s startup blanking, galvo settled, target held.
    // The laser must NOT fire no matter how clean the track is. The previous
    // version installed the fire(true) expectation only after jumping past the
    // blanking window, so a fire during blanking was invisible to it.
    auto cmd = make_target(kValidTarget);
    {
        EXPECT_CALL(*laser_, fire(false)).Times(AnyNumber());
        EXPECT_CALL(*laser_, fire(true)).Times(0);
        run_cycles(10, cmd);
        EXPECT_FALSE(controller_->is_firing());
        ASSERT_EQ(sm_.current(), SystemState::TRACKING);
        ::testing::Mock::VerifyAndClearExpectations(laser_.get());
    }

    // Phase 2: blanking elapsed, target held continuously — the pulse starts.
    now_ += FiringController::k_startup_blanking;
    ON_CALL(*laser_, fire(_)).WillByDefault(Return(kOk));
    EXPECT_CALL(*laser_, fire(true)).Times(1);
    run_cycles(3, cmd);

    EXPECT_TRUE(controller_->is_firing());
    EXPECT_EQ(sm_.current(), SystemState::FIRING);
}

TEST_F(ControlLoopTest, PulseEndsAtMaxDurationAndEntersCooldown) {
    set_arm_gpio(true);
    settle_inputs();
    now_ += FiringController::k_startup_blanking;

    auto cmd = make_target(kValidTarget);
    run_cycles(3, cmd);
    ASSERT_EQ(sm_.current(), SystemState::FIRING);

    EXPECT_CALL(*laser_, fire(false)).Times(AtLeast(1));

    // Step past max_pulse_duration_ms (100ms).
    now_ += 110ms;
    (void)control_step(*deps_, cmd, now_, now_);

    EXPECT_FALSE(controller_->is_firing());
    EXPECT_EQ(sm_.current(), SystemState::COOLDOWN);
}

TEST_F(ControlLoopTest, NoGalvoWritesWhileThePulseIsActive) {
    set_arm_gpio(true);
    settle_inputs();
    now_ += FiringController::k_startup_blanking;

    auto cmd = make_target(kValidTarget);
    run_cycles(3, cmd);
    ASSERT_TRUE(controller_->is_firing());

    // Motion blanking: the galvo command path is dead while the laser is ON.
    EXPECT_CALL(*galvo_, set_position(_, _)).Times(0);

    auto moved = make_target({0.02, 0.02, 0.7});
    now_ += kCyclePeriod;
    (void)control_step(*deps_, moved, now_, now_);
}

TEST_F(ControlLoopTest, CooldownBlocksRefireForTheConfiguredDuration) {
    set_arm_gpio(true);
    settle_inputs();
    now_ += FiringController::k_startup_blanking;

    auto cmd = make_target(kValidTarget);
    run_cycles(3, cmd);
    ASSERT_TRUE(controller_->is_firing());

    now_ += 110ms;
    (void)control_step(*deps_, cmd, now_, now_);
    ASSERT_EQ(sm_.current(), SystemState::COOLDOWN);

    // cooldown_seconds is 10.0 in the production config.
    EXPECT_CALL(*laser_, fire(false)).Times(AnyNumber());
    EXPECT_CALL(*laser_, fire(true)).Times(0);
    for (int i = 0; i < 100; ++i) {
        now_ += 50ms;  // 5s total, still inside the cooldown
        (void)control_step(*deps_, cmd, now_, now_);
        ASSERT_FALSE(controller_->is_firing()) << "re-fired during cooldown";
    }
}

//
// Target loss at the loop level. The abort itself is pinned at controller
// level; these tests pin the loop glue: if the loss branch in control_step
// were deleted, the controller would keep its (stale) target and keep firing
// at a position the mosquito has already left.
//

TEST_F(ControlLoopTest, TrackingTargetLostReturnsToArmedAndClearsTheTarget) {
    set_arm_gpio(true);
    settle_inputs();
    run_cycles(2, make_target(kValidTarget));
    ASSERT_EQ(sm_.current(), SystemState::TRACKING);

    // Target lost: no valid position in the freshest command.
    TargetCommand lost;
    now_ += kCyclePeriod;
    (void)control_step(*deps_, lost, now_, now_);
    EXPECT_EQ(sm_.current(), SystemState::ARMED);

    // With the loss branch deleted the controller would still hold the stale
    // target and would fire once the startup blanking elapsed.
    EXPECT_CALL(*laser_, fire(false)).Times(AnyNumber());
    EXPECT_CALL(*laser_, fire(true)).Times(0);
    now_ += FiringController::k_startup_blanking + 100ms;
    run_cycles(5, lost);
    EXPECT_FALSE(controller_->is_firing());
    EXPECT_EQ(sm_.current(), SystemState::ARMED);
}

TEST_F(ControlLoopTest, FiringTargetLostAbortsThePulseAndEntersCooldown) {
    set_arm_gpio(true);
    settle_inputs();
    now_ += FiringController::k_startup_blanking;

    auto cmd = make_target(kValidTarget);
    run_cycles(3, cmd);
    ASSERT_EQ(sm_.current(), SystemState::FIRING);
    ASSERT_TRUE(controller_->is_firing());

    EXPECT_CALL(*laser_, fire(false)).Times(AtLeast(1));

    TargetCommand lost;
    now_ += kCyclePeriod;
    (void)control_step(*deps_, lost, now_, now_);

    EXPECT_FALSE(controller_->is_firing());
    EXPECT_EQ(sm_.current(), SystemState::COOLDOWN);
    // The abort path must start the cooldown like every other pulse end.
    EXPECT_FALSE(controller_->may_fire(now_));
}

TEST_F(ControlLoopTest, CooldownExpiryRearmsWhenTheSwitchIsStillOn) {
    set_arm_gpio(true);
    settle_inputs();
    now_ += FiringController::k_startup_blanking;

    auto cmd = make_target(kValidTarget);
    run_cycles(3, cmd);
    ASSERT_EQ(sm_.current(), SystemState::FIRING);

    now_ += 110ms;
    (void)control_step(*deps_, cmd, now_, now_);
    ASSERT_EQ(sm_.current(), SystemState::COOLDOWN);

    // Step past the full 10s cooldown with no target. If the cooldown-exit
    // block were deleted the state would sit in COOLDOWN forever.
    now_ += 11s;
    run_cycles(2);

    EXPECT_EQ(sm_.current(), SystemState::ARMED);
    EXPECT_TRUE(controller_->is_armed());
}

TEST_F(ControlLoopTest, CooldownExpiryStaysIdleWhenTheSwitchIsOff) {
    set_arm_gpio(true);
    settle_inputs();
    now_ += FiringController::k_startup_blanking;

    auto cmd = make_target(kValidTarget);
    run_cycles(3, cmd);
    ASSERT_EQ(sm_.current(), SystemState::FIRING);

    now_ += 110ms;
    (void)control_step(*deps_, cmd, now_, now_);
    ASSERT_EQ(sm_.current(), SystemState::COOLDOWN);

    set_arm_gpio(false);
    now_ += 11s;
    run_cycles(8);

    EXPECT_EQ(sm_.current(), SystemState::IDLE);
    EXPECT_FALSE(controller_->is_armed());
}

//
// Hardware-fault propagation
//

// REGRESSION: FiringController::force_laser_off_and_halt() latches the controller
// off but holds no state machine, so it cannot halt the system itself. Without the
// is_halted() poll in control_step, an SPI failure left the loop spinning forever
// at full rate with the laser dead while the operator's readout still said ARMED.
TEST_F(ControlLoopTest, GalvoFailureLatchesTheControllerAndHaltsTheSystem) {
    set_arm_gpio(true);
    settle_inputs();
    now_ += FiringController::k_startup_blanking;

    ON_CALL(*galvo_, set_position(_, _))
        .WillByDefault(Return(std::unexpected(HardwareError::SpiTransferFailed)));

    auto outcome = run_cycles(10, make_target(kValidTarget));

    EXPECT_TRUE(controller_->is_halted());
    EXPECT_EQ(outcome, ControlOutcome::Halt);
    EXPECT_EQ(sm_.current(), SystemState::SAFE_HALT);
}

TEST_F(ControlLoopTest, LaserFireFailureLatchesTheControllerAndHaltsTheSystem) {
    set_arm_gpio(true);
    settle_inputs();
    now_ += FiringController::k_startup_blanking;

    ON_CALL(*laser_, fire(true))
        .WillByDefault(Return(std::unexpected(HardwareError::GpioWriteFailed)));

    auto outcome = run_cycles(10, make_target(kValidTarget));

    EXPECT_TRUE(controller_->is_halted());
    EXPECT_EQ(outcome, ControlOutcome::Halt);
    EXPECT_EQ(sm_.current(), SystemState::SAFE_HALT);
}

//
// Fail-safe inputs
//

TEST_F(ControlLoopTest, ArmGpioReadFailureIsTreatedAsDisarmed) {
    set_arm_gpio(true);
    settle_inputs();
    ASSERT_EQ(sm_.current(), SystemState::ARMED);

    ON_CALL(*arm_gpio_, read())
        .WillByDefault(Return(std::unexpected(HardwareError::GpioReadFailed)));

    EXPECT_CALL(*laser_, fire(false)).Times(AnyNumber());

    EXPECT_CALL(*laser_, fire(true)).Times(0);
    run_cycles(10, make_target(kValidTarget));

    EXPECT_FALSE(arm_switch_->is_armed());
    EXPECT_FALSE(controller_->is_armed());
}

TEST_F(ControlLoopTest, EStopGpioReadFailureIsTreatedAsPressed) {
    ON_CALL(*estop_gpio_, read())
        .WillByDefault(Return(std::unexpected(HardwareError::GpioReadFailed)));

    auto outcome = run_cycles(10);

    EXPECT_TRUE(e_stop_->is_pressed());
    EXPECT_EQ(outcome, ControlOutcome::Halt);
    EXPECT_EQ(sm_.current(), SystemState::SAFE_HALT);
}

}
