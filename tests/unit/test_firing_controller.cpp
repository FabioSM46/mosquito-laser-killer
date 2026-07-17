#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <chrono>
#include <cmath>
#include <memory>

#include "mocks/mock_galvo_driver.h"
#include "mocks/mock_laser.h"
#include "core/types.h"
#include "core/error.h"
#include "safety/bounding_box.h"
#include "control/coordinate_mapper.h"
#include "control/firing_controller.h"

using namespace testing;
using namespace std::chrono_literals;

namespace {

constexpr auto kOk = std::expected<void, HardwareError>{};

// Every test below runs the production config from core/types.h and drives time
// explicitly. The previous fixture used a +/-2m box with a +/-25 deg cone and
// cooldown_s = 0 — a configuration this project's own config_validator rejects as
// a critical error — and three tests slept 1050ms of real time to work around the
// controller reading a hidden clock.
class FiringControllerTest : public Test {
protected:
    void SetUp() override {
        mock_laser_ = std::make_unique<NiceMock<MockLaser>>();
        mock_galvo_ = std::make_unique<NiceMock<MockGalvoDriver>>();
        ON_CALL(*mock_laser_, fire(_)).WillByDefault(Return(kOk));
        ON_CALL(*mock_laser_, emergency_shutdown()).WillByDefault(Return(kOk));
        ON_CALL(*mock_galvo_, set_position(_, _)).WillByDefault(Return(kOk));
        ON_CALL(*mock_galvo_, zero()).WillByDefault(Return(kOk));

        t0_ = std::chrono::steady_clock::now();

        bbox_ = std::make_unique<BoundingBox3D>(config_.bounding_box);
        mapper_ = std::make_unique<CoordinateMapper>(
            *bbox_, config_.galvo_limits, config_.dac_ref_voltage, config_.galvo_driver);
        controller_ = make_controller(config_.max_pulse_duration_ms,
                                      config_.cooldown_seconds,
                                      config_.settle_delay_ms);
    }

    auto make_controller(double max_pulse_ms, double cooldown_s, double settle_ms)
        -> std::unique_ptr<FiringController> {
        return std::make_unique<FiringController>(
            *mock_laser_, *mock_galvo_, *mapper_,
            max_pulse_ms, cooldown_s, settle_ms, t0_);
    }

    // Past the 1s startup blanking that every config applies.
    [[nodiscard]] auto after_blanking() const -> std::chrono::steady_clock::time_point {
        return t0_ + FiringController::k_startup_blanking + 1ms;
    }

    // Arms, targets and steps until the pulse starts. Returns the time the laser
    // went ON.
    auto fire_once(FiringController& fc, Point3D target = kValidTarget)
        -> std::chrono::steady_clock::time_point {
        auto now = after_blanking();
        fc.set_armed(true, now);
        fc.set_target(target, now);
        for (int i = 0; i < 10 && !fc.is_firing(); ++i) {
            now += 5ms;
            (void)fc.execute_cycle(now);
        }
        return now;
    }

    // Inside the production box: x,y in [-0.09, 0.09], z in [0.5, 1.0].
    static constexpr Point3D kValidTarget{0.0, 0.0, 0.7};

    SystemConfig config_{};
    std::chrono::steady_clock::time_point t0_;
    std::unique_ptr<NiceMock<MockLaser>> mock_laser_;
    std::unique_ptr<NiceMock<MockGalvoDriver>> mock_galvo_;
    std::unique_ptr<BoundingBox3D> bbox_;
    std::unique_ptr<CoordinateMapper> mapper_;
    std::unique_ptr<FiringController> controller_;
};

//
// Startup blanking
//

TEST_F(FiringControllerTest, StartupBlankingPreventsFiringImmediatelyAfterBoot) {
    EXPECT_FALSE(controller_->may_fire(t0_));
    EXPECT_FALSE(controller_->may_fire(t0_ + 999ms));
    EXPECT_TRUE(controller_->may_fire(t0_ + FiringController::k_startup_blanking));
}

TEST_F(FiringControllerTest, NoFireDuringStartupBlankingEvenWhenArmedAndTargeted) {
    // fire(false) is routine; naming fire(true) at all makes non-matching calls
    // "unexpected" rather than "uninteresting", even under NiceMock.
    EXPECT_CALL(*mock_laser_, fire(false)).Times(AnyNumber());
    EXPECT_CALL(*mock_laser_, fire(true)).Times(0);

    auto now = t0_;
    controller_->set_armed(true, now);
    controller_->set_target(kValidTarget, now);
    for (int i = 0; i < 100; ++i) {
        now += 5ms;  // 500ms, still inside the 1s blanking
        (void)controller_->execute_cycle(now);
    }
    EXPECT_FALSE(controller_->is_firing());
}

//
// Arm gate
//

TEST_F(FiringControllerTest, SetTargetIsRejectedWhenDisarmed) {
    EXPECT_CALL(*mock_galvo_, set_position(_, _)).Times(0);
    // fire(false) is routine; naming fire(true) at all makes non-matching calls
    // "unexpected" rather than "uninteresting", even under NiceMock.
    EXPECT_CALL(*mock_laser_, fire(false)).Times(AnyNumber());
    EXPECT_CALL(*mock_laser_, fire(true)).Times(0);

    auto now = after_blanking();
    controller_->set_target(kValidTarget, now);
    (void)controller_->execute_cycle(now);
    (void)controller_->execute_cycle(now + 5ms);

    EXPECT_FALSE(controller_->is_firing());
}

TEST_F(FiringControllerTest, DisarmDuringPulseAbortsImmediately) {
    auto fire_time = fire_once(*controller_);
    ASSERT_TRUE(controller_->is_firing());

    EXPECT_CALL(*mock_laser_, fire(false)).Times(AtLeast(1));
    controller_->disarm(fire_time + 10ms);

    EXPECT_FALSE(controller_->is_firing());
    EXPECT_FALSE(controller_->is_armed());
    // The disarm-abort is a pulse end like any other: the cooldown must be
    // running, or a re-arm could re-fire immediately after a 100ms pulse.
    EXPECT_FALSE(controller_->may_fire(fire_time + 10ms));
    EXPECT_FALSE(controller_->may_fire(fire_time + 10ms + 9s));
    EXPECT_TRUE(controller_->may_fire(fire_time + 10ms + 10s));
}

//
// Max pulse duration
//

TEST_F(FiringControllerTest, PulseEndsExactlyAtMaxDuration) {
    auto fire_time = fire_once(*controller_);
    ASSERT_TRUE(controller_->is_firing());

    // One tick short of the limit: still firing.
    EXPECT_FALSE(controller_->execute_cycle(fire_time + 99ms));
    EXPECT_TRUE(controller_->is_firing());

    // At the limit: the pulse ends.
    EXPECT_TRUE(controller_->execute_cycle(fire_time + 100ms));
    EXPECT_FALSE(controller_->is_firing());
}

TEST_F(FiringControllerTest, PulseNeverExceedsMaxDurationEvenWithLateCycles) {
    auto fire_time = fire_once(*controller_);
    ASSERT_TRUE(controller_->is_firing());

    EXPECT_CALL(*mock_laser_, fire(false)).Times(AtLeast(1));

    // The control thread stalls and returns 500ms late: the pulse must end on the
    // very next cycle regardless.
    EXPECT_TRUE(controller_->execute_cycle(fire_time + 500ms));
    EXPECT_FALSE(controller_->is_firing());
}

//
// Cooldown
//

TEST_F(FiringControllerTest, CooldownBlocksRefireForTheFullConfiguredDuration) {
    auto fire_time = fire_once(*controller_);
    ASSERT_TRUE(controller_->execute_cycle(fire_time + 100ms));
    const auto pulse_end = fire_time + 100ms;

    // cooldown_seconds = 10.0 in the production config.
    EXPECT_FALSE(controller_->may_fire(pulse_end));
    EXPECT_FALSE(controller_->may_fire(pulse_end + 5s));
    EXPECT_FALSE(controller_->may_fire(pulse_end + 9999ms));
    EXPECT_TRUE(controller_->may_fire(pulse_end + 10s));
}

TEST_F(FiringControllerTest, NoRefireDuringCooldownEvenWithAHeldTarget) {
    auto fire_time = fire_once(*controller_);
    auto now = fire_time + 100ms;
    ASSERT_TRUE(controller_->execute_cycle(now));

    // fire(false) is routine; naming fire(true) at all makes non-matching calls
    // "unexpected" rather than "uninteresting", even under NiceMock.
    EXPECT_CALL(*mock_laser_, fire(false)).Times(AnyNumber());
    EXPECT_CALL(*mock_laser_, fire(true)).Times(0);

    for (int i = 0; i < 200; ++i) {
        now += 25ms;  // 5s total, well inside the 10s cooldown
        controller_->set_target(kValidTarget, now);
        (void)controller_->execute_cycle(now);
        ASSERT_FALSE(controller_->is_firing()) << "re-fired during cooldown at " << i;
    }
}

TEST_F(FiringControllerTest, AbortedPulseAlsoStartsTheCooldown) {
    auto fire_time = fire_once(*controller_);
    ASSERT_TRUE(controller_->is_firing());

    // Abort via target loss rather than the clean max-duration path.
    controller_->clear_target(fire_time + 10ms);
    ASSERT_FALSE(controller_->is_firing());

    EXPECT_FALSE(controller_->may_fire(fire_time + 10ms));
    EXPECT_FALSE(controller_->may_fire(fire_time + 5s));
    EXPECT_TRUE(controller_->may_fire(fire_time + 10ms + 10s));
}

TEST_F(FiringControllerTest, TargetChangeDuringPulseAbortsAndCoolsDown) {
    auto fire_time = fire_once(*controller_);
    ASSERT_TRUE(controller_->is_firing());

    EXPECT_CALL(*mock_laser_, fire(false)).Times(AtLeast(1));
    controller_->set_target({0.02, 0.02, 0.7}, fire_time + 10ms);

    EXPECT_FALSE(controller_->is_firing());
    EXPECT_FALSE(controller_->may_fire(fire_time + 5s));
}

//
// Motion blanking
//

TEST_F(FiringControllerTest, NoGalvoWriteWhileThePulseIsActive) {
    auto fire_time = fire_once(*controller_);
    ASSERT_TRUE(controller_->is_firing());

    EXPECT_CALL(*mock_galvo_, set_position(_, _)).Times(0);
    EXPECT_FALSE(controller_->execute_cycle(fire_time + 10ms));
    EXPECT_FALSE(controller_->execute_cycle(fire_time + 20ms));
}

TEST_F(FiringControllerTest, DoesNotFireBeforeTheSettleDelayElapses) {
    auto now = after_blanking();
    controller_->set_armed(true, now);
    controller_->set_target(kValidTarget, now);

    // fire(false) is routine; naming fire(true) at all makes non-matching calls
    // "unexpected" rather than "uninteresting", even under NiceMock.
    EXPECT_CALL(*mock_laser_, fire(false)).Times(AnyNumber());
    EXPECT_CALL(*mock_laser_, fire(true)).Times(0);

    // settle_delay_ms = 3.0. The first cycle stamps galvo_command_time_ = now, so
    // elapsed is 0 and the galvo cannot be settled yet.
    (void)controller_->execute_cycle(now);
    EXPECT_FALSE(controller_->is_firing());

    (void)controller_->execute_cycle(now + 2ms);
    EXPECT_FALSE(controller_->is_firing());
}

TEST_F(FiringControllerTest, FiresOnceTheSettleDelayHasElapsed) {
    auto now = after_blanking();
    controller_->set_armed(true, now);
    controller_->set_target(kValidTarget, now);

    EXPECT_CALL(*mock_laser_, fire(true)).Times(1);

    (void)controller_->execute_cycle(now);        // stamps the galvo command
    (void)controller_->execute_cycle(now + 4ms);  // settle (3ms) has elapsed
    EXPECT_TRUE(controller_->is_firing());
}

// A settle delay of zero must not mark the galvo settled in the same cycle the
// DAC was written — that fires while the mirrors are still slewing, painting the
// beam across the scan field at full power. config_validator rejects settle = 0,
// and the comparison is strict so the controller fails closed regardless.
TEST_F(FiringControllerTest, ZeroSettleDelayStillDefersFiringByOneCycle) {
    auto fc = make_controller(100.0, 10.0, 0.0);
    auto now = after_blanking();

    fc->set_armed(true, now);
    fc->set_target(kValidTarget, now);

    (void)fc->execute_cycle(now);
    EXPECT_FALSE(fc->is_firing()) << "fired in the same cycle the galvo was commanded";
}

TEST_F(FiringControllerTest, GalvoIsCommandedBeforeTheLaserFires) {
    auto now = after_blanking();

    // Declared outside the sequence: fire(false) is routine and unordered.
    EXPECT_CALL(*mock_laser_, fire(false)).Times(AnyNumber());

    {
        // The galvo is re-commanded every cycle while the target holds, so the
        // property under test is the ordering, not the count: every DAC write
        // must precede the pin going HIGH.
        InSequence seq;
        EXPECT_CALL(*mock_galvo_, set_position(_, _))
            .Times(AtLeast(1))
            .WillRepeatedly(Return(kOk));
        EXPECT_CALL(*mock_laser_, fire(true)).Times(1).WillOnce(Return(kOk));
    }

    controller_->set_armed(true, now);
    controller_->set_target(kValidTarget, now);
    (void)controller_->execute_cycle(now);
    (void)controller_->execute_cycle(now + 4ms);
    EXPECT_TRUE(controller_->is_firing());
}

//
// Coordinate bounds
//

TEST_F(FiringControllerTest, TargetOutsideTheBoxNeverReachesTheGalvoOrLaser) {
    EXPECT_CALL(*mock_galvo_, set_position(_, _)).Times(0);
    // fire(false) is routine; naming fire(true) at all makes non-matching calls
    // "unexpected" rather than "uninteresting", even under NiceMock.
    EXPECT_CALL(*mock_laser_, fire(false)).Times(AnyNumber());
    EXPECT_CALL(*mock_laser_, fire(true)).Times(0);

    auto now = after_blanking();
    controller_->set_armed(true, now);
    controller_->set_target({0.0, 0.0, 3.0}, now);  // z beyond z_max = 1.0

    for (int i = 0; i < 10; ++i) {
        now += 5ms;
        (void)controller_->execute_cycle(now);
    }
    EXPECT_FALSE(controller_->is_firing());
}

TEST_F(FiringControllerTest, NonFiniteTargetNeverReachesTheGalvoOrLaser) {
    EXPECT_CALL(*mock_galvo_, set_position(_, _)).Times(0);
    // fire(false) is routine; naming fire(true) at all makes non-matching calls
    // "unexpected" rather than "uninteresting", even under NiceMock.
    EXPECT_CALL(*mock_laser_, fire(false)).Times(AnyNumber());
    EXPECT_CALL(*mock_laser_, fire(true)).Times(0);

    auto now = after_blanking();
    controller_->set_armed(true, now);
    controller_->set_target({std::nan(""), 0.0, 0.7}, now);

    for (int i = 0; i < 10; ++i) {
        now += 5ms;
        (void)controller_->execute_cycle(now);
    }
    EXPECT_FALSE(controller_->is_firing());
}

//
// Hardware faults latch the controller
//

TEST_F(FiringControllerTest, GalvoWriteFailureLatchesTheControllerHalted) {
    ON_CALL(*mock_galvo_, set_position(_, _))
        .WillByDefault(Return(std::unexpected(HardwareError::SpiTransferFailed)));
    EXPECT_CALL(*mock_laser_, emergency_shutdown()).Times(AtLeast(1));

    auto now = after_blanking();
    controller_->set_armed(true, now);
    controller_->set_target(kValidTarget, now);
    (void)controller_->execute_cycle(now);

    EXPECT_TRUE(controller_->is_halted());
    EXPECT_FALSE(controller_->is_firing());
    EXPECT_FALSE(controller_->is_armed());
}

TEST_F(FiringControllerTest, LaserFireFailureLatchesTheControllerHalted) {
    ON_CALL(*mock_laser_, fire(true))
        .WillByDefault(Return(std::unexpected(HardwareError::GpioWriteFailed)));

    auto now = after_blanking();
    controller_->set_armed(true, now);
    controller_->set_target(kValidTarget, now);
    (void)controller_->execute_cycle(now);
    (void)controller_->execute_cycle(now + 4ms);

    EXPECT_TRUE(controller_->is_halted());
    EXPECT_FALSE(controller_->is_firing());
}

TEST_F(FiringControllerTest, HaltedControllerCannotBeRearmedOrRetargeted) {
    controller_->emergency_stop();
    ASSERT_TRUE(controller_->is_halted());

    // fire(false) is routine; naming fire(true) at all makes non-matching calls
    // "unexpected" rather than "uninteresting", even under NiceMock.
    EXPECT_CALL(*mock_laser_, fire(false)).Times(AnyNumber());
    EXPECT_CALL(*mock_laser_, fire(true)).Times(0);
    EXPECT_CALL(*mock_galvo_, set_position(_, _)).Times(0);

    auto now = after_blanking();
    controller_->set_armed(true, now);
    EXPECT_FALSE(controller_->is_armed());

    controller_->set_target(kValidTarget, now);
    for (int i = 0; i < 10; ++i) {
        now += 5ms;
        (void)controller_->execute_cycle(now);
    }
    EXPECT_FALSE(controller_->is_firing());
    EXPECT_FALSE(controller_->may_fire(now + 1h));
}

//
// Emergency stop
//

TEST_F(FiringControllerTest, EmergencyStopDuringPulseForcesLaserOffAndLatches) {
    fire_once(*controller_);
    ASSERT_TRUE(controller_->is_firing());

    EXPECT_CALL(*mock_laser_, emergency_shutdown()).Times(AtLeast(1));
    controller_->emergency_stop();

    EXPECT_FALSE(controller_->is_firing());
    EXPECT_TRUE(controller_->is_halted());
    EXPECT_FALSE(controller_->is_armed());
}

TEST_F(FiringControllerTest, EmergencyStopIsIrreversible) {
    controller_->emergency_stop();

    // No amount of time or re-arming clears the latch.
    EXPECT_FALSE(controller_->may_fire(t0_ + 24h));
    controller_->set_armed(true, t0_ + 24h);
    EXPECT_FALSE(controller_->is_armed());
    EXPECT_TRUE(controller_->is_halted());
}

}
