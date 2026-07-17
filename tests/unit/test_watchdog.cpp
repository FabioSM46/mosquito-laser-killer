#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <chrono>
#include <memory>
#include <atomic>

#include "mocks/mock_galvo_driver.h"
#include "mocks/mock_laser.h"
#include "safety/system_state.h"
#include "safety/watchdog.h"
#include "core/error.h"

using namespace testing;
using namespace std::chrono_literals;

namespace {

constexpr auto kOk = std::expected<void, HardwareError>{};
constexpr auto kTimeout = 25ms;
constexpr auto kGrace = 5000ms;

class WatchdogTest : public Test {
protected:
    void SetUp() override {
        mock_laser_ = std::make_unique<NiceMock<MockLaser>>();
        mock_galvo_ = std::make_unique<NiceMock<MockGalvoDriver>>();
        ON_CALL(*mock_laser_, emergency_shutdown()).WillByDefault(Return(kOk));
        ON_CALL(*mock_galvo_, zero()).WillByDefault(Return(kOk));
        t0_ = std::chrono::steady_clock::now();
    }

    auto make_watchdog() -> std::unique_ptr<Watchdog> {
        return std::make_unique<Watchdog>(sm_, *mock_laser_, *mock_galvo_, kTimeout, kGrace);
    }

    void arm_state_machine() {
        ASSERT_TRUE(sm_.transition(SystemState::IDLE));
        ASSERT_TRUE(sm_.transition(SystemState::ARMED));
    }

    SystemStateMachine sm_;
    std::unique_ptr<NiceMock<MockLaser>> mock_laser_;
    std::unique_ptr<NiceMock<MockGalvoDriver>> mock_galvo_;
    std::chrono::steady_clock::time_point t0_;
};

// REGRESSION: main publishes the producer's heartbeat into an atomic, and the
// control thread forwards that atomic to feed() every cycle whether or not the
// producer has run. The atomic was seeded with steady_clock::now() at startup, so
// the very first feed() installed that timestamp as a genuine heartbeat, which
// destroyed the startup grace and made check() measure wall-time-since-launch.
// The watchdog then halted ~16-32ms after start — long before the USB cameras
// finish opening — and SAFE_HALT is terminal, so the system could never reach
// ARMED. main now seeds the atomic with the sentinel; this pins the feed() half of
// the contract that makes that work.
TEST_F(WatchdogTest, ForwardingTheUnpublishedSentinelPreservesTheStartupGrace) {
    arm_state_machine();
    auto wd = make_watchdog();

    // Exactly what main's control thread does before the processing thread runs.
    std::atomic<std::chrono::steady_clock::time_point> heartbeat{
        std::chrono::steady_clock::time_point::min()};

    for (int cycle = 0; cycle < 200; ++cycle) {
        const auto now = t0_ + cycle * 5ms;
        wd->feed(heartbeat.load(std::memory_order_acquire));
        // 1s of cycles, 40x the 25ms timeout: only the grace can carry this.
        ASSERT_TRUE(wd->check(now)) << "halted at cycle " << cycle;
    }

    EXPECT_FALSE(wd->has_heartbeat())
        << "an unpublished sentinel must never count as a heartbeat";
    EXPECT_NE(sm_.current(), SystemState::SAFE_HALT);
}

// The complement: a producer that published once and then died must still trip
// the watchdog. The strictly-newer feed() must not turn a re-forwarded real
// heartbeat into a keepalive.
TEST_F(WatchdogTest, RepeatedlyForwardingOneRealHeartbeatStillTimesOut) {
    arm_state_machine();
    auto wd = make_watchdog();

    const auto published_once = t0_;

    bool halted = false;
    for (int cycle = 0; cycle < 50 && !halted; ++cycle) {
        const auto now = t0_ + cycle * 5ms;
        wd->feed(published_once);  // the producer is dead; the value never changes
        halted = !wd->check(now);
    }

    EXPECT_TRUE(halted) << "a dead producer's stale heartbeat kept the watchdog alive";
    EXPECT_EQ(sm_.current(), SystemState::SAFE_HALT);
}

TEST_F(WatchdogTest, StartupGraceExpiresIntoSafeHaltIfProducerNeverRuns) {
    arm_state_machine();
    auto wd = make_watchdog();

    EXPECT_TRUE(wd->check(t0_));
    EXPECT_TRUE(wd->check(t0_ + kGrace));

    // The grace window is bounded: a producer that never starts must be caught.
    EXPECT_FALSE(wd->check(t0_ + kGrace + 1ms));
    EXPECT_EQ(sm_.current(), SystemState::SAFE_HALT);
}

TEST_F(WatchdogTest, FirstRealHeartbeatEndsTheStartupGrace) {
    arm_state_machine();
    auto wd = make_watchdog();

    EXPECT_TRUE(wd->check(t0_));
    EXPECT_FALSE(wd->has_heartbeat());

    wd->feed(t0_ + 100ms);
    EXPECT_TRUE(wd->has_heartbeat());

    // The ordinary timeout now applies, measured from the heartbeat.
    EXPECT_TRUE(wd->check(t0_ + 100ms + kTimeout));
    EXPECT_FALSE(wd->check(t0_ + 100ms + kTimeout + 1ms));
    EXPECT_EQ(sm_.current(), SystemState::SAFE_HALT);
}

TEST_F(WatchdogTest, FeedRejectsAHeartbeatOlderThanTheLastAccepted) {
    auto wd = make_watchdog();

    wd->feed(t0_ + 100ms);
    wd->feed(t0_ + 50ms);  // must not rewind the timer

    // Had the rewind been accepted, elapsed would be 60ms > 25ms and this halts.
    EXPECT_TRUE(wd->check(t0_ + 110ms));
    EXPECT_NE(sm_.current(), SystemState::SAFE_HALT);
}

TEST_F(WatchdogTest, FreshHeartbeatKeepsTheSystemRunning) {
    arm_state_machine();
    auto wd = make_watchdog();

    for (int cycle = 1; cycle <= 100; ++cycle) {
        const auto now = t0_ + cycle * 5ms;
        wd->feed(now);
        ASSERT_TRUE(wd->check(now)) << "halted at cycle " << cycle;
    }

    EXPECT_NE(sm_.current(), SystemState::SAFE_HALT);
}

TEST_F(WatchdogTest, TimeoutBoundaryIsExclusive) {
    arm_state_machine();
    auto wd = make_watchdog();
    wd->feed(t0_);

    EXPECT_TRUE(wd->check(t0_ + kTimeout));
    EXPECT_FALSE(wd->check(t0_ + kTimeout + 1us));
    EXPECT_EQ(sm_.current(), SystemState::SAFE_HALT);
}

TEST_F(WatchdogTest, TimeoutIsAnAbsoluteDurationNotAFrameCount) {
    arm_state_machine();

    // There is no fps parameter to couple the interlock to. Deriving the timeout
    // from target_fps silently tightened it from 25ms to 14.3ms when target_fps
    // was raised to 210 for performance.
    Watchdog wd(sm_, *mock_laser_, *mock_galvo_, 25ms, kGrace);
    wd.feed(t0_);

    EXPECT_TRUE(wd.check(t0_ + 24ms));
    EXPECT_FALSE(wd.check(t0_ + 26ms));
}

TEST_F(WatchdogTest, TriggerForcesLaserOffAndZeroesGalvosBeforeHalting) {
    arm_state_machine();

    auto laser = std::make_unique<NiceMock<MockLaser>>();
    auto galvo = std::make_unique<NiceMock<MockGalvoDriver>>();

    // The name claims ordering, so pin it: the beam must be off and the galvos
    // centred BEFORE the state machine reaches SAFE_HALT, not merely sometime
    // during the same call.
    testing::Sequence seq;
    EXPECT_CALL(*laser, emergency_shutdown())
        .Times(1).InSequence(seq).WillOnce(Return(kOk));
    EXPECT_CALL(*galvo, zero())
        .Times(1).InSequence(seq).WillOnce(Return(kOk));

    Watchdog wd(sm_, *laser, *galvo, kTimeout, kGrace);
    wd.feed(t0_);

    EXPECT_FALSE(wd.check(t0_ + 100ms));
    EXPECT_EQ(sm_.current(), SystemState::SAFE_HALT);
}

TEST_F(WatchdogTest, TriggerIsLatchedAndFiresShutdownOnlyOnce) {
    arm_state_machine();

    auto laser = std::make_unique<NiceMock<MockLaser>>();
    auto galvo = std::make_unique<NiceMock<MockGalvoDriver>>();
    EXPECT_CALL(*laser, emergency_shutdown()).Times(1).WillOnce(Return(kOk));
    EXPECT_CALL(*galvo, zero()).Times(1).WillOnce(Return(kOk));

    Watchdog wd(sm_, *laser, *galvo, kTimeout, kGrace);
    wd.feed(t0_);

    EXPECT_FALSE(wd.check(t0_ + 100ms));
    EXPECT_FALSE(wd.check(t0_ + 200ms));
    EXPECT_FALSE(wd.check(t0_ + 300ms));
    EXPECT_TRUE(wd.triggered());
}

TEST_F(WatchdogTest, HaltingSurvivesLaserAndGalvoFailure) {
    arm_state_machine();

    auto laser = std::make_unique<NiceMock<MockLaser>>();
    auto galvo = std::make_unique<NiceMock<MockGalvoDriver>>();

    // Even if the hardware refuses, the machine must still reach SAFE_HALT.
    EXPECT_CALL(*laser, emergency_shutdown())
        .WillOnce(Return(std::unexpected(HardwareError::GpioWriteFailed)));
    EXPECT_CALL(*galvo, zero())
        .WillOnce(Return(std::unexpected(HardwareError::SpiTransferFailed)));

    Watchdog wd(sm_, *laser, *galvo, kTimeout, kGrace);
    wd.feed(t0_);

    EXPECT_FALSE(wd.check(t0_ + 100ms));
    EXPECT_EQ(sm_.current(), SystemState::SAFE_HALT);
}

TEST_F(WatchdogTest, TimeSinceHeartbeatIsMaxBeforeAnyHeartbeat) {
    auto wd = make_watchdog();
    EXPECT_EQ(wd->time_since_heartbeat(t0_), std::chrono::steady_clock::duration::max());

    wd->feed(t0_);
    EXPECT_EQ(wd->time_since_heartbeat(t0_ + 10ms),
              std::chrono::duration_cast<std::chrono::steady_clock::duration>(10ms));
}

}
