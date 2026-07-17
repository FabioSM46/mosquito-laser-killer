#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <chrono>
#include <thread>
#include <memory>
#include <atomic>
#include <random>

#include "mocks/mock_galvo_driver.h"
#include "mocks/mock_laser.h"
#include "safety/system_state.h"
#include "safety/watchdog.h"
#include "core/error.h"

using namespace testing;
using namespace std::chrono_literals;

namespace {

constexpr auto kOk = std::expected<void, HardwareError>{};

// Real threads and real scheduling jitter, which is what AGENTS.md 7.2 asks for
// ("Delay Processing Thread by 20ms, 30ms, 50ms"). The previous version of this
// file had no threads, no delays and no contention — it was five copies of the
// unit test with synthetic timestamps, so it could not observe jitter at all.
//
// Exact timeout boundaries are pinned deterministically in unit/test_watchdog.cpp
// with injected time. This file deliberately uses a generous timeout so that
// ordinary scheduler noise cannot produce a false failure: what is under test is
// the behaviour under real concurrency, not the arithmetic.
class WatchdogJitterStressTest : public Test {
protected:
    void SetUp() override {
        mock_laser_ = std::make_unique<NiceMock<MockLaser>>();
        mock_galvo_ = std::make_unique<NiceMock<MockGalvoDriver>>();
        ON_CALL(*mock_laser_, emergency_shutdown()).WillByDefault(Return(kOk));
        ON_CALL(*mock_galvo_, zero()).WillByDefault(Return(kOk));
        ASSERT_TRUE(sm_.transition(SystemState::IDLE));
        ASSERT_TRUE(sm_.transition(SystemState::ARMED));
    }

    // Mirrors main: the producer publishes into an atomic, and the control thread
    // forwards that atomic to the watchdog every cycle whether or not it changed.
    std::atomic<std::chrono::steady_clock::time_point> heartbeat_{
        std::chrono::steady_clock::time_point::min()};

    SystemStateMachine sm_;
    std::unique_ptr<NiceMock<MockLaser>> mock_laser_;
    std::unique_ptr<NiceMock<MockGalvoDriver>> mock_galvo_;
};

TEST_F(WatchdogJitterStressTest, SustainedJitterUnderTheTimeoutDoesNotHalt) {
    constexpr auto kTimeout = 300ms;
    Watchdog wd(sm_, *mock_laser_, *mock_galvo_, kTimeout, 5000ms);

    std::atomic<bool> stop{false};
    std::atomic<bool> halted{false};

    // Producer: publishes at ~200Hz but with pseudo-random stalls of up to 50ms,
    // well inside the 300ms timeout.
    std::jthread producer([&] {
        std::mt19937 rng(12345);
        std::uniform_int_distribution<int> jitter_us(500, 50'000);
        while (!stop.load(std::memory_order_acquire)) {
            heartbeat_.store(std::chrono::steady_clock::now(),
                             std::memory_order_release);
            std::this_thread::sleep_for(std::chrono::microseconds(jitter_us(rng)));
        }
    });

    // Consumer: the control loop.
    std::jthread consumer([&] {
        while (!stop.load(std::memory_order_acquire)) {
            const auto now = std::chrono::steady_clock::now();
            wd.feed(heartbeat_.load(std::memory_order_acquire));
            if (!wd.check(now)) {
                halted.store(true, std::memory_order_release);
                return;
            }
            std::this_thread::sleep_for(5ms);
        }
    });

    std::this_thread::sleep_for(1s);
    stop.store(true, std::memory_order_release);
    producer.join();
    consumer.join();

    EXPECT_FALSE(halted.load()) << "jitter well inside the timeout halted the system";
    EXPECT_NE(sm_.current(), SystemState::SAFE_HALT);
}

TEST_F(WatchdogJitterStressTest, RealProducerStallHaltsTheSystem) {
    constexpr auto kTimeout = 200ms;
    Watchdog wd(sm_, *mock_laser_, *mock_galvo_, kTimeout, 5000ms);

    std::atomic<bool> stop{false};
    std::atomic<bool> halted{false};

    // Producer runs healthily, then stops dead — a genuine pipeline stall.
    std::jthread producer([&] {
        for (int i = 0; i < 20 && !stop.load(std::memory_order_acquire); ++i) {
            heartbeat_.store(std::chrono::steady_clock::now(),
                             std::memory_order_release);
            std::this_thread::sleep_for(5ms);
        }
        // Deliberately returns while the consumer keeps running.
    });

    std::jthread consumer([&] {
        while (!stop.load(std::memory_order_acquire)) {
            const auto now = std::chrono::steady_clock::now();
            wd.feed(heartbeat_.load(std::memory_order_acquire));
            if (!wd.check(now)) {
                halted.store(true, std::memory_order_release);
                return;
            }
            std::this_thread::sleep_for(5ms);
        }
    });

    // 100ms of production, then a stall far longer than the 200ms timeout.
    std::this_thread::sleep_for(1500ms);
    stop.store(true, std::memory_order_release);
    producer.join();
    consumer.join();

    EXPECT_TRUE(halted.load()) << "a dead producer did not trip the watchdog";
    EXPECT_EQ(sm_.current(), SystemState::SAFE_HALT);
}

// REGRESSION: the control thread forwards the heartbeat atomic every cycle even
// when the producer has never run. Seeding that atomic with a real timestamp made
// the very first feed() look like a genuine heartbeat, which destroyed the startup
// grace and made the watchdog measure wall-time-since-launch — halting ~16-32ms
// after start, before the cameras could open. This reproduces the exact
// producer-never-starts sequence against a real running control loop.
TEST_F(WatchdogJitterStressTest, ProducerThatNeverStartsRidesTheGraceThenHalts) {
    constexpr auto kGrace = 400ms;
    Watchdog wd(sm_, *mock_laser_, *mock_galvo_, 25ms, kGrace);

    std::atomic<bool> halted{false};
    std::atomic<int> cycles_survived{0};

    // No producer at all. The consumer forwards the untouched atomic every cycle.
    std::jthread consumer([&] {
        for (int i = 0; i < 200; ++i) {
            const auto now = std::chrono::steady_clock::now();
            wd.feed(heartbeat_.load(std::memory_order_acquire));
            if (!wd.check(now)) {
                halted.store(true, std::memory_order_release);
                return;
            }
            cycles_survived.fetch_add(1, std::memory_order_release);
            std::this_thread::sleep_for(5ms);
        }
    });
    consumer.join();

    // It must survive well past the 25ms timeout — the grace window is what lets
    // the cameras open — and then fail closed once the grace expires.
    EXPECT_GT(cycles_survived.load(), 10)
        << "halted almost immediately: the startup grace was destroyed by a "
           "stale heartbeat, exactly the bug this test exists for";
    EXPECT_TRUE(halted.load()) << "the grace window must be bounded";
    EXPECT_EQ(sm_.current(), SystemState::SAFE_HALT);
}

TEST_F(WatchdogJitterStressTest, ConcurrentFeedAndCheckDoNotCorruptTheHeartbeat) {
    // Race smoke test only: the 500ms timeout against a max 100ms staleness
    // means a naive store() in place of the compare-exchange would NOT trip
    // either, so this test cannot catch deletion of feed()'s strictly-newer
    // guard. That semantic is pinned exactly by WatchdogTest unit tests with
    // injected time; what this adds is concurrency coverage that the CAS and
    // the check path do not corrupt state or false-trip under contention.
    Watchdog wd(sm_, *mock_laser_, *mock_galvo_, 500ms, 5000ms);

    std::atomic<bool> stop{false};
    std::atomic<bool> halted{false};

    // Several threads feeding concurrently, including deliberately stale values,
    // to exercise the compare-exchange in feed().
    std::vector<std::jthread> feeders;
    for (int i = 0; i < 4; ++i) {
        feeders.emplace_back([&, i] {
            std::mt19937 rng(static_cast<unsigned>(i));
            std::uniform_int_distribution<int> stale_ms(0, 100);
            while (!stop.load(std::memory_order_acquire)) {
                const auto now = std::chrono::steady_clock::now();
                // Half the feeds are stale and must not rewind the timer.
                wd.feed(now - std::chrono::milliseconds(stale_ms(rng)));
                std::this_thread::sleep_for(1ms);
            }
        });
    }

    std::jthread checker([&] {
        while (!stop.load(std::memory_order_acquire)) {
            if (!wd.check(std::chrono::steady_clock::now())) {
                halted.store(true, std::memory_order_release);
                return;
            }
            std::this_thread::sleep_for(2ms);
        }
    });

    std::this_thread::sleep_for(500ms);
    stop.store(true, std::memory_order_release);
    for (auto& f : feeders) {
        f.join();
    }
    checker.join();

    EXPECT_FALSE(halted.load())
        << "concurrent feeds, including stale ones, must not trip a live watchdog";
}

}
