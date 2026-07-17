#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <chrono>
#include <thread>
#include <atomic>
#include <memory>
#include <csignal>

#include "mocks/mock_gpio.h"
#include "mocks/mock_galvo_driver.h"
#include "core/thread_safe_queue.h"
#include "core/types.h"
#include "core/error.h"
#include "hal/laser.h"
#include "safety/system_state.h"
#include "safety/watchdog.h"
#include "safety/arm_switch.h"
#include "safety/e_stop.h"
#include "safety/bounding_box.h"
#include "safety/signal_handler.h"
#include "control/coordinate_mapper.h"
#include "control/firing_controller.h"
#include "control/control_loop.h"

using namespace testing;
using namespace std::chrono_literals;

namespace {

constexpr auto kOk = std::expected<void, HardwareError>{};

// These tests drive a REAL Laser over a MockGpio and observe the actual pin, so
// "the laser is off" is an observation rather than an assertion about a mock the
// test itself called. The previous version of this file called
// mock_laser_->emergency_shutdown() from inside the test thread and then asserted
// the expectation it had just satisfied; another test constructed an unarmed
// controller (so set_target was rejected and there was nothing to turn off) and
// asserted EXPECT_FALSE on a value that is false on every path but one.
class ConcurrentShutdownStressTest : public Test {
protected:
    void SetUp() override {
        t0_ = std::chrono::steady_clock::now();

        auto gpio = std::make_unique<NiceMock<MockGpio>>();
        gpio_ = gpio.get();
        ON_CALL(*gpio_, set_direction_output()).WillByDefault(Return(kOk));
        // The pin state IS the safety property, so record every write.
        ON_CALL(*gpio_, write(_)).WillByDefault([this](bool value) {
            pin_high_.store(value, std::memory_order_release);
            if (value) {
                pin_went_high_.store(true, std::memory_order_release);
            }
            return kOk;
        });

        laser_ = std::make_unique<Laser>(std::move(gpio), 18,
                                         config_.max_pulse_duration_ms);

        galvo_ = std::make_unique<NiceMock<MockGalvoDriver>>();
        ON_CALL(*galvo_, set_position(_, _)).WillByDefault(Return(kOk));
        ON_CALL(*galvo_, zero()).WillByDefault([this]() {
            galvo_zeroed_.store(true, std::memory_order_release);
            return kOk;
        });

        // The GPIO levels are held in atomics that the default actions read, and
        // the actions are registered once here, before any thread starts.
        // Re-registering an ON_CALL from the test thread while a worker thread is
        // invoking the same mock is a genuine data race: gmock's ON_CALL registry
        // is a plain vector with no synchronisation against concurrent Invoke.
        auto arm_gpio = std::make_unique<NiceMock<MockGpio>>();
        arm_gpio_ = arm_gpio.get();
        ON_CALL(*arm_gpio_, set_direction_input()).WillByDefault(Return(kOk));
        ON_CALL(*arm_gpio_, read()).WillByDefault([this] {
            return std::expected<bool, HardwareError>(
                arm_high_.load(std::memory_order_acquire));
        });

        auto estop_gpio = std::make_unique<NiceMock<MockGpio>>();
        estop_gpio_ = estop_gpio.get();
        ON_CALL(*estop_gpio_, set_direction_input()).WillByDefault(Return(kOk));
        ON_CALL(*estop_gpio_, read()).WillByDefault([this] {
            // Active LOW: HIGH means released.
            return std::expected<bool, HardwareError>(
                !estop_pressed_.load(std::memory_order_acquire));
        });

        arm_switch_ = std::make_unique<ArmSwitch>(std::move(arm_gpio));
        e_stop_ = std::make_unique<EStop>(std::move(estop_gpio));
        ASSERT_TRUE(arm_switch_->initialize().has_value());
        ASSERT_TRUE(e_stop_->initialize().has_value());

        bbox_ = std::make_unique<BoundingBox3D>(config_.bounding_box);
        mapper_ = std::make_unique<CoordinateMapper>(
            *bbox_, config_.galvo_limits, config_.dac_ref_voltage, config_.galvo_driver);
        // Construct with a start_time in the past so the 1s startup blanking has
        // already elapsed and the system can actually fire during the test.
        controller_ = std::make_unique<FiringController>(
            *laser_, *galvo_, *mapper_, config_.max_pulse_duration_ms,
            config_.cooldown_seconds, config_.settle_delay_ms,
            t0_ - FiringController::k_startup_blanking - 1s);
        watchdog_ = std::make_unique<Watchdog>(sm_, *laser_, *galvo_, 500ms, 5000ms);

        ASSERT_TRUE(sm_.transition(SystemState::IDLE));
    }

    SystemConfig config_{};
    SystemStateMachine sm_;
    std::chrono::steady_clock::time_point t0_;

    std::atomic<bool> arm_high_{true};
    std::atomic<bool> estop_pressed_{false};
    std::atomic<bool> pin_high_{false};
    std::atomic<bool> pin_went_high_{false};
    std::atomic<bool> galvo_zeroed_{false};

    NiceMock<MockGpio>* gpio_{nullptr};
    NiceMock<MockGpio>* arm_gpio_{nullptr};
    NiceMock<MockGpio>* estop_gpio_{nullptr};
    std::unique_ptr<Laser> laser_;
    std::unique_ptr<NiceMock<MockGalvoDriver>> galvo_;
    std::unique_ptr<ArmSwitch> arm_switch_;
    std::unique_ptr<EStop> e_stop_;
    std::unique_ptr<BoundingBox3D> bbox_;
    std::unique_ptr<CoordinateMapper> mapper_;
    std::unique_ptr<FiringController> controller_;
    std::unique_ptr<Watchdog> watchdog_;
};

// AGENTS.md 7.2: "Signal while all threads active; verify laser LOW within one
// cycle." Runs the real three-thread topology against the real control_step, gets
// the laser genuinely firing, then requests shutdown.
TEST_F(ConcurrentShutdownStressTest, ShutdownWhileFiringForcesLaserLowAndZeroesGalvos) {
    ThreadSafeQueue<StereoFrame> frame_queue;
    ThreadSafeQueue<TargetCommand> target_queue;
    std::atomic<std::chrono::steady_clock::time_point> heartbeat{
        std::chrono::steady_clock::time_point::min()};
    std::atomic<bool> shutdown_requested{false};

    std::jthread capture_thread([&] {
        uint64_t id = 0;
        while (!shutdown_requested.load(std::memory_order_acquire)) {
            StereoFrame f;
            f.frame_id = id++;
            f.timestamp = std::chrono::steady_clock::now();
            frame_queue.push(std::move(f));
            std::this_thread::sleep_for(4ms);
        }
    });

    std::jthread processing_thread([&] {
        while (!shutdown_requested.load(std::memory_order_acquire)) {
            auto frames = frame_queue.drain_all();
            if (frames.empty()) {
                std::this_thread::sleep_for(1ms);
                continue;
            }
            heartbeat.store(std::chrono::steady_clock::now(),
                            std::memory_order_release);

            TargetCommand cmd;
            cmd.frame_id = frames.back().frame_id;
            cmd.timestamp = frames.back().timestamp;
            cmd.target_valid = true;
            cmd.target_position = Point3D{0.0, 0.0, 0.7};
            target_queue.push(std::move(cmd));
        }
    });

    std::jthread control_thread([&] {
        ControlDeps deps{sm_, *controller_, *watchdog_, *arm_switch_, *e_stop_, *laser_};
        while (!shutdown_requested.load(std::memory_order_acquire)) {
            const auto now = std::chrono::steady_clock::now();

            std::optional<TargetCommand> cmd;
            auto commands = target_queue.drain_all();
            if (!commands.empty()) {
                cmd = std::move(commands.back());
            }

            if (control_step(deps, cmd, heartbeat.load(std::memory_order_acquire),
                             now) == ControlOutcome::Halt) {
                break;
            }
            std::this_thread::sleep_for(2ms);
        }

        // The real shutdown path from main's control thread.
        (void)laser_->emergency_shutdown();
        (void)galvo_->zero();
    });

    // Let the system arm, track and actually fire.
    std::this_thread::sleep_for(200ms);
    ASSERT_TRUE(pin_went_high_.load())
        << "the laser never fired, so this test would prove nothing about turning "
           "it off";

    const auto shutdown_at = std::chrono::steady_clock::now();
    shutdown_requested.store(true, std::memory_order_release);

    capture_thread.join();
    processing_thread.join();
    control_thread.join();
    const auto laser_off_at = std::chrono::steady_clock::now();

    EXPECT_FALSE(pin_high_.load()) << "laser pin still HIGH after shutdown";
    EXPECT_TRUE(galvo_zeroed_.load()) << "galvos were never re-centred";
    EXPECT_LT(laser_off_at - shutdown_at, 100ms)
        << "laser did not go LOW promptly after the shutdown request";
}

// The E-stop is checked by the control thread itself, so it must win against a
// live firing loop.
TEST_F(ConcurrentShutdownStressTest, EStopDuringLiveFiringForcesLaserLow) {
    std::atomic<bool> stop{false};
    std::atomic<bool> halted{false};
    std::atomic<std::chrono::steady_clock::time_point> heartbeat{
        std::chrono::steady_clock::time_point::min()};

    std::jthread producer([&] {
        while (!stop.load(std::memory_order_acquire)) {
            heartbeat.store(std::chrono::steady_clock::now(),
                            std::memory_order_release);
            std::this_thread::sleep_for(2ms);
        }
    });

    std::jthread control_thread([&] {
        ControlDeps deps{sm_, *controller_, *watchdog_, *arm_switch_, *e_stop_, *laser_};
        TargetCommand cmd;
        cmd.target_valid = true;
        cmd.target_position = Point3D{0.0, 0.0, 0.7};

        while (!stop.load(std::memory_order_acquire)) {
            const auto now = std::chrono::steady_clock::now();
            if (control_step(deps, cmd, heartbeat.load(std::memory_order_acquire),
                             now) == ControlOutcome::Halt) {
                halted.store(true, std::memory_order_release);
                break;
            }
            std::this_thread::sleep_for(2ms);
        }
        (void)laser_->emergency_shutdown();
    });

    std::this_thread::sleep_for(150ms);
    ASSERT_TRUE(pin_went_high_.load()) << "the laser never fired";

    // Press the mushroom. Flipping an atomic the mock's default action reads —
    // rather than re-registering ON_CALL while the control thread is calling
    // read() — keeps this free of a data race in the test harness itself.
    estop_pressed_.store(true, std::memory_order_release);

    std::this_thread::sleep_for(150ms);
    stop.store(true, std::memory_order_release);
    producer.join();
    control_thread.join();

    EXPECT_TRUE(halted.load()) << "E-stop did not halt the control loop";
    EXPECT_FALSE(pin_high_.load()) << "laser pin still HIGH after E-stop";
    EXPECT_EQ(sm_.current(), SystemState::SAFE_HALT);
}

// ~Laser writes the pin LOW. main declares the laser AFTER the galvo precisely so
// that reverse-order destruction runs ~Laser first — the ordering AGENTS.md 4.6
// mandates and that the declaration order previously inverted.
TEST_F(ConcurrentShutdownStressTest, LaserDestructorForcesPinLow) {
    auto gpio = std::make_unique<NiceMock<MockGpio>>();
    auto* raw_gpio = gpio.get();
    std::atomic<bool> pin{false};

    ON_CALL(*raw_gpio, set_direction_output()).WillByDefault(Return(kOk));
    ON_CALL(*raw_gpio, write(_)).WillByDefault([&pin](bool value) {
        pin.store(value);
        return kOk;
    });

    {
        auto laser = std::make_unique<Laser>(std::move(gpio), 18, 100.0);
        ASSERT_TRUE(laser->fire(true).has_value());
        ASSERT_TRUE(pin.load()) << "the pin must be HIGH for the destructor to matter";
    }

    EXPECT_FALSE(pin.load()) << "~Laser did not force the pin LOW";
}

// SignalHandler only sets an atomic; the worker threads poll it. This pins the
// real chain — signal -> flag -> loop exit -> laser LOW — rather than raising a
// signal and then calling emergency_shutdown() by hand, which is what the old
// SignalHandlerTest did (there was no causal link between the two at all).
TEST_F(ConcurrentShutdownStressTest, SignalFlagStopsTheLoopAndForcesLaserLow) {
    SignalHandler sh;
    sh.install();
    sh.reset();

    std::atomic<std::chrono::steady_clock::time_point> heartbeat{
        std::chrono::steady_clock::time_point::min()};
    std::atomic<bool> loop_exited{false};

    std::jthread producer([&] {
        while (!sh.is_shutdown_requested()) {
            heartbeat.store(std::chrono::steady_clock::now(),
                            std::memory_order_release);
            std::this_thread::sleep_for(2ms);
        }
    });

    std::jthread control_thread([&] {
        ControlDeps deps{sm_, *controller_, *watchdog_, *arm_switch_, *e_stop_, *laser_};
        TargetCommand cmd;
        cmd.target_valid = true;
        cmd.target_position = Point3D{0.0, 0.0, 0.7};

        // The same three-condition poll main's threads use.
        while (!sh.is_shutdown_requested()) {
            const auto now = std::chrono::steady_clock::now();
            if (control_step(deps, cmd, heartbeat.load(std::memory_order_acquire),
                             now) == ControlOutcome::Halt) {
                break;
            }
            std::this_thread::sleep_for(2ms);
        }
        (void)laser_->emergency_shutdown();
        (void)galvo_->zero();
        loop_exited.store(true, std::memory_order_release);
    });

    std::this_thread::sleep_for(150ms);
    ASSERT_TRUE(pin_went_high_.load()) << "the laser never fired";

    std::raise(SIGINT);

    producer.join();
    control_thread.join();

    EXPECT_TRUE(loop_exited.load()) << "SIGINT did not stop the control loop";
    EXPECT_FALSE(pin_high_.load()) << "laser pin still HIGH after SIGINT";
    EXPECT_TRUE(galvo_zeroed_.load()) << "galvos were never re-centred after SIGINT";
}

}
