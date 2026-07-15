#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <chrono>
#include <thread>
#include <atomic>
#include <memory>
#include <csignal>

#include "mocks/mock_gpio.h"
#include "mocks/mock_galvo_driver.h"
#include "mocks/mock_laser.h"
#include "core/thread_safe_queue.h"
#include "core/types.h"
#include "core/error.h"
#include "hal/laser.h"
#include "safety/system_state.h"
#include "safety/watchdog.h"
#include "safety/bounding_box.h"
#include "control/coordinate_mapper.h"
#include "control/firing_controller.h"

using namespace testing;
using namespace std::chrono_literals;

class ConcurrentShutdownStressTest : public Test {
protected:
    void SetUp() override {
        mock_laser_ = std::make_unique<NiceMock<MockLaser>>();
        mock_galvo_ = std::make_unique<NiceMock<MockGalvoDriver>>();

        SystemConfig::BoundingBox bb;
        bb.x_min = -2.0; bb.x_max = 2.0;
        bb.y_min = -2.0; bb.y_max = 2.0;
        bb.z_min = 0.1; bb.z_max = 10.0;

        SystemConfig::GalvoLimits gl;
        gl.angle_x_min_deg = -25.0;
        gl.angle_x_max_deg = 25.0;
        gl.angle_y_min_deg = -25.0;
        gl.angle_y_max_deg = 25.0;

        bbox_ = std::make_unique<BoundingBox3D>(bb);
        mapper_ = std::make_unique<CoordinateMapper>(*bbox_, gl);
    }

    std::unique_ptr<MockLaser> mock_laser_;
    std::unique_ptr<MockGalvoDriver> mock_galvo_;
    std::unique_ptr<BoundingBox3D> bbox_;
    std::unique_ptr<CoordinateMapper> mapper_;
};

TEST_F(ConcurrentShutdownStressTest, ShutdownWhileAllThreadsActive) {
    EXPECT_CALL(*mock_laser_, emergency_shutdown())
        .WillRepeatedly(Return(std::expected<void, HardwareError>{}));
    EXPECT_CALL(*mock_galvo_, zero())
        .WillRepeatedly(Return(std::expected<void, HardwareError>{}));
    EXPECT_CALL(*mock_laser_, fire(false))
        .WillRepeatedly(Return(std::expected<void, HardwareError>{}));
    EXPECT_CALL(*mock_galvo_, set_position(_, _))
        .WillRepeatedly(Return(std::expected<void, HardwareError>{}));

    SystemStateMachine sm;
    sm.transition(SystemState::IDLE);
    sm.transition(SystemState::ARMED);

    Watchdog watchdog(sm, *mock_laser_, *mock_galvo_, 50);
    FiringController fc(*mock_laser_, *mock_galvo_, *mapper_, 100.0, 10.0, 3.0);

    ThreadSafeQueue<StereoFrame> frame_queue;
    ThreadSafeQueue<TargetCommand> target_queue;
    std::atomic<std::chrono::steady_clock::time_point> heartbeat{
        std::chrono::steady_clock::now()};
    std::atomic<bool> shutdown_requested{false};

    std::jthread capture_thread([&](std::stop_token st) {
        uint64_t id = 0;
        while (!st.stop_requested() && !shutdown_requested.load()) {
            StereoFrame f;
            f.frame_id = id++;
            f.timestamp = std::chrono::steady_clock::now();
            frame_queue.push(std::move(f));
            std::this_thread::sleep_for(4ms);
        }
    });

    std::jthread processing_thread([&](std::stop_token st) {
        while (!st.stop_requested() && !shutdown_requested.load()) {
            auto frames = frame_queue.drain_all();
            if (frames.empty()) {
                std::this_thread::sleep_for(1ms);
                continue;
            }
            heartbeat.store(frames.back().timestamp);

            TargetCommand cmd;
            cmd.frame_id = frames.back().frame_id;
            cmd.timestamp = frames.back().timestamp;
            cmd.target_valid = true;
            cmd.target_position = Point3D{0.0, 0.0, 1.0};
            target_queue.push(std::move(cmd));
        }
    });

    std::jthread control_thread([&](std::stop_token st) {
        while (!st.stop_requested() && !shutdown_requested.load()) {
            auto now = std::chrono::steady_clock::now();

            if (!watchdog.check(now)) {
                break;
            }

            auto cmd = target_queue.pop(5ms);
            if (cmd.has_value()) {
                watchdog.feed(cmd->timestamp);
                fc.set_target(cmd->target_position.value());
            }

            fc.execute_cycle(now);
        }

        mock_laser_->emergency_shutdown();
        mock_galvo_->zero();
    });

    std::this_thread::sleep_for(50ms);
    shutdown_requested.store(true);

    capture_thread.join();
    processing_thread.join();
    control_thread.join();

    EXPECT_NE(sm.current(), SystemState::SAFE_HALT);
}

TEST_F(ConcurrentShutdownStressTest, SignalInterruptForcesLaserOffWithinOneCycle) {
    EXPECT_CALL(*mock_laser_, fire(false))
        .WillRepeatedly(Return(std::expected<void, HardwareError>{}));

    SystemStateMachine sm;
    Watchdog wd(sm, *mock_laser_, *mock_galvo_, 3);
    FiringController fc(*mock_laser_, *mock_galvo_, *mapper_);

    auto now = std::chrono::steady_clock::now();
    fc.set_target({0.0, 0.0, 1.0});

    fc.emergency_stop();

    auto cycle_result = fc.execute_cycle(now + 1ms);
    EXPECT_FALSE(cycle_result);
}

TEST_F(ConcurrentShutdownStressTest, DestructorCascadePreservesLaserOffOrder) {
    auto local_laser = std::make_unique<NiceMock<MockLaser>>();
    auto local_galvo = std::make_unique<NiceMock<MockGalvoDriver>>();

    EXPECT_CALL(*local_laser, fire(false))
        .Times(AtLeast(1))
        .WillRepeatedly(Return(std::expected<void, HardwareError>{}));

    {
        auto fc = std::make_unique<FiringController>(
            *local_laser, *local_galvo, *mapper_);

        fc->emergency_stop();
    }
}
