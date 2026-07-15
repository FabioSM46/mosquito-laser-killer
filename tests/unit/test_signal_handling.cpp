#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <csignal>
#include <atomic>
#include <memory>

#include "mocks/mock_gpio.h"
#include "hal/laser.h"
#include "safety/signal_handler.h"
#include "core/error.h"

using namespace testing;

namespace {
constexpr auto kOk = std::expected<void, HardwareError>{};
}

TEST(SignalHandlerTest, SigIntSetsShutdownFlag) {
    SignalHandler sh;
    sh.install();
    sh.reset();

    std::raise(SIGINT);

    EXPECT_TRUE(sh.is_shutdown_requested());
}

TEST(SignalHandlerTest, SigTermSetsShutdownFlag) {
    SignalHandler sh;
    sh.install();
    sh.reset();

    std::raise(SIGTERM);

    EXPECT_TRUE(sh.is_shutdown_requested());
}

TEST(SignalHandlerTest, ResetClearsFlag) {
    SignalHandler sh;
    sh.install();
    sh.reset();

    std::raise(SIGINT);
    ASSERT_TRUE(sh.is_shutdown_requested());

    sh.reset();
    EXPECT_FALSE(sh.is_shutdown_requested());
}

TEST(SignalHandlerTest, ProgrammaticRequestInvokesCallback) {
    SignalHandler sh;

    std::atomic<bool> callback_fired{false};
    sh.set_shutdown_callback([&] { callback_fired.store(true); });

    sh.request_shutdown();

    EXPECT_TRUE(sh.is_shutdown_requested());
    EXPECT_TRUE(callback_fired.load());
}

TEST(SignalHandlerTest, SignalThenLaserEmergencyShutdownForcesPinLow) {
    auto mock_gpio = std::make_unique<StrictMock<MockGpio>>();

    EXPECT_CALL(*mock_gpio, set_direction_output()).WillOnce(Return(kOk));
    EXPECT_CALL(*mock_gpio, write(false))
        .Times(AtLeast(2))
        .WillRepeatedly(Return(kOk));

    auto laser = std::make_unique<Laser>(std::move(mock_gpio), 18);

    SignalHandler sh;
    sh.install();
    sh.reset();

    std::raise(SIGINT);
    ASSERT_TRUE(sh.is_shutdown_requested());

    auto result = laser->emergency_shutdown();
    EXPECT_TRUE(result.has_value());
}

TEST(SignalHandlerTest, LaserDestructorForcesPinLow) {
    auto mock_gpio = std::make_unique<StrictMock<MockGpio>>();

    EXPECT_CALL(*mock_gpio, set_direction_output()).WillOnce(Return(kOk));
    EXPECT_CALL(*mock_gpio, write(false))
        .Times(AtLeast(2))
        .WillRepeatedly(Return(kOk));

    auto laser = std::make_unique<Laser>(std::move(mock_gpio), 18);
    laser.reset();
}
