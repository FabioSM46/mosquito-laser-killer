#include <gtest/gtest.h>
#include <csignal>
#include <atomic>

#include "safety/signal_handler.h"

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

TEST(SignalHandlerTest, SignalDoesNotInvokeCallbackButSetsFlag) {
    // Async-signal-safe path must only set an atomic; main threads poll
    // is_shutdown_requested() (see main.cpp worker loops).
    SignalHandler sh;
    std::atomic<bool> callback_fired{false};
    sh.set_shutdown_callback([&] { callback_fired.store(true); });
    sh.install();
    sh.reset();

    std::raise(SIGINT);

    EXPECT_TRUE(sh.is_shutdown_requested());
    EXPECT_FALSE(callback_fired.load());
}

// The former SignalThenLaserEmergencyShutdownForcesPinLow and
// LaserDestructorForcesPinLow cases lived here. Both were removed: the first
// raised a signal and then called emergency_shutdown() BY HAND — no causal
// link between the two, so it proved nothing about shutdown-on-signal — and
// the second only echoed the mock expectations it had just installed. The
// real properties are covered end-to-end, on an observed pin, by
// tests/stress/test_concurrent_shutdown.cpp (signal → loop exit → pin LOW,
// and destructor → pin LOW).
