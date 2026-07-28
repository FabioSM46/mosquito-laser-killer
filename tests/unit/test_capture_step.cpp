#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <algorithm>
#include <chrono>

#include "hal/capture_step.h"
#include "mocks/mock_camera.h"

using namespace testing;
using namespace std::chrono_literals;

namespace {

constexpr auto kOk = std::expected<void, HardwareError>{};
constexpr int k_width = 8;
constexpr int k_height = 4;
constexpr size_t k_bytes =
    static_cast<size_t>(k_width) * static_cast<size_t>(k_height);

// Writes a constant byte into the capture buffer, standing in for a frame.
auto fill_with(uint8_t value) {
    return [value](uint8_t* buffer,
                   size_t size) -> std::expected<void, HardwareError> {
        std::fill_n(buffer, size, value);
        return {};
    };
}

class CaptureStepTest : public Test {
protected:
    NiceMock<MockCamera> left_;
    NiceMock<MockCamera> right_;
    std::chrono::steady_clock::time_point t0_{
        std::chrono::steady_clock::time_point{} + 1000s};
};

TEST_F(CaptureStepTest, AssemblesAFrameWithPerCameraDataAndTimestamps) {
    const auto t_left = t0_ + 1ms;
    const auto t_right = t0_ + 3ms;
    // The buffer size handed to each camera must be exactly width*height —
    // the negotiate_format() contract the rest of the pipeline assumes.
    EXPECT_CALL(left_, capture(_, k_bytes)).WillOnce(Invoke(fill_with(0xAA)));
    EXPECT_CALL(right_, capture(_, k_bytes)).WillOnce(Invoke(fill_with(0xBB)));
    ON_CALL(left_, last_frame_timestamp()).WillByDefault(Return(t_left));
    ON_CALL(right_, last_frame_timestamp()).WillByDefault(Return(t_right));

    auto frame = capture_step(left_, right_, 42, k_width, k_height, t0_);

    ASSERT_TRUE(frame.has_value());
    EXPECT_EQ(frame->frame_id, 42u);
    EXPECT_EQ(frame->timestamp, t0_);
    EXPECT_EQ(frame->left_timestamp, t_left);
    EXPECT_EQ(frame->right_timestamp, t_right);
    ASSERT_EQ(frame->left_frame.size(), k_bytes);
    ASSERT_EQ(frame->right_frame.size(), k_bytes);
    // Each buffer must hold its OWN camera's pixels: a swap here corrupts
    // stereo disparity and aims the laser at wrong 3D positions (§10.8).
    EXPECT_TRUE(std::all_of(frame->left_frame.begin(), frame->left_frame.end(),
                            [](uint8_t b) { return b == 0xAA; }));
    EXPECT_TRUE(std::all_of(frame->right_frame.begin(), frame->right_frame.end(),
                            [](uint8_t b) { return b == 0xBB; }));
}

TEST_F(CaptureStepTest, GrabsLeftBeforeRight) {
    // Sequential order is part of the skew contract (§4.12): left's exposure
    // precedes right's, and the recorded timestamps must mean exactly that.
    InSequence seq;
    EXPECT_CALL(left_, capture(_, _)).WillOnce(Return(kOk));
    EXPECT_CALL(right_, capture(_, _)).WillOnce(Return(kOk));

    ASSERT_TRUE(
        capture_step(left_, right_, 0, k_width, k_height, t0_).has_value());
}

TEST_F(CaptureStepTest, LeftFailureShortCircuitsBeforeTheRightGrab) {
    EXPECT_CALL(left_, capture(_, _))
        .WillOnce(Return(std::unexpected(HardwareError::Timeout)));
    EXPECT_CALL(right_, capture(_, _)).Times(0);

    auto frame = capture_step(left_, right_, 0, k_width, k_height, t0_);

    ASSERT_FALSE(frame.has_value());
    EXPECT_EQ(frame.error(), HardwareError::Timeout);
}

TEST_F(CaptureStepTest, RightFailurePropagates) {
    EXPECT_CALL(left_, capture(_, _)).WillOnce(Return(kOk));
    EXPECT_CALL(right_, capture(_, _))
        .WillOnce(Return(std::unexpected(HardwareError::CameraCaptureFailed)));

    auto frame = capture_step(left_, right_, 0, k_width, k_height, t0_);

    ASSERT_FALSE(frame.has_value());
    EXPECT_EQ(frame.error(), HardwareError::CameraCaptureFailed);
}

}   // namespace
