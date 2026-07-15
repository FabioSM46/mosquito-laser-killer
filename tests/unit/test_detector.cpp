#include <gtest/gtest.h>
#include <array>
#include <cstdint>
#include <memory>

#include "vision/detector.h"

using namespace testing;

namespace {
constexpr size_t kFrameSize = 640 * 480;

auto make_dark_frame() -> std::array<uint8_t, kFrameSize> {
    std::array<uint8_t, kFrameSize> frame{};
    return frame;
}

auto make_frame_with_blob(int x0, int y0, int w, int h, uint8_t value = 255)
    -> std::array<uint8_t, kFrameSize> {
    auto frame = make_dark_frame();
    for (int y = y0; y < y0 + h; ++y) {
        for (int x = x0; x < x0 + w; ++x) {
            frame.at(static_cast<size_t>(y) * 640 + x) = value;
        }
    }
    return frame;
}
}

class DetectorTest : public Test {
protected:
    void SetUp() override {
        detector_ = std::make_unique<Detector>();
    }

    std::unique_ptr<Detector> detector_;
};

TEST_F(DetectorTest, DarkFrameReturnsNullopt) {
    auto frame = make_dark_frame();
    EXPECT_FALSE(detector_->detect(frame).has_value());
}

TEST_F(DetectorTest, BlobBelowMinAreaReturnsNullopt) {
    auto frame = make_frame_with_blob(100, 100, 3, 3);
    EXPECT_FALSE(detector_->detect(frame).has_value());
}

TEST_F(DetectorTest, BlobAboveMinAreaReturnsCentroid) {
    constexpr int x0 = 320;
    constexpr int y0 = 240;
    constexpr int w = 10;
    constexpr int h = 10;

    auto frame = make_frame_with_blob(x0, y0, w, h);
    auto result = detector_->detect(frame);

    ASSERT_TRUE(result.has_value());

    const double expected_u = x0 + (w - 1) / 2.0;
    const double expected_v = y0 + (h - 1) / 2.0;
    EXPECT_NEAR(result->u, expected_u, 0.5);
    EXPECT_NEAR(result->v, expected_v, 0.5);
}

TEST_F(DetectorTest, ThresholdBoundaryExcludesEqualPixels) {
    detector_->set_threshold(128);
    auto frame = make_frame_with_blob(200, 200, 10, 10, 128);

    EXPECT_FALSE(detector_->detect(frame).has_value());

    auto bright_frame = make_frame_with_blob(200, 200, 10, 10, 129);
    EXPECT_TRUE(detector_->detect(bright_frame).has_value());
}

TEST_F(DetectorTest, SetThresholdChangesDetection) {
    detector_->set_threshold(200);

    auto frame = make_frame_with_blob(100, 100, 10, 10, 150);
    EXPECT_FALSE(detector_->detect(frame).has_value());

    auto bright = make_frame_with_blob(100, 100, 10, 10, 210);
    EXPECT_TRUE(detector_->detect(bright).has_value());
}
