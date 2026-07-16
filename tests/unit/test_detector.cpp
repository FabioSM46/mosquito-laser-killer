#include <gtest/gtest.h>
#include <vector>
#include <cstdint>
#include <memory>

#include "vision/detector.h"

using namespace testing;

namespace {
constexpr int kWidth = 640;
constexpr int kHeight = 400;
constexpr size_t kFrameSize = static_cast<size_t>(kWidth) * kHeight;

auto make_dark_frame() -> std::vector<uint8_t> {
    return std::vector<uint8_t>(kFrameSize, 0);
}

auto make_frame_with_blob(int x0, int y0, int w, int h, uint8_t value = 255)
    -> std::vector<uint8_t> {
    auto frame = make_dark_frame();
    for (int y = y0; y < y0 + h; ++y) {
        for (int x = x0; x < x0 + w; ++x) {
            frame[static_cast<size_t>(y) * kWidth + x] = value;
        }
    }
    return frame;
}
}

class DetectorTest : public Test {
protected:
    void SetUp() override {
        detector_ = std::make_unique<Detector>(kWidth, kHeight);
    }

    std::unique_ptr<Detector> detector_;
};

TEST_F(DetectorTest, DarkFrameReturnsNullopt) {
    auto frame = make_dark_frame();
    EXPECT_FALSE(detector_->detect(frame.data(), frame.size()).has_value());
}

TEST_F(DetectorTest, BlobBelowMinAreaReturnsNullopt) {
    auto frame = make_frame_with_blob(100, 100, 3, 3);
    EXPECT_FALSE(detector_->detect(frame.data(), frame.size()).has_value());
}

TEST_F(DetectorTest, BlobAboveMinAreaReturnsCentroid) {
    constexpr int x0 = 320;
    constexpr int y0 = 200;
    constexpr int w = 10;
    constexpr int h = 10;

    auto frame = make_frame_with_blob(x0, y0, w, h);
    auto result = detector_->detect(frame.data(), frame.size());

    ASSERT_TRUE(result.has_value());

    const double expected_u = x0 + (w - 1) / 2.0;
    const double expected_v = y0 + (h - 1) / 2.0;
    EXPECT_NEAR(result->u, expected_u, 0.5);
    EXPECT_NEAR(result->v, expected_v, 0.5);
}

TEST_F(DetectorTest, ThresholdBoundaryExcludesEqualPixels) {
    detector_->set_threshold(128);
    auto frame = make_frame_with_blob(200, 200, 10, 10, 128);

    EXPECT_FALSE(detector_->detect(frame.data(), frame.size()).has_value());

    auto bright_frame = make_frame_with_blob(200, 200, 10, 10, 129);
    EXPECT_TRUE(detector_->detect(bright_frame.data(), bright_frame.size()).has_value());
}

TEST_F(DetectorTest, SetThresholdChangesDetection) {
    detector_->set_threshold(200);

    auto frame = make_frame_with_blob(100, 100, 10, 10, 150);
    EXPECT_FALSE(detector_->detect(frame.data(), frame.size()).has_value());

    auto bright = make_frame_with_blob(100, 100, 10, 10, 210);
    EXPECT_TRUE(detector_->detect(bright.data(), bright.size()).has_value());
}

TEST_F(DetectorTest, CustomResolutionWorks) {
    constexpr int w = 320;
    constexpr int h = 240;
    Detector small(w, h);

    std::vector<uint8_t> frame(static_cast<size_t>(w) * h, 0);
    for (int y = 100; y < 110; ++y) {
        for (int x = 150; x < 160; ++x) {
            frame[static_cast<size_t>(y) * w + x] = 255;
        }
    }

    auto result = small.detect(frame.data(), frame.size());
    ASSERT_TRUE(result.has_value());
    EXPECT_NEAR(result->u, 154.5, 0.5);
    EXPECT_NEAR(result->v, 104.5, 0.5);
}

TEST_F(DetectorTest, NullDataReturnsNullopt) {
    EXPECT_FALSE(detector_->detect(nullptr, kFrameSize).has_value());
}

TEST_F(DetectorTest, EmptySizeReturnsNullopt) {
    uint8_t dummy = 0;
    EXPECT_FALSE(detector_->detect(&dummy, 0).has_value());
}
