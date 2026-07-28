#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <chrono>
#include <cstdint>
#include <memory>
#include <vector>

#include "core/types.h"
#include "vision/processing_step.h"

using namespace testing;
using namespace std::chrono_literals;

// End-to-end pipeline tests over the REAL vision components — the extracted
// processing_step() is exactly what the processing thread runs per frame, so
// these exercise detect → match_all → track → select as one unit, with the
// motion gate ON (the shipped config, unlike the types.h default of 0).
namespace {

constexpr int k_width = 640;
constexpr int k_height = 400;
constexpr size_t k_frame_size =
    static_cast<size_t>(k_width) * static_cast<size_t>(k_height);
constexpr auto k_frame_period = std::chrono::microseconds(8333);

// Production stereo geometry: f = 500 px, b = 0.12 m. Disparity 86 px puts
// the target at z = 60/86 ≈ 0.698 m — mid-box.
constexpr double k_disparity = 86.0;

void paint(std::vector<uint8_t>& image, int x0, int y0, int w, int h) {
    for (int y = y0; y < y0 + h; ++y) {
        for (int x = x0; x < x0 + w; ++x) {
            image[static_cast<size_t>(y) * static_cast<size_t>(k_width) +
                  static_cast<size_t>(x)] = 255;
        }
    }
}

class ProcessingStepTest : public Test {
protected:
    ProcessingStepTest() {
        // Shipped-config behaviour: motion gate ON (yaml ships 0.05; the
        // types.h default of 0 would test the legacy bright-blob path).
        config_.detection.background_learning_rate = 0.05;
        detector_left_ = std::make_unique<Detector>(k_width, k_height,
                                                    config_.detection);
        detector_right_ = std::make_unique<Detector>(k_width, k_height,
                                                     config_.detection);
        matcher_ = std::make_unique<StereoMatcher>(
            config_.stereo, config_.detection, config_.bounding_box);
        tracker_ = std::make_unique<MultiTracker>(config_.tracking);
        selector_ = std::make_unique<TargetSelector>();
        deps_ = std::make_unique<ProcessingDeps>(ProcessingDeps{
            *detector_left_, *detector_right_, *matcher_, *tracker_,
            *selector_});
    }

    auto tick(int k) -> std::chrono::steady_clock::time_point {
        return t0_ + k * k_frame_period;
    }

    // A dark stereo frame (seeds the background models on first use).
    auto dark_frame(uint64_t id, std::chrono::steady_clock::time_point t)
        -> StereoFrame {
        StereoFrame frame;
        frame.frame_id = id;
        frame.timestamp = t;
        frame.left_timestamp = t;
        frame.right_timestamp = t;
        frame.left_frame.assign(k_frame_size, 0);
        frame.right_frame.assign(k_frame_size, 0);
        return frame;
    }

    // One 4x4 target at (u_left, v) in the left eye and (u_left - disparity,
    // v) in the right eye. 16 px² sits inside the depth-size band at z ≈ 0.7.
    auto target_frame(uint64_t id, std::chrono::steady_clock::time_point t,
                      int u_left, int v) -> StereoFrame {
        auto frame = dark_frame(id, t);
        paint(frame.left_frame, u_left, v, 4, 4);
        paint(frame.right_frame, u_left - static_cast<int>(k_disparity), v, 4, 4);
        return frame;
    }

    SystemConfig config_{};
    std::chrono::steady_clock::time_point t0_{
        std::chrono::steady_clock::time_point{} + 1000s};
    std::unique_ptr<Detector> detector_left_;
    std::unique_ptr<Detector> detector_right_;
    std::unique_ptr<StereoMatcher> matcher_;
    std::unique_ptr<MultiTracker> tracker_;
    std::unique_ptr<TargetSelector> selector_;
    std::unique_ptr<ProcessingDeps> deps_;
};

TEST_F(ProcessingStepTest, FlyingTargetConfirmsThenCommandsWithSanePosition) {
    // Frame 0: dark — seeds both background models, no target possible.
    auto seed_cmd = processing_step(*deps_, dark_frame(0, tick(0)));
    EXPECT_FALSE(seed_cmd.target_valid);

    // A target moving 8 px/frame (≈ 11 mm/frame at z ≈ 0.7 → ≈ 1.3 m/s,
    // inside the engageable speed window). Confirmation needs 3 consecutive
    // matched frames, so the first valid command must appear on the 3rd
    // detection and not before.
    int first_valid_at = -1;
    TargetCommand last;
    for (int k = 1; k <= 8; ++k) {
        auto frame = target_frame(static_cast<uint64_t>(k), tick(k),
                                  300 + 8 * k, 200);
        last = processing_step(*deps_, frame);
        EXPECT_EQ(last.frame_id, static_cast<uint64_t>(k));
        EXPECT_EQ(last.timestamp, tick(k));
        if (last.target_valid && first_valid_at < 0) {
            first_valid_at = k;
        }
    }

    EXPECT_EQ(first_valid_at, 3)
        << "one-frame phantoms must not be commanded; confirmed tracks must be";
    ASSERT_TRUE(last.target_valid);
    ASSERT_TRUE(last.target_position.has_value());
    // z from the painted disparity; x back-projected from the last left-eye
    // position (u = 364 → x ≈ (364+1.5-320)·z/500). Loose tolerances: the
    // Kalman output smooths, it does not reproduce the geometry exactly.
    EXPECT_NEAR(last.target_position->z, 0.698, 0.03);
    EXPECT_NEAR(last.target_position->x, (364.0 + 1.5 - 320.0) * 0.698 / 500.0,
                0.02);
    EXPECT_NEAR(last.target_position->y, 0.0, 0.02);
}

TEST_F(ProcessingStepTest, AmbiguousSceneProducesNoTargetCommand) {
    ASSERT_FALSE(processing_step(*deps_, dark_frame(0, tick(0))).target_valid);

    // Two blobs per eye ON THE SAME ROW, spaced so DIRECT and CROSS pairings
    // all satisfy the epipolar, disparity-window, area-ratio and depth-size
    // gates: every blob participates in more than one candidate, so every
    // pairing is void and the frame must yield silence, not a guess (§4.12).
    //   left u = {300, 320}, right u = {214, 234}, all at v = 200
    //   direct d = 86 (z≈0.70) ✓ · cross d = 66 (z≈0.91) and 106 (z≈0.57) ✓
    //   (16 px² sits inside the size band at all three depths)
    for (int k = 1; k <= 4; ++k) {
        auto frame = dark_frame(static_cast<uint64_t>(k), tick(k));
        const int shift = 8 * k;   // keep everything moving past the motion gate
        paint(frame.left_frame, 300 + shift, 200, 4, 4);
        paint(frame.left_frame, 320 + shift, 200, 4, 4);
        paint(frame.right_frame, 214 + shift, 200, 4, 4);
        paint(frame.right_frame, 234 + shift, 200, 4, 4);

        auto cmd = processing_step(*deps_, frame);
        EXPECT_FALSE(cmd.target_valid) << "ambiguous frame " << k
                                       << " produced a target";
    }
}

TEST_F(ProcessingStepTest, EmptyFramesYieldExplicitNoTargetCommands) {
    // The control thread relies on receiving an explicit target_valid=false
    // to clear a stale aim — an empty frame must still produce its command.
    for (int k = 0; k < 3; ++k) {
        auto cmd = processing_step(*deps_, dark_frame(static_cast<uint64_t>(k),
                                                      tick(k)));
        EXPECT_FALSE(cmd.target_valid);
        EXPECT_FALSE(cmd.target_position.has_value());
        EXPECT_EQ(cmd.frame_id, static_cast<uint64_t>(k));
    }
}

}   // namespace
