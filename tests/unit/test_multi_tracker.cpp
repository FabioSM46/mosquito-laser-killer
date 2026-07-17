#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "core/types.h"
#include "vision/multi_tracker.h"

using namespace testing;

namespace {

// 120 fps frame step.
constexpr auto k_frame = std::chrono::microseconds(8333);

class MultiTrackerTest : public Test {
protected:
    MultiTrackerTest()
        : tracker_(config_.tracking)
        , t0_(std::chrono::steady_clock::time_point{}) {}

    // A point flying along +x at 1.2 m/s: comfortably inside the [0.05, 3.0]
    // speed window once the filter has seen a few frames.
    auto flying_point(int k) -> Point3D {
        return Point3D{0.01 * static_cast<double>(k), 0.0, 0.75};
    }

    auto tick(int n) -> std::chrono::steady_clock::time_point {
        return t0_ + n * k_frame;
    }

    // Feeds n consecutive frames of the same moving target and returns the
    // last update's output.
    auto fly_frames(int n, int start_k = 0) -> std::vector<TrackedTarget> {
        std::vector<TrackedTarget> out;
        for (int k = start_k; k < start_k + n; ++k) {
            out = tracker_.update({flying_point(k)}, tick(k));
        }
        return out;
    }

    SystemConfig config_{};
    MultiTracker tracker_;
    std::chrono::steady_clock::time_point t0_;
};

TEST_F(MultiTrackerTest, SingleMeasurementIsTentativeNotEngageable) {
    const auto out = tracker_.update({flying_point(0)}, tick(0));

    ASSERT_EQ(out.size(), 1u);
    EXPECT_EQ(out[0].hits, 1);
    EXPECT_FALSE(out[0].confirmed);
    // A one-frame detection is a phantom candidate: it must never be aimed at.
    EXPECT_FALSE(out[0].engageable);
}

TEST_F(MultiTrackerTest, ConfirmedAndEngageableAfterEnoughConsecutiveHits) {
    ASSERT_EQ(config_.tracking.confirm_hits, 3);

    const auto out = fly_frames(3);

    ASSERT_EQ(out.size(), 1u);
    EXPECT_EQ(out[0].hits, 3);
    EXPECT_TRUE(out[0].confirmed);
    EXPECT_TRUE(out[0].engageable);

    // The velocity estimate converges over more frames (the filter seeds at
    // v=0), so check magnitude after a longer flight: 1 cm per 8.33 ms frame
    // is 1.2 m/s.
    const auto converged = fly_frames(30, 3);
    ASSERT_EQ(converged.size(), 1u);
    EXPECT_NEAR(converged[0].velocity.x, 1.2, 0.3);
}

TEST_F(MultiTrackerTest, StaticPointIsConfirmedButNeverEngageable) {
    // Perfectly trackable, perfectly stationary: a glint or a fixture. The
    // speed window is the only thing telling it apart from a flying insect.
    std::vector<TrackedTarget> out;
    for (int k = 0; k < 6; ++k) {
        out = tracker_.update({Point3D{0.02, 0.0, 0.75}}, tick(k));
    }

    ASSERT_EQ(out.size(), 1u);
    EXPECT_GE(out[0].hits, 3);
    EXPECT_TRUE(out[0].confirmed);
    EXPECT_NEAR(out[0].velocity.x, 0.0, 0.05);
    EXPECT_FALSE(out[0].engageable);
}

TEST_F(MultiTrackerTest, TrackIdIsStableAcrossFrames) {
    const auto first = fly_frames(1, 0);
    const auto later = fly_frames(3, 1);

    ASSERT_EQ(first.size(), 1u);
    ASSERT_EQ(later.size(), 1u);
    EXPECT_EQ(first[0].id, later[0].id);
}

TEST_F(MultiTrackerTest, CoastsThroughABriefDetectionGap) {
    // Fly long enough for the velocity estimate to converge, then drop a frame.
    const auto tracked = fly_frames(30);
    ASSERT_TRUE(tracked[0].engageable);
    const double last_x = tracked[0].position.x;

    // One empty frame: the track rides its prediction rather than dying.
    const auto coasted = tracker_.update({}, tick(30));
    ASSERT_EQ(coasted.size(), 1u);
    EXPECT_EQ(coasted[0].misses, 1);
    // The coasted position extrapolates along the velocity, it does not freeze.
    EXPECT_GT(coasted[0].position.x, last_x);

    // And the track recovers when the target is seen again.
    const auto recovered = tracker_.update({flying_point(31)}, tick(31));
    ASSERT_EQ(recovered.size(), 1u);
    EXPECT_EQ(recovered[0].misses, 0);
    EXPECT_EQ(recovered[0].id, coasted[0].id);
}

TEST_F(MultiTrackerTest, TrackDiesPastThePredictHorizon) {
    ASSERT_TRUE(fly_frames(3)[0].engageable);
    ASSERT_EQ(tracker_.track_count(), 1u);

    // 150 ms of silence >> k_max_predict_horizon_s (100 ms): extrapolating any
    // further would aim wherever the last velocity happened to point.
    const auto out = tracker_.update({}, tick(3) + std::chrono::milliseconds(150));
    EXPECT_THAT(out, IsEmpty());
    EXPECT_EQ(tracker_.track_count(), 0u);
}

TEST_F(MultiTrackerTest, TwoTargetsGetTwoStableTracks) {
    std::vector<TrackedTarget> out;
    for (int k = 0; k < 3; ++k) {
        out = tracker_.update(
            {Point3D{0.01 * k, 0.0, 0.75}, Point3D{-0.01 * k, 0.05, 0.6}},
            tick(k));
    }

    ASSERT_EQ(out.size(), 2u);
    EXPECT_NE(out[0].id, out[1].id);
    EXPECT_TRUE(out[0].engageable);
    EXPECT_TRUE(out[1].engageable);
}

// One measurement sitting between two tracks must bind to its genuine nearest
// track and only that one; the other track coasts. A measurement claimed by
// the wrong track would corrupt both velocity estimates.
TEST_F(MultiTrackerTest, AmbiguousMeasurementBindsToNearestTrackOnly) {
    // Two confirmed tracks 10 cm apart on x.
    for (int k = 0; k < 3; ++k) {
        const auto seeded = tracker_.update(
            {Point3D{0.0, 0.0, 0.75}, Point3D{0.10, 0.0, 0.75}}, tick(k));
        EXPECT_EQ(seeded.size(), 2u);
    }

    // A single measurement 2 cm from track A, 8 cm from track B (both well
    // inside the 0.15 m gate).
    const auto out = tracker_.update({Point3D{0.02, 0.0, 0.75}}, tick(3));

    ASSERT_EQ(out.size(), 2u);
    const TrackedTarget* near_track = nullptr;
    const TrackedTarget* far_track = nullptr;
    for (const auto& t : out) {
        if (t.position.x < 0.05) {
            near_track = &t;
        } else {
            far_track = &t;
        }
    }
    ASSERT_TRUE(near_track != nullptr);
    ASSERT_TRUE(far_track != nullptr);
    EXPECT_EQ(near_track->misses, 0) << "the nearer track must win the measurement";
    EXPECT_EQ(far_track->misses, 1) << "the farther track must coast, not steal it";
}

TEST_F(MultiTrackerTest, MaxTracksRefusesNewTracksButKeepsLiveOnes) {
    config_.tracking.max_tracks = 2;
    MultiTracker capped(config_.tracking);

    // Three far-apart targets for several frames: the cap must hold, and the
    // two tracks that exist must keep updating rather than being evicted.
    std::vector<TrackedTarget> out;
    for (int k = 0; k < 4; ++k) {
        out = capped.update(
            {Point3D{0.01 * k, 0.0, 0.75},
             Point3D{-0.01 * k, 0.0, 0.6},
             Point3D{0.01 * k, 0.08, 0.9}},
            tick(k));
        EXPECT_LE(out.size(), 2u);
    }
    EXPECT_EQ(capped.track_count(), 2u);
    // The live tracks still got their hits.
    for (const auto& t : out) {
        EXPECT_GE(t.hits, 3);
    }
}

TEST_F(MultiTrackerTest, NonFiniteMeasurementCreatesNoTrack) {
    constexpr double k_nan = std::numeric_limits<double>::quiet_NaN();

    EXPECT_THAT(tracker_.update({Point3D{k_nan, 0.0, 0.75}}, tick(0)), IsEmpty());
    EXPECT_EQ(tracker_.track_count(), 0u);

    // Mixed with a good measurement, the good one still tracks.
    const auto out = tracker_.update({Point3D{k_nan, 0.0, 0.75}, flying_point(0)},
                                     tick(1));
    EXPECT_EQ(out.size(), 1u);
}

TEST_F(MultiTrackerTest, ResetClearsAllTracks) {
    ASSERT_TRUE(fly_frames(3)[0].confirmed);
    tracker_.reset();
    EXPECT_EQ(tracker_.track_count(), 0u);

    // After a reset the world starts over: the same target is tentative again.
    const auto out = tracker_.update({flying_point(3)}, tick(3));
    ASSERT_EQ(out.size(), 1u);
    EXPECT_FALSE(out[0].confirmed);
}

}   // namespace
