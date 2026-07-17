#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <limits>
#include <memory>
#include <optional>
#include <vector>

#include "core/types.h"
#include "vision/stereo_matcher.h"

using namespace testing;

namespace {

// The shipped defaults: f = 500 px, b = 0.12 m, so f*b = 60 and z = 60 / d.
// The disparity window that implies is [f*b/z_max, f*b/z_min] = [60, 120] px,
// i.e. exactly the z = [1.0, 0.5] m engagement volume and nothing else.
constexpr double k_cx = 320.0;
constexpr double k_cy = 200.0;

auto make_blob(double u, double v, int area) -> Blob {
    Blob blob;
    blob.centroid = Pixel2D{u, v};
    blob.area_px = area;
    blob.width_px = 10;
    blob.height_px = 10;
    return blob;
}

class StereoMatcherTest : public Test {
protected:
    void SetUp() override {
        matcher_ = std::make_unique<StereoMatcher>(config_.stereo, config_.detection,
                                                   config_.bounding_box);
    }

    SystemConfig config_{};
    std::unique_ptr<StereoMatcher> matcher_;
};

TEST_F(StereoMatcherTest, DisparityWindowMatchesConfiguredDepthRange) {
    // The window is not a tuning constant; it is the z limits restated in pixels.
    // If these drift apart, the matcher is admitting depths the bounding box was
    // written to exclude.
    EXPECT_DOUBLE_EQ(matcher_->min_disparity_px(),
                     config_.stereo.focal_length_px * config_.stereo.baseline_m /
                         config_.bounding_box.z_max);
    EXPECT_DOUBLE_EQ(matcher_->max_disparity_px(),
                     config_.stereo.focal_length_px * config_.stereo.baseline_m /
                         config_.bounding_box.z_min);

    EXPECT_DOUBLE_EQ(matcher_->min_disparity_px(), 60.0);
    EXPECT_DOUBLE_EQ(matcher_->max_disparity_px(), 120.0);
}

TEST_F(StereoMatcherTest, TriangulateKnownDisparitiesGiveKnownDepths) {
    // z = f*b/d = 60/d, checked at both ends of the window and in the middle.
    const auto far_point = matcher_->triangulate({320.0, 200.0}, {260.0, 200.0});
    ASSERT_TRUE(far_point.has_value());
    EXPECT_NEAR(far_point->z, 1.0, 1e-9);   // d = 60

    const auto near_point = matcher_->triangulate({320.0, 200.0}, {200.0, 200.0});
    ASSERT_TRUE(near_point.has_value());
    EXPECT_NEAR(near_point->z, 0.5, 1e-9);   // d = 120

    const auto mid_point = matcher_->triangulate({320.0, 200.0}, {234.3, 200.0});
    ASSERT_TRUE(mid_point.has_value());
    EXPECT_NEAR(mid_point->z, 0.7, 1e-3);   // d = 85.7
}

TEST_F(StereoMatcherTest, TriangulateProjectsXAndYFromTheLeftPixel) {
    // x = (u_L - cx) * z / f and y = (v_L - cy) * z / f.
    const auto point = matcher_->triangulate({360.0, 230.0}, {280.0, 230.0});
    ASSERT_TRUE(point.has_value());

    const double z = 60.0 / 80.0;   // d = 80
    EXPECT_NEAR(point->z, z, 1e-9);
    EXPECT_NEAR(point->x, (360.0 - k_cx) * z / 500.0, 1e-9);   // +0.060 m
    EXPECT_NEAR(point->y, (230.0 - k_cy) * z / 500.0, 1e-9);   // +0.045 m
}

// The epipolar gate is the only evidence the system ever gets that the two blobs
// it is about to triangulate are the same physical object.
//
// The old triangulate() took a right_point and never read right_point.v — it
// used the u coordinates for disparity, the left v for y, and dropped the right
// v on the floor. So any left blob would pair with any right blob at any height,
// and the old suite enshrined this: `VerticalDisparityIsIgnored` fed a 20 px
// vertical offset and asserted the result was ACCEPTED. On a rectified pair a
// 20 px vertical offset is proof the two blobs are different objects, and
// triangulating across them produces a disparity — and therefore a z — computed
// from two unrelated things.
TEST_F(StereoMatcherTest, EpipolarMismatchIsRejected) {
    // 1 px of vertical offset is rectification noise; the same object, accepted.
    EXPECT_TRUE(matcher_->triangulate({320.0, 200.0}, {260.0, 201.0}).has_value());

    // 2 px is the configured tolerance and the gate is `>`, so it still passes.
    ASSERT_DOUBLE_EQ(config_.detection.epipolar_tolerance_px, 2.0);
    EXPECT_TRUE(matcher_->triangulate({320.0, 200.0}, {260.0, 202.0}).has_value());

    // 3 px is past the tolerance: these are two different objects and there is no
    // depth to be had from pairing them.
    EXPECT_FALSE(matcher_->triangulate({320.0, 200.0}, {260.0, 203.0}).has_value());

    // The exact pair the old suite asserted was fine.
    EXPECT_FALSE(matcher_->triangulate({330.0, 250.0}, {270.0, 230.0}).has_value());
}

// A NaN tolerance makes `std::abs(dv) > tol` false for every pair — the
// epipolar gate passes everything, i.e. fails OPEN. The constructor must
// sanitize an invalid tolerance to the strictest gate rather than trust the
// config blindly (config_validator catches NaN today, but triangulate's own
// inputs are already sanitized at this same boundary).
TEST_F(StereoMatcherTest, NanEpipolarToleranceFailsClosed) {
    auto config = config_;
    config.detection.epipolar_tolerance_px = std::numeric_limits<double>::quiet_NaN();
    StereoMatcher matcher(config.stereo, config.detection, config.bounding_box);

    // Even a 1 px vertical offset — normally well inside tolerance — is rejected.
    EXPECT_FALSE(matcher.triangulate({320.0, 200.0}, {260.0, 201.0}).has_value());
    // Only an exactly-aligned pair survives the strictest gate.
    EXPECT_TRUE(matcher.triangulate({320.0, 200.0}, {260.0, 200.0}).has_value());
}

TEST_F(StereoMatcherTest, NonPositiveEpipolarToleranceFailsClosed) {
    auto config = config_;
    config.detection.epipolar_tolerance_px = -1.0;
    StereoMatcher matcher(config.stereo, config.detection, config.bounding_box);

    EXPECT_FALSE(matcher.triangulate({320.0, 200.0}, {260.0, 201.0}).has_value());
    EXPECT_TRUE(matcher.triangulate({320.0, 200.0}, {260.0, 200.0}).has_value());
}

TEST_F(StereoMatcherTest, DisparityOutsideTheWindowIsRejected) {
    // Below the floor -> the point is further than z_max = 1.0 m.
    EXPECT_FALSE(matcher_->triangulate({320.0, 200.0}, {265.0, 200.0}).has_value());

    // Above the ceiling -> the point is nearer than z_min = 0.5 m.
    EXPECT_FALSE(matcher_->triangulate({320.0, 200.0}, {195.0, 200.0}).has_value());

    // Both bounds are inclusive, and the interior passes.
    EXPECT_TRUE(matcher_->triangulate({320.0, 200.0}, {260.0, 200.0}).has_value());
    EXPECT_TRUE(matcher_->triangulate({320.0, 200.0}, {200.0, 200.0}).has_value());
    EXPECT_TRUE(matcher_->triangulate({320.0, 200.0}, {240.0, 200.0}).has_value());
}

// The single most dangerous input this class can be handed.
//
// The old code rejected only `std::abs(dx) < 0.5`: a guard against dividing by
// something near zero, masquerading as a safety check. Everything from 0.5 px
// upwards was admitted, and 0.5 px is what two blobs produce when they are not
// the same object at all, or when a correspondence is off by a pixel of noise.
// It triangulates to z = 60/0.5 = 120 m. Nothing 120 m away is in the room, let
// alone in the engagement volume — the number is not a measurement, it is a
// division artefact. The old pipeline had no way to say so, because by the time
// the bounding box saw that z, the triangulator had already vouched for it.
//
// Deriving the window from z_min/z_max means a disparity implying an impossible
// depth is now rejected where it is produced, rather than being passed on as a
// number for something further downstream to be fooled by.
TEST_F(StereoMatcherTest, TinyDisparityImplyingAbsurdDepthIsRejected) {
    // The exact value the old floor let through, and the one below it.
    EXPECT_FALSE(matcher_->triangulate({320.0, 200.0}, {319.5, 200.0}).has_value())
        << "d = 0.5 px triangulates to z = 120 m; it must not reach the caller";
    EXPECT_FALSE(matcher_->triangulate({320.0, 200.0}, {319.6, 200.0}).has_value())
        << "d = 0.4 px triangulates to z = 150 m";

    // Well clear of the old floor, still nowhere near the room: the point is that
    // the window is now the depth range, not an epsilon around zero.
    EXPECT_FALSE(matcher_->triangulate({320.0, 200.0}, {318.0, 200.0}).has_value())
        << "d = 2 px triangulates to z = 30 m";
    EXPECT_FALSE(matcher_->triangulate({320.0, 200.0}, {310.0, 200.0}).has_value())
        << "d = 10 px triangulates to z = 6 m";
}

TEST_F(StereoMatcherTest, ZeroAndNegativeDisparityRejected) {
    // d = 0: the same pixel in both frames, which is a point at infinity.
    EXPECT_FALSE(matcher_->triangulate({320.0, 200.0}, {320.0, 200.0}).has_value());

    // d < 0: the right camera sees the object further right than the left camera
    // does, which the geometry forbids. It means the correspondence is inverted.
    EXPECT_FALSE(matcher_->triangulate({300.0, 200.0}, {320.0, 200.0}).has_value());
    EXPECT_FALSE(matcher_->triangulate({200.0, 200.0}, {320.0, 200.0}).has_value());
}

TEST_F(StereoMatcherTest, NonFinitePixelIsRejected) {
    constexpr double k_nan = std::numeric_limits<double>::quiet_NaN();
    constexpr double k_inf = std::numeric_limits<double>::infinity();

    // right.v is the case that most needs the explicit finite check, and the one
    // that shows why it cannot be left to the arithmetic to sort out. A NaN there
    // walks straight through the epipolar gate (every comparison against NaN is
    // false, so the |v_L - v_R| test cannot reject it), contributes nothing to
    // the disparity, which comes from the u coordinates, and never reaches y,
    // which is computed from left.v. Every value the function returns would be
    // finite and entirely plausible — built from a pixel that does not exist.
    for (const double bad : {k_nan, k_inf, -k_inf}) {
        SCOPED_TRACE(Message() << "bad value: " << bad);

        EXPECT_FALSE(matcher_->triangulate({bad, 200.0}, {260.0, 200.0}).has_value())
            << "left.u";
        EXPECT_FALSE(matcher_->triangulate({320.0, bad}, {260.0, 200.0}).has_value())
            << "left.v";
        EXPECT_FALSE(matcher_->triangulate({320.0, 200.0}, {bad, 200.0}).has_value())
            << "right.u";
        EXPECT_FALSE(matcher_->triangulate({320.0, 200.0}, {260.0, bad}).has_value())
            << "right.v";
    }
}

TEST_F(StereoMatcherTest, MatchSinglePlausiblePairTriangulates) {
    // Area 9 px: a 5 mm target at z = 0.75 m projects to ~8.7 px, so this is
    // what a real mosquito-sized pair looks like to the size gate.
    const std::vector<Blob> left{make_blob(300.0, 200.0, 9)};
    const std::vector<Blob> right{make_blob(220.0, 200.0, 9)};

    const auto point = matcher_->match(left, right);

    ASSERT_TRUE(point.has_value());
    EXPECT_NEAR(point->z, 0.75, 1e-9);    // d = 80
    EXPECT_NEAR(point->x, -0.03, 1e-9);   // (300 - 320) * 0.75 / 500
    EXPECT_NEAR(point->y, 0.0, 1e-9);
}

TEST_F(StereoMatcherTest, MatchWithNoBlobsOnEitherSideReturnsNullopt) {
    const std::vector<Blob> some{make_blob(300.0, 200.0, 100)};
    const std::vector<Blob> none{};

    EXPECT_FALSE(matcher_->match(none, some).has_value());
    EXPECT_FALSE(matcher_->match(some, none).has_value());
    EXPECT_FALSE(matcher_->match(none, none).has_value());
}

// The safety property the whole class exists to provide.
//
// Aim angle here is atan2(x, z) with x = (u_L - cx) * z / f, so z cancels and
// the direction the galvos point depends only on the left pixel. z's only job is
// to answer the one question that matters: is this thing inside the engagement
// volume? That makes a fabricated z uniquely dangerous — it does not make the
// beam miss, it makes the beam fire. Pair the left blob with the wrong right
// blob and the disparity, and therefore z, is invented out of two unrelated
// objects. The aim is still confidently pointed at whatever the left camera saw,
// and the invented z is the only thing standing between "mosquito at 0.7 m" and
// "someone's face across the room". So when two pairings both survive every gate,
// there is no evidence for either: return nothing rather than pick a winner.
TEST_F(StereoMatcherTest, MatchAmbiguousSceneFailsClosed) {
    // Two blobs per camera, arranged so that L1-R1 (d = 80) and L2-R2 (d = 80)
    // are both perfectly plausible: same rows, same areas, both disparities
    // inside the window. match_all now resolves both as separate targets — but
    // the single-target match() contract stays fail closed: anything other
    // than exactly one survivor is no target at all.
    const std::vector<Blob> left{make_blob(300.0, 200.0, 9),
                                 make_blob(400.0, 200.0, 9)};
    const std::vector<Blob> right{make_blob(220.0, 200.0, 9),
                                  make_blob(320.0, 200.0, 9)};

    EXPECT_FALSE(matcher_->match(left, right).has_value());

    // Each half of that scene resolves on its own, which is what makes the
    // rejection above a statement about ambiguity rather than about the gates.
    EXPECT_TRUE(matcher_->match({left[0]}, {right[0]}).has_value());
    EXPECT_TRUE(matcher_->match({left[1]}, {right[1]}).has_value());
}

TEST_F(StereoMatcherTest, MatchRejectsPairWithMismatchedAreas) {
    // Same row, disparity 80 px, dead centre of the window: epipolar and
    // disparity both say yes. The areas say no. The same object at one distance
    // projects to roughly the same area in both cameras, so a ~9:1 area ratio
    // means these are two different objects that happen to line up — a mosquito
    // in one frame and something much larger behind it in the other.
    //
    // Areas 3 and 26 both sit INSIDE the depth-size band ([2.9, 26.2] px for a
    // 5 mm target at z = 0.75 m), so only the area-ratio gate can be rejecting
    // these pairs: if that gate were deleted, this test would fail.
    EXPECT_FALSE(matcher_->match({make_blob(300.0, 200.0, 3)},
                                 {make_blob(220.0, 200.0, 26)})
                     .has_value());

    // Rejected in both directions: the ratio bound is two-sided.
    EXPECT_FALSE(matcher_->match({make_blob(300.0, 200.0, 26)},
                                 {make_blob(220.0, 200.0, 3)})
                     .has_value());

    // Areas that differ by a plausible amount still pair, so the gate is
    // discriminating rather than simply refusing everything.
    EXPECT_TRUE(matcher_->match({make_blob(300.0, 200.0, 9)},
                                {make_blob(220.0, 200.0, 11)})
                    .has_value());
}

// The depth-consistent size gate: at a triangulated z, a target_size_m object
// projects to a known pixel area. Blobs far outside that band are glints,
// fixtures, or different animals — not the target.
TEST_F(StereoMatcherTest, SizeGateRejectsOversizedPair) {
    // d = 80 -> z = 0.75 m -> a 5 mm target is ~8.7 px. 100 px at that depth is
    // a ~19 mm object: a fixture or a glint, not a mosquito. Both sides are
    // 100 px, so the area-ratio gate passes and only the size gate can reject.
    const std::vector<Blob> left{make_blob(300.0, 200.0, 100)};
    const std::vector<Blob> right{make_blob(220.0, 200.0, 100)};

    EXPECT_THAT(matcher_->match_all(left, right), IsEmpty());
    EXPECT_FALSE(matcher_->match(left, right).has_value());
}

TEST_F(StereoMatcherTest, SizeGateRejectsUndersizedPair) {
    // 1 px at z = 0.75 m is a sub-mm speck: sensor noise, not a target.
    const std::vector<Blob> left{make_blob(300.0, 200.0, 1)};
    const std::vector<Blob> right{make_blob(220.0, 200.0, 1)};

    EXPECT_THAT(matcher_->match_all(left, right), IsEmpty());
}

TEST_F(StereoMatcherTest, SizeGateDisabledWhenNoTargetSizeConfigured) {
    auto config = config_;
    config.detection.target_size_m = 0.0;
    StereoMatcher matcher(config.stereo, config.detection, config.bounding_box);

    // With no expectation to check against, the gate must not invent one: the
    // same oversized pair that the default config rejects now matches.
    const std::vector<Blob> left{make_blob(300.0, 200.0, 100)};
    const std::vector<Blob> right{make_blob(220.0, 200.0, 100)};
    EXPECT_EQ(matcher.match_all(left, right).size(), 1u);
}

TEST_F(StereoMatcherTest, NanSizeToleranceFailsClosed) {
    auto config = config_;
    config.detection.size_tolerance_factor = std::numeric_limits<double>::quiet_NaN();
    StereoMatcher matcher(config.stereo, config.detection, config.bounding_box);

    // Sanitized to the strictest band (x1.0, exact-size-only): at 8.7 px
    // expected, even an honest 9 px mosquito-sized blob cannot land inside.
    EXPECT_THAT(matcher.match_all({make_blob(300.0, 200.0, 9)},
                                  {make_blob(220.0, 200.0, 9)}),
                IsEmpty());
}

// match_all: two clean, mutually exclusive correspondences in one frame are two
// targets — the multi-target case the old match() collapsed to nullopt.
TEST_F(StereoMatcherTest, MatchAllReturnsBothCleanPairs) {
    const std::vector<Blob> left{make_blob(300.0, 200.0, 9),
                                 make_blob(400.0, 200.0, 9)};
    const std::vector<Blob> right{make_blob(220.0, 200.0, 9),
                                  make_blob(320.0, 200.0, 9)};

    auto points = matcher_->match_all(left, right);

    ASSERT_EQ(points.size(), 2u);
    // d = 80 -> z = 0.75 for both; x comes from the left pixel.
    EXPECT_NEAR(points[0].z, 0.75, 1e-9);
    EXPECT_NEAR(points[0].x, -0.03, 1e-9);   // (300 - 320) * 0.75 / 500
    EXPECT_NEAR(points[1].z, 0.75, 1e-9);
    EXPECT_NEAR(points[1].x, 0.12, 1e-9);    // (400 - 320) * 0.75 / 500
}

// Per-cluster exclusivity: L1-R1 and L2-R1 both pass every gate but share R1,
// so the whole ambiguous cluster is void — while a clean pair elsewhere in the
// same frame still produces its target. The old all-or-nothing rule would have
// sunk the clean pair too.
TEST_F(StereoMatcherTest, MatchAllAmbiguousClusterFailsClosedButCleanPairSurvives) {
    const std::vector<Blob> left{make_blob(300.0, 200.0, 9),    // L1
                                 make_blob(320.0, 200.0, 9),    // L2
                                 make_blob(500.0, 300.0, 9)};   // L3
    const std::vector<Blob> right{make_blob(220.0, 200.0, 9),   // R1
                                  make_blob(420.0, 300.0, 9)};  // R3

    // L1-R1: d = 80, valid. L2-R1: d = 100, valid. Both claim R1 -> cluster void.
    // L3-R3: d = 80, exclusive -> survives.
    auto points = matcher_->match_all(left, right);

    ASSERT_EQ(points.size(), 1u);
    EXPECT_NEAR(points[0].z, 0.75, 1e-9);
    EXPECT_NEAR(points[0].x, 0.27, 1e-9);    // (500 - 320) * 0.75 / 500
    EXPECT_NEAR(points[0].y, 0.15, 1e-9);    // (300 - 200) * 0.75 / 500

    // The cluster members genuinely would match in isolation — the void is
    // about the shared blob, not the gates.
    EXPECT_TRUE(matcher_->match({make_blob(300.0, 200.0, 9)},
                                {make_blob(220.0, 200.0, 9)})
                    .has_value());
    EXPECT_TRUE(matcher_->match({make_blob(320.0, 200.0, 9)},
                                {make_blob(220.0, 200.0, 9)})
                    .has_value());
}

}   // namespace
