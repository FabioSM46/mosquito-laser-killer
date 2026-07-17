#include <vector>
#include <gtest/gtest.h>
#include <memory>
#include <cmath>
#include <algorithm>
#include <string_view>
#include <ranges>

#include "core/types.h"
#include "safety/config_validator.h"

using namespace testing;

namespace {
auto find_warning(const std::vector<ValidationWarning>& warnings, std::string_view category)
    -> bool {
    return std::ranges::any_of(warnings,
        [&](const auto& w) { return w.category == category; });
}
}

class ConfigValidatorTest : public Test {
protected:
    SystemConfig config_;

    void SetUp() override {
        config_.bounding_box = {-0.09, 0.09, -0.09, 0.09, 0.5, 1.0};
        config_.galvo_limits = {-15.0, 15.0, -15.0, 15.0};
        config_.galvo_driver = {0.33, 5.0, 15.0};
        config_.camera_optics = {3.0, 3.84, 2.4};
        config_.stereo = {0.12, 500.0, 320.0, 240.0};
    }
};

TEST_F(ConfigValidatorTest, DefaultRealisticConfigHasNoWarnings) {
    auto warnings = validate_engagement_volume(config_);
    EXPECT_TRUE(warnings.empty());
    EXPECT_FALSE(has_critical_validation_errors(warnings));
}

TEST_F(ConfigValidatorTest, BoxCornerBeyondGalvoConeIsCritical) {
    config_.bounding_box.x_max = 0.3;
    config_.bounding_box.y_max = 0.3;

    auto warnings = validate_engagement_volume(config_);
    EXPECT_TRUE(find_warning(warnings, "bounding-box"));
    EXPECT_TRUE(has_critical_validation_errors(warnings));
}

TEST_F(ConfigValidatorTest, GalvoConeExceedingDriverVoltageIsCritical) {
    config_.galvo_limits.angle_x_min_deg = -20.0;
    config_.galvo_limits.angle_x_max_deg = 20.0;
    config_.galvo_limits.angle_y_min_deg = -20.0;
    config_.galvo_limits.angle_y_max_deg = 20.0;

    auto warnings = validate_engagement_volume(config_);
    EXPECT_TRUE(find_warning(warnings, "galvo-voltage"));
    EXPECT_TRUE(has_critical_validation_errors(warnings));
}

TEST_F(ConfigValidatorTest, NarrowLensFovIsNonCriticalWarning) {
    config_.camera_optics.lens_focal_length_mm = 10.0;

    auto warnings = validate_engagement_volume(config_);
    EXPECT_TRUE(find_warning(warnings, "camera-fov"));
    EXPECT_FALSE(has_critical_validation_errors(warnings));
}

TEST_F(ConfigValidatorTest, ThreeMmLensFovIsConsistent) {
    auto warnings = validate_engagement_volume(config_);
    EXPECT_FALSE(find_warning(warnings, "camera-fov"));
}

// A NaN reaching any of these fields must abort startup. `x <= 0.0` is FALSE for
// NaN, so the natural phrasing fails OPEN — and the validator is the last line of
// defense for the two galvo-driver fields: a NaN scale makes map_to_dac's own
// voltage guards false as well, and std::lround(NaN) then yields DAC code 0
// (full deflection on both axes) returned as a SUCCESS, with the beam steered to
// a corner of the cone the bounding box was never checked against.
TEST_F(ConfigValidatorTest, NanInAnyCriticalNumericFieldAbortsStartup) {
    const double nan = std::nan("");

    struct Field {
        const char* name;
        double SystemConfig::* member;
    };

    // Top-level doubles.
    for (const auto& f : std::vector<Field>{
             {"settle_delay_ms", &SystemConfig::settle_delay_ms},
             {"max_pulse_duration_ms", &SystemConfig::max_pulse_duration_ms},
             {"cooldown_seconds", &SystemConfig::cooldown_seconds},
             {"watchdog_timeout_ms", &SystemConfig::watchdog_timeout_ms},
             {"watchdog_startup_grace_ms", &SystemConfig::watchdog_startup_grace_ms},
         }) {
        auto config = config_;
        config.*(f.member) = nan;
        EXPECT_TRUE(has_critical_validation_errors(validate_engagement_volume(config)))
            << f.name << ": NaN passed validation";
    }

    // Nested fields that feed the DAC voltage chain and the back-projection.
    {
        auto config = config_;
        config.galvo_driver.input_scale_v_per_deg = nan;
        EXPECT_TRUE(has_critical_validation_errors(validate_engagement_volume(config)))
            << "input_scale_v_per_deg: NaN passed validation";
    }
    {
        auto config = config_;
        config.galvo_driver.dac_max_diff_voltage = nan;
        EXPECT_TRUE(has_critical_validation_errors(validate_engagement_volume(config)))
            << "dac_max_diff_voltage: NaN passed validation";
    }
    {
        auto config = config_;
        config.stereo.baseline_m = nan;
        EXPECT_TRUE(has_critical_validation_errors(validate_engagement_volume(config)))
            << "baseline_m: NaN passed validation";
    }
    {
        auto config = config_;
        config.stereo.focal_length_px = nan;
        EXPECT_TRUE(has_critical_validation_errors(validate_engagement_volume(config)))
            << "focal_length_px: NaN passed validation";
    }
    {
        auto config = config_;
        config.stereo.cx = nan;
        EXPECT_TRUE(has_critical_validation_errors(validate_engagement_volume(config)))
            << "stereo.cx: NaN passed validation";
    }
    {
        auto config = config_;
        config.stereo.cy = nan;
        EXPECT_TRUE(has_critical_validation_errors(validate_engagement_volume(config)))
            << "stereo.cy: NaN passed validation";
    }
    {
        auto config = config_;
        config.detection.epipolar_tolerance_px = nan;
        EXPECT_TRUE(has_critical_validation_errors(validate_engagement_volume(config)))
            << "epipolar_tolerance_px: NaN passed validation";
    }
    // The mapper's own `dac_ref_voltage_ <= 0.0` guard fails open on NaN and
    // lround(NaN) wraps to DAC code 0 (full deflection, reported as success) —
    // this check is the only place that can still catch it.
    {
        auto config = config_;
        config.dac_ref_voltage = nan;
        EXPECT_TRUE(has_critical_validation_errors(validate_engagement_volume(config)))
            << "dac_ref_voltage: NaN passed validation";
    }
    // A NaN bounding-box coordinate: `c.z <= 0.0` and `angle_deg > cone` are
    // both false for NaN, so an unguarded corner check sails through.
    {
        auto config = config_;
        config.bounding_box.z_min = nan;
        EXPECT_TRUE(has_critical_validation_errors(validate_engagement_volume(config)))
            << "bounding_box.z_min: NaN passed validation";
    }
    {
        auto config = config_;
        config.bounding_box.x_max = nan;
        EXPECT_TRUE(has_critical_validation_errors(validate_engagement_volume(config)))
            << "bounding_box.x_max: NaN passed validation";
    }
    // NaN or inverted galvo limits disable the cone check in map_to_dac itself
    // (`angle < min` / `angle > max` are both false for NaN bounds).
    {
        auto config = config_;
        config.galvo_limits.angle_x_max_deg = nan;
        EXPECT_TRUE(has_critical_validation_errors(validate_engagement_volume(config)))
            << "galvo_limits.angle_x_max_deg: NaN passed validation";
    }
}

TEST_F(ConfigValidatorTest, HorizontalFovDerivationCorrect) {
    constexpr double expected_hfov = 2.0 * std::atan(3.84 / 6.0) * (180.0 / M_PI);
    EXPECT_NEAR(horizontal_fov_deg(config_.camera_optics), expected_hfov, 0.01);
}

TEST_F(ConfigValidatorTest, SixMmLensFovIsNarrower) {
    config_.camera_optics.lens_focal_length_mm = 6.0;
    const auto fov3 = horizontal_fov_deg({3.0, 3.84, 2.4});
    const auto fov6 = horizontal_fov_deg(config_.camera_optics);
    EXPECT_LT(fov6, fov3);
}

TEST_F(ConfigValidatorTest, InvalidStereoBaselineIsCritical) {
    config_.stereo.baseline_m = 0.0;
    auto warnings = validate_engagement_volume(config_);
    EXPECT_TRUE(find_warning(warnings, "stereo"));
    EXPECT_TRUE(has_critical_validation_errors(warnings));
}

TEST_F(ConfigValidatorTest, MaxCommandableAngleMatchesDriverScale) {
    config_.bounding_box = {-2.0, 2.0, -2.0, 2.0, 5.0, 10.0};
    config_.galvo_limits = {-25.0, 25.0, -25.0, 25.0};

    auto warnings = validate_engagement_volume(config_);
    EXPECT_TRUE(find_warning(warnings, "galvo-voltage"));
    EXPECT_TRUE(has_critical_validation_errors(warnings));
}

TEST_F(ConfigValidatorTest, InvalidPulseDurationIsCritical) {
    config_.max_pulse_duration_ms = 250.0;
    auto warnings = validate_engagement_volume(config_);
    EXPECT_TRUE(find_warning(warnings, "pulse"));
    EXPECT_TRUE(has_critical_validation_errors(warnings));
}

TEST_F(ConfigValidatorTest, NonPositiveDacReferenceVoltageIsCritical) {
    for (const double v : {0.0, -5.0}) {
        auto config = config_;
        config.dac_ref_voltage = v;
        auto warnings = validate_engagement_volume(config);
        EXPECT_TRUE(find_warning(warnings, "galvo-voltage")) << "v=" << v;
        EXPECT_TRUE(has_critical_validation_errors(warnings)) << "v=" << v;
    }
}

TEST_F(ConfigValidatorTest, InvertedGalvoLimitsAreCritical) {
    config_.galvo_limits.angle_x_min_deg = 15.0;
    config_.galvo_limits.angle_x_max_deg = -15.0;

    auto warnings = validate_engagement_volume(config_);
    EXPECT_TRUE(find_warning(warnings, "galvo-limits"));
    EXPECT_TRUE(has_critical_validation_errors(warnings));
}

TEST_F(ConfigValidatorTest, ZeroWidthGalvoRangeIsCritical) {
    config_.galvo_limits.angle_y_min_deg = 5.0;
    config_.galvo_limits.angle_y_max_deg = 5.0;

    auto warnings = validate_engagement_volume(config_);
    EXPECT_TRUE(find_warning(warnings, "galvo-limits"));
    EXPECT_TRUE(has_critical_validation_errors(warnings));
}

//
// Boundary tests for every remaining critical bound in §4.10. Each of these
// pairs an out-of-range value (must abort) with the nearest in-range value
// (must not abort), so a deleted or shifted bound fails one half of the pair.
//

TEST_F(ConfigValidatorTest, CooldownBoundary) {
    // cooldown_seconds: 0 lets end_pulse set cooldown_until_ = now, so the next
    // re-acquire re-fires within a few cycles — effectively CW 2.5W.
    for (const double v : {0.0, 0.9}) {
        auto config = config_;
        config.cooldown_seconds = v;
        EXPECT_TRUE(has_critical_validation_errors(validate_engagement_volume(config)))
            << "cooldown_seconds=" << v << " passed validation";
    }
    auto config = config_;
    config.cooldown_seconds = 1.0;
    EXPECT_FALSE(has_critical_validation_errors(validate_engagement_volume(config)));
}

TEST_F(ConfigValidatorTest, SettleDelayBoundary) {
    // settle_delay_ms: 0 marks the galvo settled in the same cycle the DAC was
    // written, firing while the mirrors slew.
    for (const double v : {0.0, 0.4, 50.1}) {
        auto config = config_;
        config.settle_delay_ms = v;
        EXPECT_TRUE(has_critical_validation_errors(validate_engagement_volume(config)))
            << "settle_delay_ms=" << v << " passed validation";
    }
    for (const double v : {0.5, 50.0}) {
        auto config = config_;
        config.settle_delay_ms = v;
        EXPECT_FALSE(has_critical_validation_errors(validate_engagement_volume(config)))
            << "settle_delay_ms=" << v << " rejected";
    }
}

TEST_F(ConfigValidatorTest, WatchdogTimeoutBoundary) {
    for (const double v : {4.0, 501.0}) {
        auto config = config_;
        config.watchdog_timeout_ms = v;
        EXPECT_TRUE(has_critical_validation_errors(validate_engagement_volume(config)))
            << "watchdog_timeout_ms=" << v << " passed validation";
    }
    for (const double v : {5.0, 500.0}) {
        auto config = config_;
        config.watchdog_timeout_ms = v;
        EXPECT_FALSE(has_critical_validation_errors(validate_engagement_volume(config)))
            << "watchdog_timeout_ms=" << v << " rejected";
    }
}

TEST_F(ConfigValidatorTest, WatchdogStartupGraceBoundary) {
    for (const double v : {99.0, 60001.0}) {
        auto config = config_;
        config.watchdog_startup_grace_ms = v;
        EXPECT_TRUE(has_critical_validation_errors(validate_engagement_volume(config)))
            << "watchdog_startup_grace_ms=" << v << " passed validation";
    }
    for (const double v : {100.0, 60000.0}) {
        auto config = config_;
        config.watchdog_startup_grace_ms = v;
        EXPECT_FALSE(has_critical_validation_errors(validate_engagement_volume(config)))
            << "watchdog_startup_grace_ms=" << v << " rejected";
    }
}

TEST_F(ConfigValidatorTest, TargetFpsBoundary) {
    // main derives cycle periods as 1'000'000 / target_fps; zero is a SIGFPE.
    for (const int v : {0, -120}) {
        auto config = config_;
        config.target_fps = v;
        EXPECT_TRUE(has_critical_validation_errors(validate_engagement_volume(config)))
            << "target_fps=" << v << " passed validation";
    }
}

TEST_F(ConfigValidatorTest, FrameDimensionsBoundary) {
    for (const int v : {0, -640}) {
        auto config = config_;
        config.frame_width = v;
        EXPECT_TRUE(has_critical_validation_errors(validate_engagement_volume(config)))
            << "frame_width=" << v << " passed validation";
        config = config_;
        config.frame_height = v;
        EXPECT_TRUE(has_critical_validation_errors(validate_engagement_volume(config)))
            << "frame_height=" << v << " passed validation";
    }
}

TEST_F(ConfigValidatorTest, PrincipalPointBoundary) {
    // The last valid column in a 640-wide frame is 639; 640 is outside.
    for (const double cx : {-1.0, 640.0, 1000.0}) {
        auto config = config_;
        config.stereo.cx = cx;
        EXPECT_TRUE(has_critical_validation_errors(validate_engagement_volume(config)))
            << "cx=" << cx << " passed validation";
    }
    for (const double cy : {-1.0, 400.0, 1000.0}) {
        auto config = config_;
        config.stereo.cy = cy;
        EXPECT_TRUE(has_critical_validation_errors(validate_engagement_volume(config)))
            << "cy=" << cy << " passed validation";
    }
    auto config = config_;
    config.stereo.cx = 639.0;
    config.stereo.cy = 399.0;
    EXPECT_FALSE(has_critical_validation_errors(validate_engagement_volume(config)));
}

TEST_F(ConfigValidatorTest, DetectionThresholdBoundary) {
    for (const int v : {0, 255, -1}) {
        auto config = config_;
        config.detection.threshold = v;
        EXPECT_TRUE(has_critical_validation_errors(validate_engagement_volume(config)))
            << "threshold=" << v << " passed validation";
    }
    for (const int v : {1, 254}) {
        auto config = config_;
        config.detection.threshold = v;
        EXPECT_FALSE(has_critical_validation_errors(validate_engagement_volume(config)))
            << "threshold=" << v << " rejected";
    }
}

TEST_F(ConfigValidatorTest, BlobAreaBoundsBoundary) {
    {
        auto config = config_;
        config.detection.min_blob_area_px = 0;
        EXPECT_TRUE(has_critical_validation_errors(validate_engagement_volume(config)))
            << "min_blob_area_px=0 passed validation";
    }
    {
        auto config = config_;
        config.detection.min_blob_area_px = 500;
        config.detection.max_blob_area_px = 400;
        EXPECT_TRUE(has_critical_validation_errors(validate_engagement_volume(config)))
            << "min > max blob area passed validation";
    }
    {
        auto config = config_;
        config.detection.max_blobs = 0;
        EXPECT_TRUE(has_critical_validation_errors(validate_engagement_volume(config)))
            << "max_blobs=0 passed validation";
    }
}

TEST_F(ConfigValidatorTest, EpipolarToleranceBoundary) {
    for (const double v : {0.0, -1.0}) {
        auto config = config_;
        config.detection.epipolar_tolerance_px = v;
        EXPECT_TRUE(has_critical_validation_errors(validate_engagement_volume(config)))
            << "epipolar_tolerance_px=" << v << " passed validation";
    }
}

TEST_F(ConfigValidatorTest, PulseDurationBoundary) {
    for (const double v : {0.0, 100.1}) {
        auto config = config_;
        config.max_pulse_duration_ms = v;
        EXPECT_TRUE(has_critical_validation_errors(validate_engagement_volume(config)))
            << "max_pulse_duration_ms=" << v << " passed validation";
    }
    auto config = config_;
    config.max_pulse_duration_ms = 100.0;
    EXPECT_FALSE(has_critical_validation_errors(validate_engagement_volume(config)));
}

TEST_F(ConfigValidatorTest, BackgroundLearningRateBoundary) {
    // 0 disables the motion gate by design; above 0.5 the model chases the
    // current frame so fast a slow target erases itself — silent blindness.
    for (const double v : {-0.1, 0.51, std::nan("")}) {
        auto config = config_;
        config.detection.background_learning_rate = v;
        EXPECT_TRUE(has_critical_validation_errors(validate_engagement_volume(config)))
            << "background_learning_rate=" << v << " passed validation";
    }
    for (const double v : {0.0, 0.05, 0.5}) {
        auto config = config_;
        config.detection.background_learning_rate = v;
        EXPECT_FALSE(has_critical_validation_errors(validate_engagement_volume(config)))
            << "background_learning_rate=" << v << " rejected";
    }
}

TEST_F(ConfigValidatorTest, MotionThresholdBoundary) {
    for (const int v : {0, 255, -1}) {
        auto config = config_;
        config.detection.motion_threshold = v;
        EXPECT_TRUE(has_critical_validation_errors(validate_engagement_volume(config)))
            << "motion_threshold=" << v << " passed validation";
    }
}

TEST_F(ConfigValidatorTest, SizeToleranceFactorBoundary) {
    // Below 1.0 the accept band inverts and rejects every real target.
    for (const double v : {0.5, 0.0, -2.0, 100.1, std::nan("")}) {
        auto config = config_;
        config.detection.size_tolerance_factor = v;
        EXPECT_TRUE(has_critical_validation_errors(validate_engagement_volume(config)))
            << "size_tolerance_factor=" << v << " passed validation";
    }
    auto config = config_;
    config.detection.size_tolerance_factor = 1.0;
    EXPECT_FALSE(has_critical_validation_errors(validate_engagement_volume(config)));
}

TEST_F(ConfigValidatorTest, TrackingConfirmHitsBoundary) {
    for (const int v : {0, -1, 61}) {
        auto config = config_;
        config.tracking.confirm_hits = v;
        EXPECT_TRUE(has_critical_validation_errors(validate_engagement_volume(config)))
            << "confirm_hits=" << v << " passed validation";
    }
}

TEST_F(ConfigValidatorTest, TrackingAssociationGateBoundary) {
    // A NaN gate makes every distance comparison false -> nothing associates
    // (fail closed), but the system is then silently blind; reject at startup.
    for (const double v : {0.0, -0.1, 1.01, std::nan("")}) {
        auto config = config_;
        config.tracking.association_gate_m = v;
        EXPECT_TRUE(has_critical_validation_errors(validate_engagement_volume(config)))
            << "association_gate_m=" << v << " passed validation";
    }
}

TEST_F(ConfigValidatorTest, TrackingSpeedWindowBoundary) {
    {
        auto config = config_;
        config.tracking.min_speed_mps = -0.5;
        EXPECT_TRUE(has_critical_validation_errors(validate_engagement_volume(config)))
            << "negative min_speed_mps passed validation";
    }
    {
        auto config = config_;
        config.tracking.min_speed_mps = 3.0;
        config.tracking.max_speed_mps = 3.0;
        EXPECT_TRUE(has_critical_validation_errors(validate_engagement_volume(config)))
            << "empty speed window passed validation";
    }
    {
        auto config = config_;
        config.tracking.max_speed_mps = std::nan("");
        EXPECT_TRUE(has_critical_validation_errors(validate_engagement_volume(config)))
            << "NaN max_speed_mps passed validation";
    }
    {
        auto config = config_;
        config.tracking.max_speed_mps = 25.0;
        EXPECT_TRUE(has_critical_validation_errors(validate_engagement_volume(config)))
            << "absurd max_speed_mps passed validation";
    }
}

TEST_F(ConfigValidatorTest, TrackingMaxTracksBoundary) {
    for (const int v : {0, -4, 257}) {
        auto config = config_;
        config.tracking.max_tracks = v;
        EXPECT_TRUE(has_critical_validation_errors(validate_engagement_volume(config)))
            << "max_tracks=" << v << " passed validation";
    }
}
