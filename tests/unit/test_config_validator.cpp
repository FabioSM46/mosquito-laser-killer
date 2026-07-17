#include <vector>
#include <cmath>
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
