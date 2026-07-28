#include <gtest/gtest.h>
#include <chrono>
#include <memory>
#include <cmath>
#include <limits>

#include "core/types.h"
#include "core/error.h"
#include "safety/bounding_box.h"
#include "control/coordinate_mapper.h"

using namespace testing;

// The main fixture runs the PRODUCTION geometry from core/types.h — box
// x,y ∈ ±0.09 m, z ∈ [0.5, 1.0] m, cone ±15°, 0.33 V/°. This fixture once
// used a ±1 m box with z down to 0.3 m, values the project's own
// config_validator rejects as critical (§7 forbids fixtures more permissive
// than production). A handful of tests below construct deliberately
// validator-INVALID configs, each labelled: they exercise the mapper's own
// inner guards, which a fully validated config makes unreachable by design —
// defense in depth per §4.5.
class CoordinateMapperTest : public Test {
protected:
    void SetUp() override {
        bbox_ = std::make_unique<BoundingBox3D>(config_.bounding_box);
        mapper_ = std::make_unique<CoordinateMapper>(
            *bbox_, config_.galvo_limits, config_.dac_ref_voltage,
            config_.galvo_driver);
    }

    SystemConfig config_{};
    std::unique_ptr<BoundingBox3D> bbox_;
    std::unique_ptr<CoordinateMapper> mapper_;
};

TEST_F(CoordinateMapperTest, ValidTargetInCenterReturnsMidScaleDac) {
    auto result = mapper_->map_to_dac({0.0, 0.0, 0.7});
    ASSERT_TRUE(result.has_value());

    auto dac = result.value();
    EXPECT_NEAR(dac.channel_a, 2048, 1);
    EXPECT_NEAR(dac.channel_b, 2048, 1);
}

TEST_F(CoordinateMapperTest, TargetOutsideBoundingBoxRejected) {
    auto result = mapper_->map_to_dac({10.0, 0.0, 0.7});
    EXPECT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), MappingError::OutOfBounds);
}

TEST_F(CoordinateMapperTest, TargetTooCloseRejected) {
    auto result = mapper_->map_to_dac({0.0, 0.0, 0.1});
    EXPECT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), MappingError::OutOfBounds);
}

TEST_F(CoordinateMapperTest, TargetTooFarRejected) {
    auto result = mapper_->map_to_dac({0.0, 0.0, 10.0});
    EXPECT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), MappingError::OutOfBounds);
}

TEST_F(CoordinateMapperTest, NegativeZRejectedAsOutOfBounds) {
    auto result = mapper_->map_to_dac({0.0, 0.0, -1.0});
    EXPECT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), MappingError::OutOfBounds);
}

TEST_F(CoordinateMapperTest, TargetInBoxButBeyondGalvoConeRejected) {
    // Deliberately validator-INVALID box (±0.2 m at z_min = 0.5 needs ~29.5°):
    // in a validated config no in-box point can exceed the cone — that is what
    // validation proves — so reaching the mapper's own cone guard requires a
    // box the validator would abort on. Defense in depth, not a template for
    // behaviour tests.
    SystemConfig::BoundingBox bb{-0.2, 0.2, -0.2, 0.2, 0.5, 1.0};
    BoundingBox3D box(bb);
    CoordinateMapper mapper(box, config_.galvo_limits, config_.dac_ref_voltage,
                            config_.galvo_driver);

    // atan2(0.2, 0.5) ≈ 21.8° > 15°
    auto result = mapper.map_to_dac({0.2, 0.0, 0.5});
    EXPECT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), MappingError::GalvoAngleLimitExceeded);
}

TEST_F(CoordinateMapperTest, DacValuesWithin12BitRange) {
    // Sweep off-axis targets across the production box: on-axis points all map
    // to 2048, so an on-axis sweep cannot fail even with the 0-4095 range
    // check deleted. ±0.08 m at z = 0.75 is ±6.1° → ±2.0 V, well inside the
    // ±15° / 5 V budget.
    bool saw_above_center = false;
    bool saw_below_center = false;
    for (double x = -0.08; x <= 0.08; x += 0.02) {
        auto result = mapper_->map_to_dac({x, x / 2.0, 0.75});
        ASSERT_TRUE(result.has_value()) << "x=" << x;
        auto dac = result.value();
        EXPECT_GE(dac.channel_a, 0u);
        EXPECT_LE(dac.channel_a, 4095u);
        EXPECT_GE(dac.channel_b, 0u);
        EXPECT_LE(dac.channel_b, 4095u);
        saw_above_center |= dac.channel_a > 2048;
        saw_below_center |= dac.channel_a < 2048;
    }
    // The sweep must actually exercise both sides of the range, or the bounds
    // assertions above are vacuous (a constant 2048 passes them).
    EXPECT_TRUE(saw_above_center);
    EXPECT_TRUE(saw_below_center);
}

TEST_F(CoordinateMapperTest, SymmetricXMapping) {
    // ±0.08 m at z = 0.75 m ≈ ±6.1° — inside the production box and budget.
    auto left = mapper_->map_to_dac({-0.08, 0.0, 0.75});
    auto right = mapper_->map_to_dac({0.08, 0.0, 0.75});

    ASSERT_TRUE(left.has_value());
    ASSERT_TRUE(right.has_value());

    auto center = mapper_->map_to_dac({0.0, 0.0, 0.75});
    ASSERT_TRUE(center.has_value());

    EXPECT_LT(left->channel_a, center->channel_a);
    EXPECT_GT(right->channel_a, center->channel_a);
}

TEST_F(CoordinateMapperTest, VoltageScaleRejectsBeyondMaxDiff) {
    // Production geometry with ONE deliberately validator-invalid knob: a
    // 0.5 V/° scale makes the DAC budget run out at 10°, below the ±15° cone
    // (the validator flags exactly that mismatch as critical). It is the only
    // way to reach the voltage guard from inside the box: a validated config
    // proves the guard unreachable. Defense in depth per §4.5.
    SystemConfig::GalvoDriver gd{0.5, 5.0};
    CoordinateMapper mapper(*bbox_, config_.galvo_limits,
                            config_.dac_ref_voltage, gd);

    // Corner-ish target: atan2(0.09, 0.5) ≈ 10.2° → 5.1 V > 5 V.
    auto result = mapper.map_to_dac({0.09, 0.0, 0.5});
    EXPECT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), MappingError::DacRangeInvalid);
}

TEST_F(CoordinateMapperTest, DoesNotClampOutOfRangeCodes) {
    // Same deliberately-invalid 0.5 V/° knob as above: the over-budget voltage
    // must come back as an ERROR, never as a silently clamped 0/4095 code.
    SystemConfig::GalvoDriver gd{0.5, 5.0};
    CoordinateMapper mapper(*bbox_, config_.galvo_limits,
                            config_.dac_ref_voltage, gd);

    auto result = mapper.map_to_dac({0.09, 0.09, 0.5});
    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), MappingError::DacRangeInvalid);
}

//
// Non-finite coordinates.
//
// Each of these asserts the error is specifically Invalid3DPoint — the code
// returned by the isfinite() guard at the top of map_to_dac — rather than merely
// asserting that some rejection happened. The distinction is the whole point.
// BoundingBox3D::contains() also happens to reject NaN, because every one of its
// comparisons is false for NaN, so a test that only checked has_value() would
// pass with the isfinite guard deleted and would be measuring the bounding box
// instead. That matters because every OTHER link in the chain fails OPEN on NaN:
// the galvo-limit and voltage comparisons are all false for NaN, and lround(NaN)
// is unspecified — on this target it yields LONG_MIN, which casts to 0 and passes
// the 0..4095 range check as a legitimate DAC code, i.e. full negative deflection
// on both axes, reported as success.
TEST_F(CoordinateMapperTest, NanXIsRejectedAsInvalidPointNotMerelyOutOfBounds) {
    auto result = mapper_->map_to_dac({std::nan(""), 0.0, 1.0});
    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), MappingError::Invalid3DPoint);
}

TEST_F(CoordinateMapperTest, NanYIsRejectedAsInvalidPoint) {
    auto result = mapper_->map_to_dac({0.0, std::nan(""), 1.0});
    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), MappingError::Invalid3DPoint);
}

TEST_F(CoordinateMapperTest, NanZIsRejectedAsInvalidPoint) {
    auto result = mapper_->map_to_dac({0.0, 0.0, std::nan("")});
    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), MappingError::Invalid3DPoint);
}

TEST_F(CoordinateMapperTest, InfiniteCoordinatesAreRejectedAsInvalidPoint) {
    constexpr double inf = std::numeric_limits<double>::infinity();

    for (const auto& p : {Point3D{inf, 0.0, 1.0}, Point3D{0.0, inf, 1.0},
                          Point3D{0.0, 0.0, inf}, Point3D{-inf, 0.0, 1.0}}) {
        auto result = mapper_->map_to_dac(p);
        ASSERT_FALSE(result.has_value())
            << "accepted a non-finite point (" << p.x << ", " << p.y << ", " << p.z << ")";
        EXPECT_EQ(result.error(), MappingError::Invalid3DPoint);
    }
}

//
// Non-finite / non-positive DAC reference voltage.
//
// The `dac_ref_voltage_ <= 0.0` guard alone fails OPEN on NaN: every comparison
// is false for NaN, the division yields a NaN code, and lround(NaN) is
// unspecified — on this target it wraps to a value that passes the integer
// range check as a legitimate code. The rejection must therefore happen on the
// NaN itself (isfinite) and on the double-domain code BEFORE lround. These
// tests fail if either guard is weakened back to a plain `<= 0.0` comparison
// or moved after the rounding.
TEST_F(CoordinateMapperTest, NanDacRefVoltageRejectsAsConversionError) {
    // Production geometry; only vref is (deliberately) invalid — the value
    // under test.
    CoordinateMapper mapper(*bbox_, config_.galvo_limits, std::nan(""),
                            config_.galvo_driver);

    auto result = mapper.map_to_dac({0.0, 0.0, 0.7});
    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), MappingError::ConversionError);
}

TEST_F(CoordinateMapperTest, NonPositiveDacRefVoltageRejectsAsConversionError) {
    for (const double vref : {0.0, -5.0}) {
        CoordinateMapper mapper(*bbox_, config_.galvo_limits, vref,
                                config_.galvo_driver);
        auto result = mapper.map_to_dac({0.0, 0.0, 0.7});
        ASSERT_FALSE(result.has_value()) << "vref=" << vref;
        EXPECT_EQ(result.error(), MappingError::ConversionError) << "vref=" << vref;
    }
}

TEST_F(CoordinateMapperTest, NanInputScaleRejectsAsConversionError) {
    SystemConfig::GalvoDriver gd{std::nan(""), 5.0};
    CoordinateMapper mapper(*bbox_, config_.galvo_limits,
                            config_.dac_ref_voltage, gd);

    auto result = mapper.map_to_dac({0.0, 0.0, 0.7});
    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), MappingError::ConversionError);
}
