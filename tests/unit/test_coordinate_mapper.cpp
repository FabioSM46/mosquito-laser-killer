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

class CoordinateMapperTest : public Test {
protected:
    void SetUp() override {
        SystemConfig::BoundingBox bb;
        bb.x_min = -1.0;
        bb.x_max = 1.0;
        bb.y_min = -1.0;
        bb.y_max = 1.0;
        bb.z_min = 0.3;
        bb.z_max = 5.0;

        SystemConfig::GalvoLimits gl;
        gl.angle_x_min_deg = -15.0;
        gl.angle_x_max_deg = 15.0;
        gl.angle_y_min_deg = -15.0;
        gl.angle_y_max_deg = 15.0;

        SystemConfig::GalvoDriver gd;
        gd.input_scale_v_per_deg = 0.33;
        gd.dac_max_diff_voltage = 5.0;
        gd.driver_input_voltage = 15.0;

        bbox_ = std::make_unique<BoundingBox3D>(bb);
        mapper_ = std::make_unique<CoordinateMapper>(*bbox_, gl, 5.0, gd);
    }

    std::unique_ptr<BoundingBox3D> bbox_;
    std::unique_ptr<CoordinateMapper> mapper_;
};

TEST_F(CoordinateMapperTest, ValidTargetInCenterReturnsMidScaleDac) {
    auto result = mapper_->map_to_dac({0.0, 0.0, 1.0});
    ASSERT_TRUE(result.has_value());

    auto dac = result.value();
    EXPECT_NEAR(dac.channel_a, 2048, 1);
    EXPECT_NEAR(dac.channel_b, 2048, 1);
}

TEST_F(CoordinateMapperTest, TargetOutsideBoundingBoxRejected) {
    auto result = mapper_->map_to_dac({10.0, 0.0, 1.0});
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
    // atan2(1.0, 0.3) ≈ 73° ≫ 15°
    auto result = mapper_->map_to_dac({1.0, 0.0, 0.3});
    EXPECT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), MappingError::GalvoAngleLimitExceeded);
}

TEST_F(CoordinateMapperTest, DacValuesWithin12BitRange) {
    // Sweep off-axis targets across the cone: on-axis points all map to 2048,
    // so an on-axis sweep cannot fail even with the 0-4095 range check deleted.
    // tan(15 deg) = 0.268, so x in [-0.25, 0.25] at z = 1 stays inside the
    // +/-15 deg limits and the 5 V budget.
    bool saw_above_center = false;
    bool saw_below_center = false;
    for (double x = -0.25; x <= 0.25; x += 0.05) {
        auto result = mapper_->map_to_dac({x, x / 2.0, 1.0});
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
    // ±0.1 m at z=1 m ≈ ±5.7° — within ±15° and voltage budget
    auto left = mapper_->map_to_dac({-0.1, 0.0, 1.0});
    auto right = mapper_->map_to_dac({0.1, 0.0, 1.0});

    ASSERT_TRUE(left.has_value());
    ASSERT_TRUE(right.has_value());

    auto center = mapper_->map_to_dac({0.0, 0.0, 1.0});
    ASSERT_TRUE(center.has_value());

    EXPECT_LT(left->channel_a, center->channel_a);
    EXPECT_GT(right->channel_a, center->channel_a);
}

TEST_F(CoordinateMapperTest, VoltageScaleRejectsBeyondMaxDiff) {
    SystemConfig::BoundingBox bb;
    bb.x_min = -2.0;
    bb.x_max = 2.0;
    bb.y_min = -2.0;
    bb.y_max = 2.0;
    bb.z_min = 0.1;
    bb.z_max = 10.0;

    SystemConfig::GalvoLimits gl;
    gl.angle_x_min_deg = -30.0;
    gl.angle_x_max_deg = 30.0;
    gl.angle_y_min_deg = -30.0;
    gl.angle_y_max_deg = 30.0;

    SystemConfig::GalvoDriver gd;
    gd.input_scale_v_per_deg = 0.33;
    gd.dac_max_diff_voltage = 5.0;

    BoundingBox3D box(bb);
    CoordinateMapper mapper(box, gl, 5.0, gd);

    // ~26.6° → V_diff ≈ 8.8 V > 5 V
    auto result = mapper.map_to_dac({0.5, 0.0, 1.0});
    EXPECT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), MappingError::DacRangeInvalid);
}

TEST_F(CoordinateMapperTest, DoesNotClampOutOfRangeCodes) {
    // Angle within mechanical limits (±40°) but beyond voltage budget (~15.15°).
    // tan(20°) ≈ 0.364 → V_diff = 20·0.33 = 6.6 V > 5 V → reject, no clamp.
    SystemConfig::BoundingBox bb{-2, 2, -2, 2, 0.1, 10};
    SystemConfig::GalvoLimits gl{-40, 40, -40, 40};
    SystemConfig::GalvoDriver gd{0.33, 5.0, 15.0};
    BoundingBox3D box(bb);
    CoordinateMapper mapper(box, gl, 5.0, gd);

    auto result = mapper.map_to_dac({0.364, 0.0, 1.0});
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
    SystemConfig::BoundingBox bb{-1, 1, -1, 1, 0.3, 5.0};
    SystemConfig::GalvoLimits gl{-15, 15, -15, 15};
    SystemConfig::GalvoDriver gd{0.33, 5.0, 15.0};
    BoundingBox3D box(bb);
    CoordinateMapper mapper(box, gl, std::nan(""), gd);

    auto result = mapper.map_to_dac({0.0, 0.0, 1.0});
    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), MappingError::ConversionError);
}

TEST_F(CoordinateMapperTest, NonPositiveDacRefVoltageRejectsAsConversionError) {
    SystemConfig::BoundingBox bb{-1, 1, -1, 1, 0.3, 5.0};
    SystemConfig::GalvoLimits gl{-15, 15, -15, 15};
    SystemConfig::GalvoDriver gd{0.33, 5.0, 15.0};
    BoundingBox3D box(bb);

    for (const double vref : {0.0, -5.0}) {
        CoordinateMapper mapper(box, gl, vref, gd);
        auto result = mapper.map_to_dac({0.0, 0.0, 1.0});
        ASSERT_FALSE(result.has_value()) << "vref=" << vref;
        EXPECT_EQ(result.error(), MappingError::ConversionError) << "vref=" << vref;
    }
}

TEST_F(CoordinateMapperTest, NanInputScaleRejectsAsConversionError) {
    SystemConfig::BoundingBox bb{-1, 1, -1, 1, 0.3, 5.0};
    SystemConfig::GalvoLimits gl{-15, 15, -15, 15};
    SystemConfig::GalvoDriver gd{std::nan(""), 5.0, 15.0};
    BoundingBox3D box(bb);
    CoordinateMapper mapper(box, gl, 5.0, gd);

    auto result = mapper.map_to_dac({0.0, 0.0, 1.0});
    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), MappingError::ConversionError);
}
