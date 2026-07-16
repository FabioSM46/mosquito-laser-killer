#include <gtest/gtest.h>
#include <chrono>
#include <memory>

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
    for (double z = 0.5; z <= 4.5; z += 0.5) {
        auto result = mapper_->map_to_dac({0.0, 0.0, z});
        if (result.has_value()) {
            auto dac = result.value();
            EXPECT_LE(dac.channel_a, 4095);
            EXPECT_LE(dac.channel_b, 4095);
        }
    }
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
