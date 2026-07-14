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
        gl.angle_x_min_deg = -25.0;
        gl.angle_x_max_deg = 25.0;
        gl.angle_y_min_deg = -25.0;
        gl.angle_y_max_deg = 25.0;

        bbox_ = std::make_unique<BoundingBox3D>(bb);
        mapper_ = std::make_unique<CoordinateMapper>(*bbox_, gl);
    }

    std::unique_ptr<BoundingBox3D> bbox_;
    std::unique_ptr<CoordinateMapper> mapper_;
};

TEST_F(CoordinateMapperTest, ValidTargetInCenterReturnsValidDac) {
    auto result = mapper_->map_to_dac({0.0, 0.0, 1.0});
    EXPECT_TRUE(result.has_value());

    auto dac = result.value();
    EXPECT_GE(dac.channel_a, 0);
    EXPECT_LE(dac.channel_a, 4095);
    EXPECT_GE(dac.channel_b, 0);
    EXPECT_LE(dac.channel_b, 4095);
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

TEST_F(CoordinateMapperTest, TargetBehindBaselineRejected) {
    auto result = mapper_->map_to_dac({0.0, 0.0, -1.0});
    EXPECT_FALSE(result.has_value());
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
    auto left = mapper_->map_to_dac({-0.3, 0.0, 1.0});
    auto right = mapper_->map_to_dac({0.3, 0.0, 1.0});

    ASSERT_TRUE(left.has_value());
    ASSERT_TRUE(right.has_value());

    auto center = mapper_->map_to_dac({0.0, 0.0, 1.0});
    ASSERT_TRUE(center.has_value());

    EXPECT_LT(left->channel_a, center->channel_a);
    EXPECT_GT(right->channel_a, center->channel_a);
}
