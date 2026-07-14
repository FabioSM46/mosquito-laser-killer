#include <gtest/gtest.h>
#include <cmath>

#include "core/types.h"
#include "vision/stereo_matcher.h"

using namespace testing;

class StereoMatcherTest : public Test {
protected:
    void SetUp() override {
        SystemConfig::Stereo stereo_cfg;
        stereo_cfg.baseline_m = 0.12;
        stereo_cfg.focal_length_px = 800.0;
        stereo_cfg.cx = 320.0;
        stereo_cfg.cy = 240.0;

        matcher_ = std::make_unique<StereoMatcher>(stereo_cfg);
    }

    std::unique_ptr<StereoMatcher> matcher_;
};

TEST_F(StereoMatcherTest, TriangulationReturnsValidPoint) {
    Pixel2D left{330.0, 240.0};
    Pixel2D right{310.0, 240.0};

    auto result = matcher_->triangulate(left, right);
    ASSERT_TRUE(result.has_value());

    EXPECT_GT(result->z, 0.0);
}

TEST_F(StereoMatcherTest, SameCoordinatesReturnNullopt) {
    Pixel2D left{320.0, 240.0};
    Pixel2D right{320.0, 240.0};

    auto result = matcher_->triangulate(left, right);
    EXPECT_FALSE(result.has_value());
}

TEST_F(StereoMatcherTest, LargerDisparityYieldsCloserPoint) {
    Pixel2D left1{330.0, 240.0};
    Pixel2D right1{310.0, 240.0};

    Pixel2D left2{340.0, 240.0};
    Pixel2D right2{300.0, 240.0};

    auto result1 = matcher_->triangulate(left1, right1);
    auto result2 = matcher_->triangulate(left2, right2);

    ASSERT_TRUE(result1.has_value());
    ASSERT_TRUE(result2.has_value());

    EXPECT_LT(result2->z, result1->z);
}

TEST_F(StereoMatcherTest, NegativeDisparityRejected) {
    Pixel2D left{310.0, 240.0};
    Pixel2D right{330.0, 240.0};

    auto result = matcher_->triangulate(left, right);
    EXPECT_FALSE(result.has_value());
}

TEST_F(StereoMatcherTest, PointAtPrincipalPointReturnsCorrectZ) {
    Pixel2D left{325.0, 240.0};
    Pixel2D right{315.0, 240.0};

    auto result = matcher_->triangulate(left, right);
    ASSERT_TRUE(result.has_value());

    double expected_z = (800.0 * 0.12) / 10.0;
    EXPECT_NEAR(result->z, expected_z, 0.01);
}

TEST_F(StereoMatcherTest, VerticalDisparityIsIgnored) {
    Pixel2D left{330.0, 250.0};
    Pixel2D right{310.0, 230.0};

    auto result = matcher_->triangulate(left, right);
    ASSERT_TRUE(result.has_value());
    EXPECT_GT(result->z, 0.0);
}
