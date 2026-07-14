#include <gtest/gtest.h>
#include <chrono>
#include <cmath>

#include "core/types.h"
#include "vision/tracker.h"

using namespace testing;
using namespace std::chrono_literals;

class KalmanTrackerTest : public Test {
protected:
    void SetUp() override {
        tracker_ = std::make_unique<KalmanTracker>();
    }

    std::unique_ptr<KalmanTracker> tracker_;
};

TEST_F(KalmanTrackerTest, InitialStateHasNoLock) {
    EXPECT_FALSE(tracker_->has_lock());
}

TEST_F(KalmanTrackerTest, FirstUpdateReturnsMeasurement) {
    auto t0 = std::chrono::steady_clock::now();
    auto result = tracker_->update({1.0, 0.0, 2.0}, t0);

    ASSERT_TRUE(result.has_value());
    EXPECT_NEAR(result->x, 1.0, 0.01);
    EXPECT_NEAR(result->y, 0.0, 0.01);
    EXPECT_NEAR(result->z, 2.0, 0.01);
}

TEST_F(KalmanTrackerTest, FirstUpdateEstablishesLock) {
    auto t0 = std::chrono::steady_clock::now();
    (void)tracker_->update({0.5, 0.5, 1.5}, t0);

    EXPECT_TRUE(tracker_->has_lock());
}

TEST_F(KalmanTrackerTest, MultipleUpdatesConverge) {
    auto t0 = std::chrono::steady_clock::now();

    Point3D target{1.0, 0.5, 3.0};

    (void)tracker_->update(target, t0);

    for (int i = 1; i <= 20; ++i) {
        Point3D measurement{
            target.x,
            target.y,
            target.z
        };
        auto result = tracker_->update(measurement, t0 + i * 8ms);
        ASSERT_TRUE(result.has_value());

        EXPECT_NEAR(result->x, target.x, 0.5);
        EXPECT_NEAR(result->y, target.y, 0.5);
        EXPECT_NEAR(result->z, target.z, 0.5);
    }
}

TEST_F(KalmanTrackerTest, PredictWithoutUpdateReturnsNullopt) {
    auto t0 = std::chrono::steady_clock::now();
    auto result = tracker_->predict(t0);
    EXPECT_FALSE(result.has_value());
}

TEST_F(KalmanTrackerTest, PredictAfterUpdateProjectsPosition) {
    auto t0 = std::chrono::steady_clock::now();
    (void)tracker_->update({1.0, 0.0, 2.0}, t0);

    auto result = tracker_->predict(t0 + 8ms);
    ASSERT_TRUE(result.has_value());
}

TEST_F(KalmanTrackerTest, ResetClearsState) {
    auto t0 = std::chrono::steady_clock::now();
    (void)tracker_->update({1.0, 0.0, 2.0}, t0);
    EXPECT_TRUE(tracker_->has_lock());

    tracker_->reset();
    EXPECT_FALSE(tracker_->has_lock());
}

TEST_F(KalmanTrackerTest, TracksMovingTarget) {
    auto t0 = std::chrono::steady_clock::now();

    for (int i = 0; i < 30; ++i) {
        double t = i * 0.008;
        Point3D measurement{1.0 + 0.5 * t, 0.0, 2.0 + 1.0 * t};

        auto result = tracker_->update(measurement, t0 + i * 8ms);
        ASSERT_TRUE(result.has_value());
    }

    auto predict = tracker_->predict(t0 + 31 * 8ms);
    ASSERT_TRUE(predict.has_value());

    EXPECT_GT(predict->x, 1.0);
    EXPECT_GT(predict->z, 2.0);
}
