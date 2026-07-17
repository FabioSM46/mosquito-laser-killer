#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <random>

#include "core/types.h"
#include "vision/tracker.h"

using namespace testing;
using namespace std::chrono_literals;

namespace {

// Every timestamp below is derived from this one fixed point by explicit
// arithmetic. Nothing here reads a real clock or sleeps, so the filter sees the
// same dt sequence on an idle laptop and on a loaded CI box.
const auto k_t0 = std::chrono::steady_clock::time_point{} + 1s;

// A 120 fps frame interval, comfortably inside the 100 ms predict horizon.
constexpr double k_dt_s = 0.008;
constexpr auto k_dt = 8ms;

// Truth for the moving-target tests: a mosquito crossing the engagement volume
// left to right at a constant 0.4 m/s, 0.75 m out.
constexpr double k_x0 = -0.06;
constexpr double k_vx = 0.4;
constexpr double k_z = 0.75;

auto truth_at(double t_s) -> Point3D {
    return Point3D{k_x0 + k_vx * t_s, 0.0, k_z};
}

// Feeds `samples` noise-free measurements of the constant-velocity track, one
// every k_dt, starting at k_t0.
void feed_constant_velocity_track(KalmanTracker& tracker, int samples) {
    for (int i = 0; i < samples; ++i) {
        const auto corrected = tracker.update(truth_at(i * k_dt_s), k_t0 + i * k_dt);
        ASSERT_TRUE(corrected.has_value()) << "update rejected sample " << i;
    }
}

// Deterministic pseudo-noise. std::mt19937's output sequence is fixed by the
// standard, but the distribution templates are not — their algorithms are
// unspecified and differ between standard libraries. Mapping the raw output by
// hand keeps this test reproducible everywhere it is built.
class UniformNoise {
public:
    explicit UniformNoise(uint32_t seed) : gen_(seed) {}

    auto next(double amplitude) -> double {
        const double unit = static_cast<double>(gen_()) /
                            static_cast<double>(std::mt19937::max());
        return (unit * 2.0 - 1.0) * amplitude;
    }

private:
    std::mt19937 gen_;
};

class KalmanTrackerTest : public Test {
protected:
    void SetUp() override { tracker_ = std::make_unique<KalmanTracker>(); }

    std::unique_ptr<KalmanTracker> tracker_;
};

TEST_F(KalmanTrackerTest, InitialStateHasNoLock) {
    EXPECT_FALSE(tracker_->has_lock());
}

TEST_F(KalmanTrackerTest, PredictBeforeAnyUpdateReturnsNullopt) {
    EXPECT_FALSE(tracker_->predict(k_t0).has_value());
    EXPECT_FALSE(tracker_->predict(k_t0 + 8ms).has_value());
}

TEST_F(KalmanTrackerTest, FirstUpdateSeedsWithTheMeasurementVerbatim) {
    const Point3D measurement{0.02, -0.01, 0.75};
    const auto seeded = tracker_->update(measurement, k_t0);

    ASSERT_TRUE(seeded.has_value());
    // Verbatim, not merely close: with no prior there is nothing to blend
    // against, and a first estimate that had drifted off the only measurement
    // available would mean the filter invented the difference.
    EXPECT_DOUBLE_EQ(seeded->x, measurement.x);
    EXPECT_DOUBLE_EQ(seeded->y, measurement.y);
    EXPECT_DOUBLE_EQ(seeded->z, measurement.z);

    EXPECT_TRUE(tracker_->has_lock());
    // A seeded track has no velocity evidence yet, and must not pretend to.
    EXPECT_DOUBLE_EQ(tracker_->velocity().x, 0.0);
    EXPECT_DOUBLE_EQ(tracker_->velocity().y, 0.0);
    EXPECT_DOUBLE_EQ(tracker_->velocity().z, 0.0);
}

// predict() is a question, not a command.
//
// The old predict() advanced x_ and P_ by dt but left last_update_ alone, so the
// next call computed the same dt again and applied it to the state the previous
// call had already moved. Asking the same question twice got two different
// answers, each further downrange than the last, and the aim point depended on
// how many times the control loop happened to interrogate the filter — a number
// with no physical meaning whatsoever.
//
// The track has to be moving for this to have any teeth: the damage the old code
// did was proportional to the estimated velocity, so against a freshly seeded,
// stationary track it looked perfectly well behaved.
TEST_F(KalmanTrackerTest, PredictIsPureAndRepeatable) {
    feed_constant_velocity_track(*tracker_, 40);
    ASSERT_NEAR(tracker_->velocity().x, k_vx, 0.01) << "track must be moving";

    const auto t_query = k_t0 + 39 * k_dt + 50ms;

    const auto first = tracker_->predict(t_query);
    const auto second = tracker_->predict(t_query);
    const auto third = tracker_->predict(t_query);

    ASSERT_TRUE(first.has_value());
    ASSERT_TRUE(second.has_value());
    ASSERT_TRUE(third.has_value());

    EXPECT_DOUBLE_EQ(second->x, first->x);
    EXPECT_DOUBLE_EQ(second->y, first->y);
    EXPECT_DOUBLE_EQ(second->z, first->z);
    EXPECT_DOUBLE_EQ(third->x, first->x);
    EXPECT_DOUBLE_EQ(third->y, first->y);
    EXPECT_DOUBLE_EQ(third->z, first->z);
}

// The other half of the same bug: because the old predict() mutated the filter,
// a predict() between two update()s corrupted the second update. The control
// loop predicts on every frame, so this was not a hypothetical path — it was the
// normal one.
//
// Two trackers, identical measurement streams; the only difference is that one
// of them is also being asked to predict. Any divergence at all means the
// filter's output depends on caller behaviour rather than on the measurements.
TEST_F(KalmanTrackerTest, PredictDoesNotDisturbSubsequentUpdates) {
    KalmanTracker with_predicts;
    KalmanTracker without_predicts;

    for (int i = 0; i < 40; ++i) {
        const Point3D measurement = truth_at(i * k_dt_s);
        const auto timestamp = k_t0 + i * k_dt;

        const auto polled = with_predicts.update(measurement, timestamp);
        const auto quiet = without_predicts.update(measurement, timestamp);

        ASSERT_TRUE(polled.has_value());
        ASSERT_TRUE(quiet.has_value());

        EXPECT_DOUBLE_EQ(polled->x, quiet->x) << "diverged at update " << i;
        EXPECT_DOUBLE_EQ(polled->y, quiet->y) << "diverged at update " << i;
        EXPECT_DOUBLE_EQ(polled->z, quiet->z) << "diverged at update " << i;

        for (int poll = 0; poll < 3; ++poll) {
            (void)with_predicts.predict(timestamp + 30ms);
        }
    }

    // The covariance must be untouched too, not just the state: P_ sets the
    // Kalman gain, so a P_ the predicts had inflated would change how much every
    // later measurement is trusted.
    EXPECT_DOUBLE_EQ(with_predicts.covariance().trace(),
                     without_predicts.covariance().trace());
}

// The old convergence test fed the filter the exact target value on every
// iteration and then asserted the estimate was within +/-0.5 of a target at
// x=1.0. The filter was seeded with the answer and then handed the answer
// twenty more times, so it could not have failed: an implementation whose entire
// body was `return measurement;` passed it, and so would one that returned a
// constant 1.0. It measured nothing.
//
// The only way to show a filter is filtering is to give it something to filter.
// Feed a noisy stream around a known truth and require the estimate to be
// materially closer to truth than the measurements it was built from — a bar
// that `return measurement;` fails by construction, since it would score exactly
// the measurement error.
TEST_F(KalmanTrackerTest, EstimateConvergesCloserToTruthThanTheNoisyMeasurements) {
    constexpr double k_noise_amplitude = 0.015;   // +/-15 mm
    constexpr int k_samples = 120;
    constexpr int k_tail = 40;   // score only after the filter has settled

    const Point3D truth{0.02, -0.01, 0.75};
    UniformNoise noise(12345u);

    double filtered_error = 0.0;
    double measured_error = 0.0;
    int scored = 0;

    for (int i = 0; i < k_samples; ++i) {
        const double nx = noise.next(k_noise_amplitude);
        const double ny = noise.next(k_noise_amplitude);
        const double nz = noise.next(k_noise_amplitude);
        const Point3D measurement{truth.x + nx, truth.y + ny, truth.z + nz};

        const auto estimate = tracker_->update(measurement, k_t0 + i * k_dt);
        ASSERT_TRUE(estimate.has_value()) << "update rejected sample " << i;

        if (i >= k_samples - k_tail) {
            filtered_error += std::abs(estimate->x - truth.x) +
                              std::abs(estimate->y - truth.y) +
                              std::abs(estimate->z - truth.z);
            measured_error += std::abs(measurement.x - truth.x) +
                              std::abs(measurement.y - truth.y) +
                              std::abs(measurement.z - truth.z);
            ++scored;
        }
    }

    ASSERT_EQ(scored, k_tail);
    const double filtered_mae = filtered_error / (3.0 * scored);
    const double measured_mae = measured_error / (3.0 * scored);

    // Both streams are scored against the same truth over the same samples, so
    // this compares the filter against the exact noise it was given.
    EXPECT_LT(filtered_mae, 0.5 * measured_mae)
        << "filtered MAE " << filtered_mae << " vs raw measurement MAE "
        << measured_mae;
    EXPECT_LT(filtered_mae, 0.005);
}

// P_ was private with no accessor, so the filter's actual convergence — the
// thing that decides how much each new measurement is trusted — could not be
// asserted at all. Everything about it had to be taken on faith.
TEST_F(KalmanTrackerTest, CovarianceShrinksAsMeasurementsArrive) {
    const Point3D truth{0.02, -0.01, 0.75};

    const auto seeded = tracker_->update(truth, k_t0);
    ASSERT_TRUE(seeded.has_value());

    // A freshly seeded track knows nothing: 1000 on each of six states.
    EXPECT_DOUBLE_EQ(tracker_->covariance().trace(), 6000.0);

    double previous = tracker_->covariance().trace();
    for (int i = 1; i <= 20; ++i) {
        const auto corrected = tracker_->update(truth, k_t0 + i * k_dt);
        ASSERT_TRUE(corrected.has_value());

        const double current = tracker_->covariance().trace();
        EXPECT_LT(current, previous)
            << "uncertainty grew at update " << i << " despite a measurement";
        previous = current;
    }

    // Twenty corrections take the trace from 6000 to roughly 34, so the filter
    // has genuinely learned the state rather than merely avoided diverging.
    EXPECT_LT(previous, 100.0);
}

TEST_F(KalmanTrackerTest, ResetClearsLockAndRestoresInitialCovariance) {
    feed_constant_velocity_track(*tracker_, 10);
    ASSERT_TRUE(tracker_->has_lock());
    ASSERT_LT(tracker_->covariance().trace(), 6000.0);

    tracker_->reset();

    EXPECT_FALSE(tracker_->has_lock());
    EXPECT_FALSE(tracker_->predict(k_t0 + 10 * k_dt).has_value());
    EXPECT_DOUBLE_EQ(tracker_->covariance().trace(), 6000.0);
    EXPECT_TRUE(tracker_->covariance().isApprox(
        Eigen::Matrix<double, 6, 6>::Identity() * 1000.0));
    EXPECT_DOUBLE_EQ(tracker_->velocity().x, 0.0);
}

// The old test asserted predict()->x > 1.0 after feeding a track whose last
// measurement already sat at 1.116. Every value between 1.0 and 1.116 passed,
// which means a predict() that ignored velocity entirely and echoed back the
// last corrected position passed — and so would one that returned a constant
// 1.001. The assertion was anchored to the track's starting point instead of to
// where the track had got to, so it could not tell leading from lagging.
//
// Leading is the entire reason predict() exists: the beam must be sent where the
// mosquito will be by the time the galvos settle, not where it was. Anchor to
// the last measurement and require the whole lead distance.
TEST_F(KalmanTrackerTest, PredictLeadsTheTargetAheadOfTheLastMeasurement) {
    constexpr int k_samples = 40;
    feed_constant_velocity_track(*tracker_, k_samples);

    const double t_last = (k_samples - 1) * k_dt_s;
    const Point3D last_measurement = truth_at(t_last);

    // The filter recovered the velocity it was never told about.
    EXPECT_NEAR(tracker_->velocity().x, k_vx, 0.01);
    EXPECT_NEAR(tracker_->velocity().y, 0.0, 0.01);
    EXPECT_NEAR(tracker_->velocity().z, 0.0, 0.01);

    constexpr double k_lead_s = 0.050;
    const auto predicted = tracker_->predict(k_t0 + (k_samples - 1) * k_dt + 50ms);
    ASSERT_TRUE(predicted.has_value());

    // Strictly ahead of where the target actually was, in the direction it is
    // travelling, by most of the full lead distance (0.4 m/s * 50 ms = 20 mm).
    const double expected_lead = k_vx * k_lead_s;
    EXPECT_GT(predicted->x, last_measurement.x + 0.75 * expected_lead)
        << "predicted x=" << predicted->x
        << " is not leading the last measurement at x=" << last_measurement.x;

    // And ahead in the right place, not merely somewhere further along.
    EXPECT_NEAR(predicted->x, truth_at(t_last + k_lead_s).x, 0.002);
    EXPECT_NEAR(predicted->y, last_measurement.y, 0.002);
    EXPECT_NEAR(predicted->z, last_measurement.z, 0.002);
}

TEST_F(KalmanTrackerTest, PredictBeyondHorizonReturnsNullopt) {
    feed_constant_velocity_track(*tracker_, 10);
    const auto t_last = k_t0 + 9 * k_dt;

    const auto horizon = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(KalmanTracker::k_max_predict_horizon_s));

    // Inside the horizon the filter still answers.
    EXPECT_TRUE(tracker_->predict(t_last + horizon - 1ms).has_value());

    // Past it, the velocity estimate is the only thing left driving the answer,
    // and it is describing a target that has not been seen for a tenth of a
    // second. This target moves 40 mm in that time, and a mosquito that changed
    // direction moved somewhere else entirely; either way the extrapolation is
    // fiction. Refuse to answer rather than aim a Class 4 beam from a stale
    // velocity at a place nothing has confirmed.
    EXPECT_FALSE(tracker_->predict(t_last + horizon + 1ms).has_value());
    EXPECT_FALSE(tracker_->predict(t_last + 500ms).has_value());
}

TEST_F(KalmanTrackerTest, PredictWithTimestampBeforeLastUpdateReturnsNullopt) {
    feed_constant_velocity_track(*tracker_, 10);
    const auto t_last = k_t0 + 9 * k_dt;

    // dt == 0 is a legitimate query: "where is it right now".
    EXPECT_TRUE(tracker_->predict(t_last).has_value());

    // A negative dt means the caller's timestamps are not monotonic. Running the
    // state transition backwards would produce a confident answer built on a
    // caller bug, so there is nothing sensible to return.
    EXPECT_FALSE(tracker_->predict(t_last - 1ms).has_value());
    EXPECT_FALSE(tracker_->predict(t_last - 50ms).has_value());
}

TEST_F(KalmanTrackerTest, UpdateAfterGapReSeedsInsteadOfExtrapolating) {
    constexpr int k_samples = 40;
    feed_constant_velocity_track(*tracker_, k_samples);
    ASSERT_NEAR(tracker_->velocity().x, k_vx, 0.01);

    // 200 ms of silence, then a measurement somewhere else. Whatever is being
    // reported now is a new object; the tracked one is long gone. Propagating the
    // old state across the gap would drag it 80 mm along a heading that is no
    // longer evidence of anything and then blend the new measurement against it,
    // leaving the filter aiming at the gap between the two for several frames.
    const Point3D fresh{0.08, 0.05, 0.55};
    const auto result = tracker_->update(fresh, k_t0 + (k_samples - 1) * k_dt + 200ms);

    ASSERT_TRUE(result.has_value());
    EXPECT_DOUBLE_EQ(result->x, fresh.x);
    EXPECT_DOUBLE_EQ(result->y, fresh.y);
    EXPECT_DOUBLE_EQ(result->z, fresh.z);

    // A re-seed and not a correction: the stale velocity is discarded outright
    // and the covariance goes back to knowing nothing.
    EXPECT_DOUBLE_EQ(tracker_->velocity().x, 0.0);
    EXPECT_DOUBLE_EQ(tracker_->velocity().y, 0.0);
    EXPECT_DOUBLE_EQ(tracker_->velocity().z, 0.0);
    EXPECT_DOUBLE_EQ(tracker_->covariance().trace(), 6000.0);
    EXPECT_TRUE(tracker_->has_lock());
}

TEST_F(KalmanTrackerTest, NonFiniteMeasurementDoesNotSeedATrack) {
    constexpr double k_nan = std::numeric_limits<double>::quiet_NaN();
    constexpr double k_inf = std::numeric_limits<double>::infinity();

    // A non-finite measurement must not establish a lock. If it did, has_lock()
    // would report a track built on garbage, and the NaN would spread through x_
    // and P_ on the first correction — after which every output is NaN and every
    // comparison against it is silently false, including the bounding-box test.
    for (const double bad : {k_nan, k_inf, -k_inf}) {
        SCOPED_TRACE(Message() << "bad value: " << bad);

        KalmanTracker fresh;
        EXPECT_FALSE(fresh.update({bad, 0.0, 0.75}, k_t0).has_value());
        EXPECT_FALSE(fresh.update({0.02, bad, 0.75}, k_t0).has_value());
        EXPECT_FALSE(fresh.update({0.02, 0.0, bad}, k_t0).has_value());
        EXPECT_FALSE(fresh.has_lock());
    }
}

TEST_F(KalmanTrackerTest, NonFiniteMeasurementLeavesAnEstablishedTrackIntact) {
    constexpr double k_nan = std::numeric_limits<double>::quiet_NaN();
    feed_constant_velocity_track(*tracker_, 20);

    const auto t_query = k_t0 + 19 * k_dt + 20ms;
    const auto before = tracker_->predict(t_query);
    ASSERT_TRUE(before.has_value());

    EXPECT_FALSE(tracker_->update({k_nan, 0.0, k_z}, k_t0 + 20 * k_dt).has_value());

    // Rejected before the state was touched: one corrupt frame must cost the
    // system nothing more than that frame.
    EXPECT_TRUE(tracker_->has_lock());
    const auto after = tracker_->predict(t_query);
    ASSERT_TRUE(after.has_value());
    EXPECT_DOUBLE_EQ(after->x, before->x);
    EXPECT_DOUBLE_EQ(after->y, before->y);
    EXPECT_DOUBLE_EQ(after->z, before->z);
}

}   // namespace
