#include "vision/tracker.h"
#include "core/print.h"
#include <chrono>
#include <cmath>

KalmanTracker::KalmanTracker() {
    P_ = Eigen::Matrix<double, 6, 6>::Identity() * 1000.0;

    F_ = Eigen::Matrix<double, 6, 6>::Identity();

    H_ = Eigen::Matrix<double, 3, 6>::Zero();
    H_(0, 0) = 1.0;
    H_(1, 1) = 1.0;
    H_(2, 2) = 1.0;

    Q_ = Eigen::Matrix<double, 6, 6>::Identity() * 0.01;
    R_ = Eigen::Matrix<double, 3, 3>::Identity() * 0.1;

    x_ = Eigen::Matrix<double, 6, 1>::Zero();

    println("[TRACKER] Kalman filter initialized, {}D state, {}D measurement",
                 k_state_dim, k_meas_dim);
}

auto KalmanTracker::seed(const Point3D& measurement,
                         std::chrono::steady_clock::time_point timestamp) -> void {
    x_.head<3>() = measurement.as_eigen();
    x_.tail<3>().setZero();
    P_ = Eigen::Matrix<double, 6, 6>::Identity() * 1000.0;
    last_update_ = timestamp;
    initialized_ = true;
}

auto KalmanTracker::predict(std::chrono::steady_clock::time_point timestamp) const
    -> std::optional<Point3D> {
    if (!initialized_) {
        return std::nullopt;
    }

    const double dt = std::chrono::duration<double>(timestamp - last_update_).count();

    // Reject a negative dt (non-monotonic caller) and a stale track. Phrased so
    // NaN falls through to the reject branch.
    if (!(dt >= 0.0 && dt <= k_max_predict_horizon_s)) {
        return std::nullopt;
    }

    Eigen::Matrix<double, 6, 6> f = Eigen::Matrix<double, 6, 6>::Identity();
    f(0, 3) = dt;
    f(1, 4) = dt;
    f(2, 5) = dt;

    const Eigen::Matrix<double, 6, 1> predicted = f * x_;
    if (!predicted.allFinite()) {
        return std::nullopt;
    }

    return Point3D{predicted(0), predicted(1), predicted(2)};
}

auto KalmanTracker::update(const Point3D& measurement,
                            std::chrono::steady_clock::time_point timestamp)
    -> std::optional<Point3D> {
    if (!std::isfinite(measurement.x) || !std::isfinite(measurement.y) ||
        !std::isfinite(measurement.z)) {
        return std::nullopt;
    }

    if (!initialized_) {
        seed(measurement, timestamp);
        return measurement;
    }

    const double dt = std::chrono::duration<double>(timestamp - last_update_).count();

    // A gap this large means the previous track cannot inform this measurement.
    // Re-seed from the measurement instead of extrapolating across the gap.
    if (!(dt >= 0.0 && dt <= k_max_predict_horizon_s)) {
        seed(measurement, timestamp);
        return measurement;
    }

    F_(0, 3) = dt;
    F_(1, 4) = dt;
    F_(2, 5) = dt;

    x_ = F_ * x_;
    P_ = F_ * P_ * F_.transpose() + Q_;

    const Eigen::Vector3d z(measurement.x, measurement.y, measurement.z);
    const Eigen::Vector3d y = z - H_ * x_;

    const Eigen::Matrix<double, 3, 3> S = H_ * P_ * H_.transpose() + R_;
    const Eigen::Matrix<double, 6, 3> K = P_ * H_.transpose() * S.inverse();

    x_ = x_ + K * y;
    P_ = (Eigen::Matrix<double, 6, 6>::Identity() - K * H_) * P_;

    if (!x_.allFinite() || !P_.allFinite()) {
        println(stderr, "[TRACKER] Non-finite state after update, resetting");
        reset();
        return std::nullopt;
    }

    last_update_ = timestamp;

    return Point3D{x_(0), x_(1), x_(2)};
}

auto KalmanTracker::has_lock() const -> bool {
    return initialized_;
}

void KalmanTracker::reset() {
    initialized_ = false;
    x_.setZero();
    P_ = Eigen::Matrix<double, 6, 6>::Identity() * 1000.0;
    println("[TRACKER] Reset");
}
