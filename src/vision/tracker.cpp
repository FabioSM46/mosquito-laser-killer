#include "vision/tracker.h"
#include "core/print.h"
#include <chrono>

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

auto KalmanTracker::predict(std::chrono::steady_clock::time_point timestamp)
    -> std::optional<Point3D> {
    if (!initialized_) {
        return std::nullopt;
    }

    double dt = 0.0;
    if (last_update_.time_since_epoch().count() > 0) {
        dt = std::chrono::duration<double>(timestamp - last_update_).count();
    }

    F_(0, 3) = dt;
    F_(1, 4) = dt;
    F_(2, 5) = dt;

    x_ = F_ * x_;
    P_ = F_ * P_ * F_.transpose() + Q_;

    return Point3D{x_(0), x_(1), x_(2)};
}

auto KalmanTracker::update(const Point3D& measurement,
                            std::chrono::steady_clock::time_point timestamp)
    -> std::optional<Point3D> {
    if (!initialized_) {
        x_.head<3>() = measurement.as_eigen();
        x_.tail<3>().setZero();
        last_update_ = timestamp;
        initialized_ = true;
        return measurement;
    }

    auto predicted = predict(timestamp);
    if (!predicted.has_value()) {
        return std::nullopt;
    }

    Eigen::Vector3d z(measurement.x, measurement.y, measurement.z);
    Eigen::Vector3d y = z - H_ * x_;

    Eigen::Matrix<double, 3, 3> S = H_ * P_ * H_.transpose() + R_;
    Eigen::Matrix<double, 6, 3> K = P_ * H_.transpose() * S.inverse();

    x_ = x_ + K * y;
    P_ = (Eigen::Matrix<double, 6, 6>::Identity() - K * H_) * P_;

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
