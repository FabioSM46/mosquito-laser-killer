#pragma once

#include "core/types.h"
#include <optional>
#include <Eigen/Dense>

class KalmanTracker {
public:
    KalmanTracker();
    ~KalmanTracker() = default;

    [[nodiscard]] auto update(const Point3D& measurement,
                               std::chrono::steady_clock::time_point timestamp)
        -> std::optional<Point3D>;

    [[nodiscard]] auto predict(std::chrono::steady_clock::time_point timestamp)
        -> std::optional<Point3D>;

    [[nodiscard]] auto has_lock() const -> bool;

    void reset();

private:
    Eigen::Matrix<double, 6, 6> P_;
    Eigen::Matrix<double, 6, 6> F_;
    Eigen::Matrix<double, 3, 6> H_;
    Eigen::Matrix<double, 6, 6> Q_;
    Eigen::Matrix<double, 3, 3> R_;

    Eigen::Matrix<double, 6, 1> x_;
    bool initialized_{false};
    std::chrono::steady_clock::time_point last_update_{};

    static constexpr int k_state_dim{6};
    static constexpr int k_meas_dim{3};
};
