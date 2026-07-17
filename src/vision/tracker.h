#pragma once

#include "core/types.h"
#include <optional>
#include <Eigen/Dense>

class KalmanTracker {
public:
    // A track older than this is stale; extrapolating across a gap that long
    // would aim the beam wherever the last velocity estimate happens to point.
    static constexpr double k_max_predict_horizon_s{0.100};

    KalmanTracker();
    ~KalmanTracker() = default;

    [[nodiscard]] auto update(const Point3D& measurement,
                               std::chrono::steady_clock::time_point timestamp)
        -> std::optional<Point3D>;

    // Pure query: does NOT advance the filter. Calling it repeatedly for the same
    // or increasing timestamps is safe and returns a consistent extrapolation
    // from the last correction.
    [[nodiscard]] auto predict(std::chrono::steady_clock::time_point timestamp) const
        -> std::optional<Point3D>;

    [[nodiscard]] auto has_lock() const -> bool;

    // Exposed so the filter's convergence can actually be asserted in tests.
    [[nodiscard]] auto covariance() const -> const Eigen::Matrix<double, 6, 6>& {
        return P_;
    }
    [[nodiscard]] auto velocity() const -> Point3D {
        return Point3D{x_(3), x_(4), x_(5)};
    }

    void reset();

private:
    auto seed(const Point3D& measurement,
              std::chrono::steady_clock::time_point timestamp) -> void;

    Eigen::Matrix<double, 6, 6> P_;
    Eigen::Matrix<double, 3, 6> H_;
    Eigen::Matrix<double, 6, 6> Q_;
    Eigen::Matrix<double, 3, 3> R_;

    Eigen::Matrix<double, 6, 1> x_;
    bool initialized_{false};
    std::chrono::steady_clock::time_point last_update_{};

    static constexpr int k_state_dim{6};
    static constexpr int k_meas_dim{3};
};
