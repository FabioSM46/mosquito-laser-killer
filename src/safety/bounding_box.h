#pragma once

#include "core/types.h"
#include <Eigen/Dense>

class BoundingBox3D {
public:
    BoundingBox3D(const SystemConfig::BoundingBox& config);

    [[nodiscard]] auto contains(const Point3D& point) const -> bool;
    [[nodiscard]] auto is_inside(const Eigen::Vector3d& point) const -> bool;

    [[nodiscard]] auto x_min() const -> double { return x_min_; }
    [[nodiscard]] auto x_max() const -> double { return x_max_; }
    [[nodiscard]] auto y_min() const -> double { return y_min_; }
    [[nodiscard]] auto y_max() const -> double { return y_max_; }
    [[nodiscard]] auto z_min() const -> double { return z_min_; }
    [[nodiscard]] auto z_max() const -> double { return z_max_; }

private:
    double x_min_, x_max_;
    double y_min_, y_max_;
    double z_min_, z_max_;
};
