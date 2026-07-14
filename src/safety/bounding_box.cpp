#include "safety/bounding_box.h"

BoundingBox3D::BoundingBox3D(const SystemConfig::BoundingBox& config)
    : x_min_(config.x_min)
    , x_max_(config.x_max)
    , y_min_(config.y_min)
    , y_max_(config.y_max)
    , z_min_(config.z_min)
    , z_max_(config.z_max) {
}

auto BoundingBox3D::contains(const Point3D& point) const -> bool {
    return point.x >= x_min_ && point.x <= x_max_ &&
           point.y >= y_min_ && point.y <= y_max_ &&
           point.z >= z_min_ && point.z <= z_max_;
}

auto BoundingBox3D::is_inside(const Eigen::Vector3d& point) const -> bool {
    return point.x() >= x_min_ && point.x() <= x_max_ &&
           point.y() >= y_min_ && point.y() <= y_max_ &&
           point.z() >= z_min_ && point.z() <= z_max_;
}
