#include "control/coordinate_mapper.h"
#include <cmath>
#include "core/print.h"

CoordinateMapper::CoordinateMapper(const BoundingBox3D& bounding_box,
                                   const SystemConfig::GalvoLimits& galvo_limits,
                                   double dac_ref_voltage)
    : bounding_box_(bounding_box)
    , angle_x_min_(galvo_limits.angle_x_min_deg)
    , angle_x_max_(galvo_limits.angle_x_max_deg)
    , angle_y_min_(galvo_limits.angle_y_min_deg)
    , angle_y_max_(galvo_limits.angle_y_max_deg)
    , dac_ref_voltage_(dac_ref_voltage) {
    println("[MAPPER] Initialized, bounding box: x=[{:.2f},{:.2f}] y=[{:.2f},{:.2f}] "
                 "z=[{:.2f},{:.2f}]", bounding_box_.x_min(), bounding_box_.x_max(),
                 bounding_box_.y_min(), bounding_box_.y_max(),
                 bounding_box_.z_min(), bounding_box_.z_max());
    println("[MAPPER] Galvo limits: x=[{:.1f},{:.1f}]deg y=[{:.1f},{:.1f}]deg",
                 angle_x_min_, angle_x_max_, angle_y_min_, angle_y_max_);
    println("[MAPPER] DAC ref voltage: {:.1f}V (midpoint={:.1f}V, "
                 "galvo input range: 0-{:.1f}V unipolar -> {:.1f}-{:.1f} deg)",
                 dac_ref_voltage_, dac_ref_voltage_ / 2.0,
                 dac_ref_voltage_, angle_x_min_, angle_x_max_);
}

auto CoordinateMapper::map_to_dac(const Point3D& target)
    -> std::expected<DacValues, MappingError> {
    if (!bounding_box_.contains(target)) {
        println(stderr, "[MAPPER] Target OUTSIDE bounding box: ({:.3f}, {:.3f}, {:.3f})",
                     target.x, target.y, target.z);
        return std::unexpected(MappingError::OutOfBounds);
    }

    auto angles = point_to_angles(target);
    if (!angles.has_value()) {
        return std::unexpected(angles.error());
    }

    auto [angle_x, angle_y] = angles.value();

    if (angle_x < angle_x_min_ || angle_x > angle_x_max_ ||
        angle_y < angle_y_min_ || angle_y > angle_y_max_) {
        println(stderr, "[MAPPER] Galvo angle limits exceeded: x={:.2f} y={:.2f}",
                     angle_x, angle_y);
        return std::unexpected(MappingError::GalvoAngleLimitExceeded);
    }

    return angles_to_dac(angle_x, angle_y);
}

auto CoordinateMapper::point_to_angles(const Point3D& p)
    -> std::expected<std::pair<double, double>, MappingError> {
    double angle_x = std::atan2(p.x, p.z) * (180.0 / M_PI);
    double angle_y = std::atan2(p.y, p.z) * (180.0 / M_PI);

    return std::pair{angle_x, angle_y};
}

auto CoordinateMapper::angles_to_dac(double angle_x, double angle_y)
    -> std::expected<DacValues, MappingError> {
    double range_x = angle_x_max_ - angle_x_min_;
    double range_y = angle_y_max_ - angle_y_min_;

    double normalized_x = (angle_x - angle_x_min_) / range_x;
    double normalized_y = (angle_y - angle_y_min_) / range_y;

    int dac_x = static_cast<int>(std::round(normalized_x * dac_max_value_));
    int dac_y = static_cast<int>(std::round(normalized_y * dac_max_value_));

    dac_x = std::clamp(dac_x, 0, 4095);
    dac_y = std::clamp(dac_y, 0, 4095);

    return DacValues{
        static_cast<uint16_t>(dac_x),
        static_cast<uint16_t>(dac_y)
    };
}
