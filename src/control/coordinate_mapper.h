#pragma once

#include "core/types.h"
#include "core/error.h"
#include "safety/bounding_box.h"
#include <expected>
#include <utility>
#include <memory>

class CoordinateMapper {
public:
    CoordinateMapper(const BoundingBox3D& bounding_box,
                     const SystemConfig::GalvoLimits& galvo_limits);
    ~CoordinateMapper() = default;

    [[nodiscard]] auto map_to_dac(const Point3D& target)
        -> std::expected<DacValues, MappingError>;

private:
    const BoundingBox3D& bounding_box_;
    double angle_x_min_, angle_x_max_;
    double angle_y_min_, angle_y_max_;

    static constexpr double dac_ref_voltage_{5.0};
    static constexpr double dac_max_value_{4095.0};

    [[nodiscard]] auto point_to_angles(const Point3D& p)
        -> std::expected<std::pair<double, double>, MappingError>;

    [[nodiscard]] auto angles_to_dac(double angle_x, double angle_y)
        -> std::expected<DacValues, MappingError>;
};
