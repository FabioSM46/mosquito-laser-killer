#pragma once

#include "core/types.h"
#include "core/error.h"
#include "safety/bounding_box.h"
#include <expected>
#include <utility>

class CoordinateMapper {
public:
    CoordinateMapper(const BoundingBox3D& bounding_box,
                     const SystemConfig::GalvoLimits& galvo_limits,
                     double dac_ref_voltage = 5.0,
                     const SystemConfig::GalvoDriver& galvo_driver = {});
    ~CoordinateMapper() = default;

    [[nodiscard]] auto map_to_dac(const Point3D& target)
        -> std::expected<DacValues, MappingError>;

private:
    const BoundingBox3D& bounding_box_;
    double angle_x_min_, angle_x_max_;
    double angle_y_min_, angle_y_max_;
    double dac_ref_voltage_;
    double input_scale_v_per_deg_;
    double dac_max_diff_voltage_;
    static constexpr double dac_max_value_{4095.0};
    static constexpr uint16_t dac_center_{2048};

    [[nodiscard]] auto point_to_angles(const Point3D& p)
        -> std::expected<std::pair<double, double>, MappingError>;

    [[nodiscard]] auto angles_to_dac(double angle_x, double angle_y)
        -> std::expected<DacValues, MappingError>;

    [[nodiscard]] auto angle_to_dac_code(double angle_deg) const
        -> std::expected<uint16_t, MappingError>;
};
