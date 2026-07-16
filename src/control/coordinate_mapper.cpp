#include "control/coordinate_mapper.h"
#include <cmath>
#include "core/print.h"

CoordinateMapper::CoordinateMapper(const BoundingBox3D& bounding_box,
                                   const SystemConfig::GalvoLimits& galvo_limits,
                                   double dac_ref_voltage,
                                   const SystemConfig::GalvoDriver& galvo_driver)
    : bounding_box_(bounding_box)
    , angle_x_min_(galvo_limits.angle_x_min_deg)
    , angle_x_max_(galvo_limits.angle_x_max_deg)
    , angle_y_min_(galvo_limits.angle_y_min_deg)
    , angle_y_max_(galvo_limits.angle_y_max_deg)
    , dac_ref_voltage_(dac_ref_voltage)
    , input_scale_v_per_deg_(galvo_driver.input_scale_v_per_deg)
    , dac_max_diff_voltage_(galvo_driver.dac_max_diff_voltage) {
    println("[MAPPER] Initialized, bounding box: x=[{:.2f},{:.2f}] y=[{:.2f},{:.2f}] "
                 "z=[{:.2f},{:.2f}]", bounding_box_.x_min(), bounding_box_.x_max(),
                 bounding_box_.y_min(), bounding_box_.y_max(),
                 bounding_box_.z_min(), bounding_box_.z_max());
    println("[MAPPER] Galvo limits: x=[{:.1f},{:.1f}]deg y=[{:.1f},{:.1f}]deg",
                 angle_x_min_, angle_x_max_, angle_y_min_, angle_y_max_);
    println("[MAPPER] DAC ref={:.1f}V, scale={:.3f}V/deg, max_diff={:.1f}V "
                 "(center DAC={})",
                 dac_ref_voltage_, input_scale_v_per_deg_, dac_max_diff_voltage_,
                 dac_center_);
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
    if (p.z <= 0.0) {
        return std::unexpected(MappingError::Invalid3DPoint);
    }

    double angle_x = std::atan2(p.x, p.z) * (180.0 / M_PI);
    double angle_y = std::atan2(p.y, p.z) * (180.0 / M_PI);

    return std::pair{angle_x, angle_y};
}

auto CoordinateMapper::angle_to_dac_code(double angle_deg) const
    -> std::expected<uint16_t, MappingError> {
    if (input_scale_v_per_deg_ <= 0.0 || dac_ref_voltage_ <= 0.0) {
        return std::unexpected(MappingError::ConversionError);
    }

    // V_diff = θ · scale; complementary DAC: V_diff = (2c/4095 − 1) · Vref
    // ⇒ c = (V_diff / Vref + 1) · 4095 / 2
    const double v_diff = angle_deg * input_scale_v_per_deg_;

    if (std::abs(v_diff) > dac_max_diff_voltage_ + 1e-9) {
        println(stderr, "[MAPPER] Differential voltage {:.3f}V exceeds max {:.3f}V",
                     v_diff, dac_max_diff_voltage_);
        return std::unexpected(MappingError::DacRangeInvalid);
    }

    const double normalized = (v_diff / dac_ref_voltage_) + 1.0;
    const double code = normalized * (dac_max_value_ / 2.0);
    const int dac = static_cast<int>(std::lround(code));

    if (dac < 0 || dac > 4095) {
        println(stderr, "[MAPPER] DAC code {} out of 0–4095 range", dac);
        return std::unexpected(MappingError::DacRangeInvalid);
    }

    return static_cast<uint16_t>(dac);
}

auto CoordinateMapper::angles_to_dac(double angle_x, double angle_y)
    -> std::expected<DacValues, MappingError> {
    auto dac_x = angle_to_dac_code(angle_x);
    if (!dac_x.has_value()) {
        return std::unexpected(dac_x.error());
    }

    auto dac_y = angle_to_dac_code(angle_y);
    if (!dac_y.has_value()) {
        return std::unexpected(dac_y.error());
    }

    return DacValues{dac_x.value(), dac_y.value()};
}
