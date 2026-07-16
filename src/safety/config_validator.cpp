#include "safety/config_validator.h"
#include <cmath>

namespace {

struct Corner {
    double x;
    double y;
    double z;
};

auto box_corners(const SystemConfig::BoundingBox& bb) -> std::array<Corner, 8> {
    return {{
        {bb.x_min, bb.y_min, bb.z_min},
        {bb.x_max, bb.y_min, bb.z_min},
        {bb.x_min, bb.y_max, bb.z_min},
        {bb.x_max, bb.y_max, bb.z_min},
        {bb.x_min, bb.y_min, bb.z_max},
        {bb.x_max, bb.y_min, bb.z_max},
        {bb.x_min, bb.y_max, bb.z_max},
        {bb.x_max, bb.y_max, bb.z_max},
    }};
}

}

auto horizontal_fov_deg(const SystemConfig::CameraOptics& optics) -> double {
    if (optics.lens_focal_length_mm <= 0.0) {
        return 0.0;
    }
    return 2.0 * std::atan(optics.image_sensor_width_mm / (2.0 * optics.lens_focal_length_mm))
           * (180.0 / M_PI);
}

auto vertical_fov_deg(const SystemConfig::CameraOptics& optics) -> double {
    if (optics.lens_focal_length_mm <= 0.0) {
        return 0.0;
    }
    return 2.0 * std::atan(optics.image_sensor_height_mm / (2.0 * optics.lens_focal_length_mm))
           * (180.0 / M_PI);
}

auto validate_engagement_volume(const SystemConfig& config)
    -> std::vector<ValidationWarning> {
    std::vector<ValidationWarning> warnings;

    const double galvo_half_cone_deg =
        std::min({config.galvo_limits.angle_x_max_deg, config.galvo_limits.angle_y_max_deg,
                  -config.galvo_limits.angle_x_min_deg, -config.galvo_limits.angle_y_min_deg});

    const double max_commandable_deg =
        config.galvo_driver.dac_max_diff_voltage /
        std::max(config.galvo_driver.input_scale_v_per_deg, 1e-9);

    if (galvo_half_cone_deg > max_commandable_deg) {
        warnings.push_back({
            "galvo-voltage",
            "Galvo half-cone (" + std::to_string(galvo_half_cone_deg) +
                " deg) exceeds the angle commandable by the DAC/driver chain (" +
                std::to_string(max_commandable_deg) + " deg = " +
                std::to_string(config.galvo_driver.dac_max_diff_voltage) + "V / " +
                std::to_string(config.galvo_driver.input_scale_v_per_deg) + "V/deg)."
        });
    }

    const double hfov = horizontal_fov_deg(config.camera_optics);
    const double half_hfov = hfov / 2.0;
    if (half_hfov < galvo_half_cone_deg) {
        warnings.push_back({
            "camera-fov",
            "Camera horizontal half-FOV (" + std::to_string(half_hfov) +
                " deg) is narrower than the galvo half-cone (" +
                std::to_string(galvo_half_cone_deg) +
                " deg). Targets reachable by the galvo may be outside the camera view."
        });
    }

    for (const auto& c : box_corners(config.bounding_box)) {
        if (c.z <= 0.0) {
            warnings.push_back({"bounding-box", "Bounding-box corner has non-positive z."});
            continue;
        }
        const double lateral = std::sqrt(c.x * c.x + c.y * c.y);
        const double angle_deg = std::atan2(lateral, c.z) * (180.0 / M_PI);
        if (angle_deg > galvo_half_cone_deg) {
            warnings.push_back({
                "bounding-box",
                "Bounding-box corner (" + std::to_string(c.x) + ", " +
                    std::to_string(c.y) + ", " + std::to_string(c.z) +
                    ") requires " + std::to_string(angle_deg) +
                    " deg, beyond the " + std::to_string(galvo_half_cone_deg) +
                    " deg galvo half-cone."
            });
        }
    }

    if (config.stereo.baseline_m <= 0.0) {
        warnings.push_back({"stereo", "Stereo baseline must be positive."});
    }
    if (config.stereo.focal_length_px <= 0.0) {
        warnings.push_back({"stereo", "Stereo focal_length_px must be positive."});
    }

    return warnings;
}
