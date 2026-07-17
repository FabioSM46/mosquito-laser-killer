#include "safety/config_validator.h"
#include <algorithm>
#include <array>
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

auto has_critical_validation_errors(const std::vector<ValidationWarning>& warnings) -> bool {
    for (const auto& w : warnings) {
        if (w.critical) {
            return true;
        }
    }
    return false;
}

auto validate_engagement_volume(const SystemConfig& config)
    -> std::vector<ValidationWarning> {
    std::vector<ValidationWarning> warnings;

    const double galvo_half_cone_deg =
        std::min({config.galvo_limits.angle_x_max_deg, config.galvo_limits.angle_y_max_deg,
                  -config.galvo_limits.angle_x_min_deg, -config.galvo_limits.angle_y_min_deg});

    // A NaN or inverted galvo range disables the cone check in map_to_dac
    // (`angle < min` / `angle > max` are both false for NaN bounds), so the
    // limits themselves must be proven finite and ordered here.
    const auto& gl = config.galvo_limits;
    if (!std::isfinite(gl.angle_x_min_deg) || !std::isfinite(gl.angle_x_max_deg) ||
        !std::isfinite(gl.angle_y_min_deg) || !std::isfinite(gl.angle_y_max_deg) ||
        !(gl.angle_x_min_deg < gl.angle_x_max_deg) ||
        !(gl.angle_y_min_deg < gl.angle_y_max_deg)) {
        warnings.push_back({
            "galvo-limits",
            "Galvo angle limits must be finite with min < max on both axes.",
            true
        });
    }

    if (!std::isfinite(galvo_half_cone_deg) || galvo_half_cone_deg <= 0.0) {
        warnings.push_back({
            "galvo-limits",
            "Galvo half-cone must be finite and positive.",
            true
        });
    }

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
                std::to_string(config.galvo_driver.input_scale_v_per_deg) + "V/deg).",
            true
        });
    }

    const double hfov = horizontal_fov_deg(config.camera_optics);
    const double half_hfov = hfov / 2.0;
    if (half_hfov < galvo_half_cone_deg) {
        // Informational: targets commandable by galvo may be outside camera FOV.
        warnings.push_back({
            "camera-fov",
            "Camera horizontal half-FOV (" + std::to_string(half_hfov) +
                " deg) is narrower than the galvo half-cone (" +
                std::to_string(galvo_half_cone_deg) +
                " deg). Targets reachable by the galvo may be outside the camera view.",
            false
        });
    }

    for (const auto& c : box_corners(config.bounding_box)) {
        // isfinite first: `c.z <= 0.0` and `angle_deg > cone` are both false for
        // NaN, so a NaN corner would otherwise sail through both checks.
        if (!std::isfinite(c.x) || !std::isfinite(c.y) || !std::isfinite(c.z)) {
            warnings.push_back({
                "bounding-box",
                "Bounding-box corner has a non-finite coordinate.",
                true
            });
            continue;
        }
        if (c.z <= 0.0) {
            warnings.push_back({
                "bounding-box",
                "Bounding-box corner has non-positive z.",
                true
            });
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
                    " deg galvo half-cone.",
                true
            });
        }
    }

    // Every bound below is negated so a NaN from YAML lands in the reject branch.
    // `x <= 0.0` is FALSE for NaN and would pass — which matters most for the two
    // galvo-driver fields: a NaN scale makes map_to_dac's own voltage guards false
    // too, and lround(NaN) then yields DAC code 0 (full deflection) reported as
    // success. The validator is the only place that can still catch it.
    if (!(config.stereo.baseline_m > 0.0)) {
        warnings.push_back({"stereo", "Stereo baseline must be positive.", true});
    }
    if (!(config.stereo.focal_length_px > 0.0)) {
        warnings.push_back({"stereo", "Stereo focal_length_px must be positive.", true});
    }
    if (!(config.galvo_driver.input_scale_v_per_deg > 0.0)) {
        warnings.push_back({"galvo-voltage", "input_scale_v_per_deg must be positive.", true});
    }
    if (!(config.galvo_driver.dac_max_diff_voltage > 0.0)) {
        warnings.push_back({"galvo-voltage", "dac_max_diff_voltage must be positive.", true});
    }
    // The mapper's own `dac_ref_voltage_ <= 0.0` guard fails open on NaN, and
    // lround(NaN) then wraps to DAC code 0 — full deflection reported as
    // success. This check is the only place that can still catch it.
    if (!(config.dac_ref_voltage > 0.0)) {
        warnings.push_back({"galvo-voltage", "dac_reference_voltage must be positive.", true});
    }
    // Phrased so a NaN from YAML falls into the reject branch rather than passing
    // every comparison.
    if (!(config.max_pulse_duration_ms > 0.0 && config.max_pulse_duration_ms <= 100.0)) {
        warnings.push_back({
            "pulse",
            "max_pulse_duration_ms must be in (0, 100].",
            true
        });
    }

    // The duty-cycle limit. cooldown_seconds: 0 lets end_pulse set cooldown_until_
    // to now, so the next re-acquire re-fires within a few cycles — a ~100ms-on /
    // ~15ms-off beam, which is effectively CW 2.5W.
    if (!(config.cooldown_seconds >= 1.0)) {
        warnings.push_back({
            "pulse",
            "cooldown_seconds must be at least 1.0.",
            true
        });
    }

    // Motion blanking. settle_delay_ms: 0 marks the galvo settled in the same
    // cycle the DAC was written, firing while the mirrors are still slewing.
    if (!(config.settle_delay_ms >= 0.5 && config.settle_delay_ms <= 50.0)) {
        warnings.push_back({
            "settle",
            "settle_delay_ms must be in [0.5, 50].",
            true
        });
    }

    if (!(config.watchdog_timeout_ms >= 5.0 && config.watchdog_timeout_ms <= 500.0)) {
        warnings.push_back({
            "watchdog",
            "watchdog_timeout_ms must be in [5, 500].",
            true
        });
    }

    if (!(config.watchdog_startup_grace_ms >= 100.0 &&
          config.watchdog_startup_grace_ms <= 60'000.0)) {
        warnings.push_back({
            "watchdog",
            "watchdog_startup_grace_ms must be in [100, 60000].",
            true
        });
    }

    // main derives cycle periods as 1'000'000 / target_fps; zero is a SIGFPE.
    if (config.target_fps <= 0) {
        warnings.push_back({"camera", "target_fps must be positive.", true});
    }

    if (config.frame_width <= 0 || config.frame_height <= 0) {
        warnings.push_back({"camera", "frame_width and frame_height must be positive.", true});
    } else {
        // A principal point outside the frame is a calibration error that biases
        // every target's back-projected x/y. Negated so NaN rejects. The last
        // valid column/row is width-1/height-1, so the comparison is strict.
        if (!(config.stereo.cx >= 0.0 && config.stereo.cx < config.frame_width &&
              config.stereo.cy >= 0.0 && config.stereo.cy < config.frame_height)) {
            warnings.push_back({
                "stereo",
                "Principal point (" + std::to_string(config.stereo.cx) + ", " +
                    std::to_string(config.stereo.cy) + ") lies outside the " +
                    std::to_string(config.frame_width) + "x" +
                    std::to_string(config.frame_height) + " frame.",
                true
            });
        } else {
            const double cx_offset = std::abs(config.stereo.cx - config.frame_width / 2.0);
            const double cy_offset = std::abs(config.stereo.cy - config.frame_height / 2.0);
            const double tolerance = 0.1 * std::max(config.frame_width, config.frame_height);
            if (cx_offset > tolerance || cy_offset > tolerance) {
                warnings.push_back({
                    "stereo",
                    "Principal point (" + std::to_string(config.stereo.cx) + ", " +
                        std::to_string(config.stereo.cy) + ") is far from the frame "
                        "centre (" + std::to_string(config.frame_width / 2.0) + ", " +
                        std::to_string(config.frame_height / 2.0) +
                        "). Verify the calibration matches the configured resolution.",
                    false
                });
            }
        }
    }

    if (!(config.detection.threshold >= 1 && config.detection.threshold <= 254)) {
        warnings.push_back({"detection", "detection.threshold must be in [1, 254].", true});
    }

    if (config.detection.min_blob_area_px <= 0 ||
        config.detection.max_blob_area_px < config.detection.min_blob_area_px) {
        warnings.push_back({
            "detection",
            "detection blob area bounds must satisfy 0 < min_blob_area_px <= "
            "max_blob_area_px.",
            true
        });
    }

    if (config.detection.max_blobs <= 0) {
        warnings.push_back({"detection", "detection.max_blobs must be positive.", true});
    }

    if (!(config.detection.epipolar_tolerance_px > 0.0)) {
        warnings.push_back({
            "detection", "detection.epipolar_tolerance_px must be positive.", true});
    }

    // Cross-check the blob-area floor against the geometry: a target of
    // target_size_m at the far plane projects to roughly
    // (pi/4) * (size * f / z_max)^2 pixels. A floor above that silently filters
    // out every real target and passes only objects several times larger.
    if (config.detection.target_size_m > 0.0 && config.stereo.focal_length_px > 0.0 &&
        config.bounding_box.z_max > 0.0) {
        const double diameter_px =
            config.detection.target_size_m * config.stereo.focal_length_px /
            config.bounding_box.z_max;
        const double area_px = (M_PI / 4.0) * diameter_px * diameter_px;
        if (static_cast<double>(config.detection.min_blob_area_px) > area_px) {
            warnings.push_back({
                "detection",
                "detection.min_blob_area_px (" +
                    std::to_string(config.detection.min_blob_area_px) +
                    ") exceeds the ~" + std::to_string(area_px) +
                    " px a " + std::to_string(config.detection.target_size_m) +
                    " m target projects to at z_max=" +
                    std::to_string(config.bounding_box.z_max) +
                    " m. Real targets will be filtered out and only larger objects "
                    "will pass.",
                false
            });
        }
    }

    return warnings;
}
