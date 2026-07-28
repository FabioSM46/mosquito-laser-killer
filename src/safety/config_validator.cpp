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

    // Deliberately collapses asymmetric limits to the TIGHTEST arm: the box is
    // then checked against a smaller cone than the galvo can reach, which is
    // the conservative direction.
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

    // A NaN or non-positive optics value silently suppresses both FOV
    // cross-checks (every comparison involving NaN is false), so say so
    // explicitly. Non-critical: the optics feed no runtime path, only these
    // checks.
    const auto& optics = config.camera_optics;
    if (!(std::isfinite(optics.lens_focal_length_mm) &&
          optics.lens_focal_length_mm > 0.0) ||
        !(std::isfinite(optics.image_sensor_width_mm) &&
          optics.image_sensor_width_mm > 0.0) ||
        !(std::isfinite(optics.image_sensor_height_mm) &&
          optics.image_sensor_height_mm > 0.0)) {
        warnings.push_back({
            "camera-fov",
            "camera_optics values must be finite and positive for the FOV "
            "cross-checks to run; they are being skipped.",
            false
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

    const double half_vfov = vertical_fov_deg(config.camera_optics) / 2.0;
    if (half_vfov < galvo_half_cone_deg) {
        warnings.push_back({
            "camera-fov",
            "Camera vertical half-FOV (" + std::to_string(half_vfov) +
                " deg) is narrower than the galvo half-cone (" +
                std::to_string(galvo_half_cone_deg) +
                " deg). Targets reachable by the galvo may be outside the camera view.",
            false
        });
    }

    // An inverted box passes the per-corner checks (each corner is
    // individually reachable) but makes contains() the empty set: every
    // target rejected, the system silently dead with no diagnostic. Negated
    // so a NaN bound rejects here as well as in the corner loop.
    const auto& bb = config.bounding_box;
    if (!(bb.x_min < bb.x_max && bb.y_min < bb.y_max && bb.z_min < bb.z_max)) {
        warnings.push_back({
            "bounding-box",
            "Bounding box must satisfy min < max on x, y and z.",
            true
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

    // The capture thread derives its pacing as 1'000'000 / target_fps; zero is
    // a SIGFPE. (The CONTROL thread's period is a fixed constant and does not
    // depend on this — see control_loop.h.)
    if (config.target_fps <= 0) {
        warnings.push_back({"camera", "target_fps must be positive.", true});
    } else if (!(1000.0 / config.target_fps < config.watchdog_timeout_ms)) {
        // The heartbeat advances once per processed frame, so the frame period
        // must fit inside the watchdog tolerance. A target_fps low enough to
        // violate this can only ever ride out the startup grace and then
        // SAFE_HALT with a baffling "heartbeat stale" message — make it a
        // config error with a config-shaped message instead. Negated so a NaN
        // watchdog_timeout_ms lands in the reject branch.
        warnings.push_back({
            "watchdog",
            "Frame period (1000/target_fps = " +
                std::to_string(1000.0 / config.target_fps) +
                " ms) must be shorter than watchdog_timeout_ms (" +
                std::to_string(config.watchdog_timeout_ms) +
                " ms), or the watchdog trips between consecutive frames.",
            true
        });
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

    // Bounded above as well: the epipolar gate is the correspondence PROOF,
    // and a huge tolerance from YAML (1e9) accepts any vertical offset — the
    // guard is then formally present but proves nothing. 20 px is far beyond
    // any sane rectification error at 640x400.
    if (!(config.detection.epipolar_tolerance_px > 0.0 &&
          config.detection.epipolar_tolerance_px <= 20.0)) {
        warnings.push_back({
            "detection", "detection.epipolar_tolerance_px must be in (0, 20].", true});
    }

    // 0 disables the motion gate deliberately; above 0.5 the model chases the
    // current frame so fast that a slow target erases itself from the mask,
    // and the system goes blind without ever logging why.
    if (!(config.detection.background_learning_rate >= 0.0 &&
          config.detection.background_learning_rate <= 0.5)) {
        warnings.push_back({
            "detection",
            "detection.background_learning_rate must be in [0, 0.5] (0 disables "
            "the motion gate).",
            true
        });
    }

    // Only meaningful when the gate is live, but validated unconditionally: a
    // garbage value here must not sit dormant in the YAML waiting for the gate
    // to be switched on.
    if (!(config.detection.motion_threshold >= 1 &&
          config.detection.motion_threshold <= 254)) {
        warnings.push_back({
            "detection", "detection.motion_threshold must be in [1, 254].", true});
    }

    // Below 1.0 the size accept band inverts and rejects every real target.
    if (!(config.detection.size_tolerance_factor >= 1.0 &&
          config.detection.size_tolerance_factor <= 100.0)) {
        warnings.push_back({
            "detection", "detection.size_tolerance_factor must be in [1, 100].", true});
    }

    if (config.tracking.confirm_hits < 1 || config.tracking.confirm_hits > 60) {
        warnings.push_back({
            "tracking", "tracking.confirm_hits must be in [1, 60].", true});
    }

    // A gate larger than the engagement volume associates any measurement with
    // any track; the bounding box diagonal here is well under 1 m.
    if (!(config.tracking.association_gate_m > 0.0 &&
          config.tracking.association_gate_m <= 1.0)) {
        warnings.push_back({
            "tracking", "tracking.association_gate_m must be in (0, 1.0].", true});
    }

    if (!(config.tracking.min_speed_mps >= 0.0) ||
        !(config.tracking.max_speed_mps > config.tracking.min_speed_mps) ||
        !(config.tracking.max_speed_mps <= 20.0)) {
        warnings.push_back({
            "tracking",
            "tracking speeds must satisfy 0 <= min_speed_mps < max_speed_mps <= 20.",
            true
        });
    }

    if (config.tracking.max_tracks < 1 || config.tracking.max_tracks > 256) {
        warnings.push_back({
            "tracking", "tracking.max_tracks must be in [1, 256].", true});
    }

    // spi_speed_hz is signed in the config but unsigned in SpiImpl, so a
    // negative YAML value wraps to a huge uint32_t; and beyond 20 MHz is out
    // of MCP4922 spec — marginal transfers can corrupt DAC codes AFTER the
    // mapper validated them.
    if (config.spi_speed_hz <= 0 || config.spi_speed_hz > 20'000'000) {
        warnings.push_back({
            "spi", "spi_speed_hz must be in (0, 20000000] (MCP4922 maximum).", true});
    }

    // Three interlocks on one pin is a wiring error the software cannot make
    // safe: e.g. the e-stop sense doubling as the laser TTL.
    if (config.laser_pin == config.arm_switch_pin ||
        config.laser_pin == config.e_stop_pin ||
        config.arm_switch_pin == config.e_stop_pin) {
        warnings.push_back({
            "gpio",
            "laser_pin, arm_switch_pin and e_stop_pin must be three distinct GPIOs.",
            true});
    }

    // The matcher sanitizes a non-finite or negative target_size_m to "size
    // gate off" (0 disables it deliberately); that silent degradation
    // deserves a line in the startup record.
    if (!(config.detection.target_size_m >= 0.0) ||
        !std::isfinite(config.detection.target_size_m)) {
        warnings.push_back({
            "detection",
            "detection.target_size_m is not a finite non-negative value; the "
            "depth-consistent size gate is disabled.",
            false});
    }

    // An exposure longer than the frame period cannot be honoured: the driver
    // clamps the frame rate instead, silently violating the target_fps the
    // rest of the timing budget assumes.
    if (config.target_fps > 0 &&
        static_cast<double>(config.camera_controls.exposure_absolute_us) >
            1'000'000.0 / config.target_fps) {
        warnings.push_back({
            "camera",
            "camera_controls.exposure_absolute_us (" +
                std::to_string(config.camera_controls.exposure_absolute_us) +
                " us) exceeds the frame period at target_fps (" +
                std::to_string(1'000'000.0 / config.target_fps) +
                " us); the driver will clamp the frame rate.",
            false});
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
