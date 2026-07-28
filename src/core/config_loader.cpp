#include "core/config_loader.h"

#include <filesystem>
#include <format>
#include <yaml-cpp/yaml.h>

namespace {

// Assigns only when the key is present, so types.h stays the single source of
// truth for defaults. A present key with an inconvertible value (including an
// explicit null, `key:` with nothing after it) makes .as<T>() throw, which the
// caller turns into a fatal load error — never a silent default.
template <typename T>
auto load_field(const YAML::Node& parent, const char* key, T& out) -> void {
    if (parent && parent[key]) {
        out = parent[key].as<T>();
    }
}

auto absolute_or_verbatim(const std::string& path) -> std::string {
    std::error_code ec;
    auto abs = std::filesystem::absolute(path, ec);
    return ec ? path : abs.string();
}

}

auto load_config(const std::string& path)
    -> std::expected<SystemConfig, std::string> {
    SystemConfig config{};

    try {
        YAML::Node yaml = YAML::LoadFile(path);

        load_field(yaml, "settle_delay_ms", config.settle_delay_ms);
        load_field(yaml, "max_pulse_duration_ms", config.max_pulse_duration_ms);
        load_field(yaml, "cooldown_seconds", config.cooldown_seconds);
        load_field(yaml, "watchdog_timeout_ms", config.watchdog_timeout_ms);
        load_field(yaml, "watchdog_startup_grace_ms", config.watchdog_startup_grace_ms);
        load_field(yaml, "frame_width", config.frame_width);
        load_field(yaml, "frame_height", config.frame_height);
        load_field(yaml, "target_fps", config.target_fps);
        load_field(yaml, "spi_device_x", config.spi_device_x);
        load_field(yaml, "spi_device_y", config.spi_device_y);
        load_field(yaml, "spi_speed_hz", config.spi_speed_hz);
        load_field(yaml, "dac_reference_voltage", config.dac_ref_voltage);
        load_field(yaml, "laser_pin", config.laser_pin);
        load_field(yaml, "arm_switch_pin", config.arm_switch_pin);
        load_field(yaml, "e_stop_pin", config.e_stop_pin);
        load_field(yaml, "left_camera_device", config.left_camera_device);
        load_field(yaml, "right_camera_device", config.right_camera_device);

        auto bb = yaml["bounding_box"];
        load_field(bb, "x_min", config.bounding_box.x_min);
        load_field(bb, "x_max", config.bounding_box.x_max);
        load_field(bb, "y_min", config.bounding_box.y_min);
        load_field(bb, "y_max", config.bounding_box.y_max);
        load_field(bb, "z_min", config.bounding_box.z_min);
        load_field(bb, "z_max", config.bounding_box.z_max);

        auto gl = yaml["galvo_limits"];
        load_field(gl, "angle_x_min_deg", config.galvo_limits.angle_x_min_deg);
        load_field(gl, "angle_x_max_deg", config.galvo_limits.angle_x_max_deg);
        load_field(gl, "angle_y_min_deg", config.galvo_limits.angle_y_min_deg);
        load_field(gl, "angle_y_max_deg", config.galvo_limits.angle_y_max_deg);

        auto gd = yaml["galvo_driver"];
        load_field(gd, "input_scale_v_per_deg", config.galvo_driver.input_scale_v_per_deg);
        load_field(gd, "dac_max_diff_voltage", config.galvo_driver.dac_max_diff_voltage);

        auto co = yaml["camera_optics"];
        load_field(co, "lens_focal_length_mm", config.camera_optics.lens_focal_length_mm);
        load_field(co, "image_sensor_width_mm", config.camera_optics.image_sensor_width_mm);
        load_field(co, "image_sensor_height_mm", config.camera_optics.image_sensor_height_mm);

        auto cc = yaml["camera_controls"];
        load_field(cc, "exposure_auto", config.camera_controls.exposure_auto);
        load_field(cc, "exposure_absolute_us", config.camera_controls.exposure_absolute_us);
        load_field(cc, "brightness", config.camera_controls.brightness);
        load_field(cc, "gamma", config.camera_controls.gamma);
        load_field(cc, "sharpness", config.camera_controls.sharpness);
        load_field(cc, "gain", config.camera_controls.gain);

        auto st = yaml["stereo"];
        load_field(st, "baseline_m", config.stereo.baseline_m);
        load_field(st, "focal_length_px", config.stereo.focal_length_px);
        load_field(st, "cx", config.stereo.cx);
        load_field(st, "cy", config.stereo.cy);

        auto det = yaml["detection"];
        load_field(det, "threshold", config.detection.threshold);
        load_field(det, "min_blob_area_px", config.detection.min_blob_area_px);
        load_field(det, "max_blob_area_px", config.detection.max_blob_area_px);
        load_field(det, "max_blobs", config.detection.max_blobs);
        load_field(det, "epipolar_tolerance_px", config.detection.epipolar_tolerance_px);
        load_field(det, "target_size_m", config.detection.target_size_m);
        load_field(det, "background_learning_rate", config.detection.background_learning_rate);
        load_field(det, "motion_threshold", config.detection.motion_threshold);
        load_field(det, "size_tolerance_factor", config.detection.size_tolerance_factor);

        auto tr = yaml["tracking"];
        load_field(tr, "confirm_hits", config.tracking.confirm_hits);
        load_field(tr, "association_gate_m", config.tracking.association_gate_m);
        load_field(tr, "min_speed_mps", config.tracking.min_speed_mps);
        load_field(tr, "max_speed_mps", config.tracking.max_speed_mps);
        load_field(tr, "max_tracks", config.tracking.max_tracks);

    } catch (const YAML::Exception& e) {
        return std::unexpected(std::format("cannot load config '{}': {}",
                                           absolute_or_verbatim(path), e.what()));
    } catch (const std::exception& e) {
        return std::unexpected(std::format("cannot load config '{}': {}",
                                           absolute_or_verbatim(path), e.what()));
    }

    return config;
}
