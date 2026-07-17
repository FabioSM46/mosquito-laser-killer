#pragma once

#include <cstdint>
#include <chrono>
#include <vector>
#include <string>
#include <optional>
#include <Eigen/Dense>

struct Point3D {
    double x{0.0};
    double y{0.0};
    double z{0.0};

    [[nodiscard]] auto as_eigen() const -> Eigen::Vector3d {
        return {x, y, z};
    }

    static auto from_eigen(const Eigen::Vector3d& v) -> Point3D {
        return {v.x(), v.y(), v.z()};
    }
};

struct Pixel2D {
    double u{0.0};
    double v{0.0};
};

struct StereoFrame {
    uint64_t frame_id{0};
    std::chrono::steady_clock::time_point timestamp{};
    std::vector<uint8_t> left_frame{};
    std::vector<uint8_t> right_frame{};
};

struct TargetCommand {
    uint64_t frame_id{0};
    std::chrono::steady_clock::time_point timestamp{};
    std::optional<Point3D> target_position{};
    bool target_valid{false};
};

struct DacValues {
    uint16_t channel_a{0};
    uint16_t channel_b{0};

    [[nodiscard]] friend auto operator<=>(const DacValues&, const DacValues&) = default;
};

struct SystemConfig {
    std::string left_camera_device{};
    std::string right_camera_device{};
    std::string spi_device_x{"/dev/spidev0.0"};
    std::string spi_device_y{"/dev/spidev0.1"};
    int spi_speed_hz{20'000'000};
    double dac_ref_voltage{5.0};
    unsigned int laser_pin{18};
    unsigned int arm_switch_pin{24};
    unsigned int e_stop_pin{25};
    double settle_delay_ms{3.0};
    double max_pulse_duration_ms{100.0};
    double cooldown_seconds{10.0};
    // Absolute, deliberately not derived from target_fps: a performance knob must
    // never retune a safety interlock.
    double watchdog_timeout_ms{25.0};
    double watchdog_startup_grace_ms{5000.0};
    int frame_width{640};
    int frame_height{400};
    int target_fps{120};

    struct BoundingBox {
        double x_min{-0.09};
        double x_max{0.09};
        double y_min{-0.09};
        double y_max{0.09};
        double z_min{0.5};
        double z_max{1.0};
    } bounding_box;

    struct GalvoLimits {
        double angle_x_min_deg{-15.0};
        double angle_x_max_deg{15.0};
        double angle_y_min_deg{-15.0};
        double angle_y_max_deg{15.0};
    } galvo_limits;

    struct GalvoDriver {
        double input_scale_v_per_deg{0.33};
        double dac_max_diff_voltage{5.0};
        double driver_input_voltage{15.0};
    } galvo_driver;

    struct CameraOptics {
        double lens_focal_length_mm{3.0};
        double image_sensor_width_mm{3.84};
        double image_sensor_height_mm{2.4};
    } camera_optics;

    struct CameraControls {
        int exposure_auto{1};
        int exposure_absolute_us{156};
        int brightness{0};
        int gamma{100};
        int sharpness{0};
        int gain{0};
    } camera_controls;

    struct Stereo {
        double baseline_m{0.12};
        double focal_length_px{500.0};
        double cx{320.0};
        double cy{200.0};
    } stereo;

    struct Detection {
        // 8-bit intensity gate for the dark-field/bright-target regime.
        int threshold{128};
        // Per-blob area bounds, in pixels. A 5 mm target at f=500 px subtends
        // ~5 px across at z=0.5 m (~20 px area) and ~2.5 px at z=1.0 m (~5 px
        // area), so the floor must stay low. The ceiling is what rejects lamps,
        // windows, and specular glints, which are orders of magnitude larger.
        int min_blob_area_px{4};
        int max_blob_area_px{400};
        // Frames busier than this are ambiguous; fail closed rather than guess.
        int max_blobs{8};
        // Rectified-pair epipolar gate: |v_left - v_right| above this proves the
        // two blobs are not the same object.
        double epipolar_tolerance_px{2.0};
        // Nominal target size, used to sanity-check min_blob_area_px against the
        // configured z range at startup.
        double target_size_m{0.005};
    } detection;
};

// A connected bright region in one camera's frame.
struct Blob {
    Pixel2D centroid{};
    int area_px{0};
    int width_px{0};
    int height_px{0};
};
