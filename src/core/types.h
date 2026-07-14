#pragma once

#include <cstdint>
#include <chrono>
#include <array>
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
    std::array<uint8_t, 640 * 480> left_frame{};
    std::array<uint8_t, 640 * 480> right_frame{};
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
};

struct SystemConfig {
    double settle_delay_ms{3.0};
    double max_pulse_duration_ms{100.0};
    double cooldown_seconds{10.0};
    uint32_t watchdog_missed_threshold{3};
    int frame_width{640};
    int frame_height{480};
    int target_fps{120};

    struct BoundingBox {
        double x_min{-1.0};
        double x_max{1.0};
        double y_min{-1.0};
        double y_max{1.0};
        double z_min{0.5};
        double z_max{5.0};
    } bounding_box;

    struct GalvoLimits {
        double angle_x_min_deg{-20.0};
        double angle_x_max_deg{20.0};
        double angle_y_min_deg{-20.0};
        double angle_y_max_deg{20.0};
    } galvo_limits;

    struct Stereo {
        double baseline_m{0.12};
        double focal_length_px{800.0};
        double cx{320.0};
        double cy{240.0};
    } stereo;
};
