#include "vision/stereo_matcher.h"
#include "core/print.h"
#include <cmath>

StereoMatcher::StereoMatcher(const SystemConfig::Stereo& config)
    : baseline_m_(config.baseline_m)
    , focal_length_px_(config.focal_length_px)
    , cx_(config.cx)
    , cy_(config.cy) {
    println("[STEREO] Initialized: baseline={:.3f}m, f={:.1f}px",
                 baseline_m_, focal_length_px_);
}

auto StereoMatcher::compute_disparity_from_positions(
    const Pixel2D& left, const Pixel2D& right) const -> std::optional<double> {
    double dx = left.u - right.u;

    if (std::abs(dx) < 0.5) {
        return std::nullopt;
    }

    return dx;
}

auto StereoMatcher::triangulate(const Pixel2D& left_point,
                                 const Pixel2D& right_point)
    -> std::optional<Point3D> {
    auto disparity = compute_disparity_from_positions(left_point, right_point);
    if (!disparity.has_value()) {
        return std::nullopt;
    }

    double d = disparity.value();
    if (d <= 0.0) {
        return std::nullopt;
    }

    double z = (focal_length_px_ * baseline_m_) / d;
    double x = (left_point.u - cx_) * z / focal_length_px_;
    double y = (left_point.v - cy_) * z / focal_length_px_;

    if (z <= 0.0) {
        return std::nullopt;
    }

    return Point3D{x, y, z};
}
