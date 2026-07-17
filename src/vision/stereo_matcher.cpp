#include "vision/stereo_matcher.h"
#include "core/print.h"
#include <cmath>

namespace {

// The same object seen by two cameras at a shared distance projects to a similar
// area. A large mismatch means the two blobs are different objects.
constexpr double k_min_area_ratio = 0.33;
constexpr double k_max_area_ratio = 3.0;

}

StereoMatcher::StereoMatcher(const SystemConfig::Stereo& config,
                             const SystemConfig::Detection& detection,
                             const SystemConfig::BoundingBox& bounds)
    : baseline_m_(config.baseline_m)
    , focal_length_px_(config.focal_length_px)
    , cx_(config.cx)
    , cy_(config.cy)
    , epipolar_tolerance_px_(detection.epipolar_tolerance_px) {
    // d = f * b / z, so the near plane gives the largest disparity.
    if (bounds.z_max > 0.0 && bounds.z_min > 0.0) {
        min_disparity_px_ = (focal_length_px_ * baseline_m_) / bounds.z_max;
        max_disparity_px_ = (focal_length_px_ * baseline_m_) / bounds.z_min;
    }

    println("[STEREO] Initialized: baseline={:.3f}m, f={:.1f}px, "
            "epipolar_tol={:.1f}px, disparity=[{:.1f}, {:.1f}]px (z=[{:.2f}, {:.2f}]m)",
            baseline_m_, focal_length_px_, epipolar_tolerance_px_,
            min_disparity_px_, max_disparity_px_, bounds.z_min, bounds.z_max);
}

auto StereoMatcher::triangulate(const Pixel2D& left_point,
                                const Pixel2D& right_point) const
    -> std::optional<Point3D> {
    if (!std::isfinite(left_point.u) || !std::isfinite(left_point.v) ||
        !std::isfinite(right_point.u) || !std::isfinite(right_point.v)) {
        return std::nullopt;
    }

    // Epipolar constraint. For a rectified pair the same object lands on the same
    // row in both frames; a vertical offset proves these are different objects.
    if (std::abs(left_point.v - right_point.v) > epipolar_tolerance_px_) {
        return std::nullopt;
    }

    const double d = left_point.u - right_point.u;

    // Phrased so NaN falls through to the reject branch.
    if (!(d >= min_disparity_px_ && d <= max_disparity_px_)) {
        return std::nullopt;
    }

    const double z = (focal_length_px_ * baseline_m_) / d;
    const double x = (left_point.u - cx_) * z / focal_length_px_;
    const double y = (left_point.v - cy_) * z / focal_length_px_;

    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        return std::nullopt;
    }

    return Point3D{x, y, z};
}

auto StereoMatcher::blobs_are_plausible_pair(const Blob& left, const Blob& right) const
    -> bool {
    if (left.area_px <= 0 || right.area_px <= 0) {
        return false;
    }

    const double ratio = static_cast<double>(left.area_px) /
                         static_cast<double>(right.area_px);
    return ratio >= k_min_area_ratio && ratio <= k_max_area_ratio;
}

auto StereoMatcher::match(const std::vector<Blob>& left,
                          const std::vector<Blob>& right) const
    -> std::optional<Point3D> {
    std::optional<Point3D> sole_candidate;

    for (const auto& l : left) {
        for (const auto& r : right) {
            if (!blobs_are_plausible_pair(l, r)) {
                continue;
            }

            auto point = triangulate(l.centroid, r.centroid);
            if (!point.has_value()) {
                continue;
            }

            // A second surviving correspondence means the scene is ambiguous.
            // Fail closed rather than pick a winner.
            if (sole_candidate.has_value()) {
                return std::nullopt;
            }
            sole_candidate = point;
        }
    }

    return sole_candidate;
}
