#include "vision/stereo_matcher.h"
#include "core/print.h"
#include <cmath>
#include <vector>

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
    , epipolar_tolerance_px_(
          // A NaN or non-positive tolerance fails the epipolar gate OPEN
          // (`std::abs(dv) > tol` is false for NaN, and <= 0 only rejects
          // exact-zero offsets), so sanitize at the boundary: anything that is
          // not a finite positive tolerance becomes 0, the strictest gate.
          std::isfinite(detection.epipolar_tolerance_px) &&
                  detection.epipolar_tolerance_px > 0.0
              ? detection.epipolar_tolerance_px
              : 0.0)
    // 0 or NaN disables the size gate rather than failing it open: with no
    // target size configured there is no expectation to check against.
    , target_size_m_(std::isfinite(detection.target_size_m) &&
                             detection.target_size_m > 0.0
                         ? detection.target_size_m
                         : 0.0)
    // A factor below 1 inverts the accept band (rejects everything); a NaN
    // fails every comparison open. Both become the strictest sane band.
    , size_tolerance_factor_(std::isfinite(detection.size_tolerance_factor) &&
                                     detection.size_tolerance_factor >= 1.0
                                 ? detection.size_tolerance_factor
                                 : 1.0) {
    // d = f * b / z, so the near plane gives the largest disparity.
    if (bounds.z_max > 0.0 && bounds.z_min > 0.0) {
        min_disparity_px_ = (focal_length_px_ * baseline_m_) / bounds.z_max;
        max_disparity_px_ = (focal_length_px_ * baseline_m_) / bounds.z_min;
    }

    println("[STEREO] Initialized: baseline={:.3f}m, f={:.1f}px, "
            "epipolar_tol={:.1f}px, disparity=[{:.1f}, {:.1f}]px (z=[{:.2f}, {:.2f}]m), "
            "size gate={} (target={:.4f}m, tolerance x{:.1f})",
            baseline_m_, focal_length_px_, epipolar_tolerance_px_,
            min_disparity_px_, max_disparity_px_, bounds.z_min, bounds.z_max,
            target_size_m_ > 0.0 ? "on" : "off", target_size_m_,
            size_tolerance_factor_);
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

auto StereoMatcher::size_consistent_with_target(const Blob& left_blob, double z) const
    -> bool {
    if (target_size_m_ <= 0.0 || focal_length_px_ <= 0.0 || !(z > 0.0)) {
        return true;
    }

    const double diameter_px = target_size_m_ * focal_length_px_ / z;
    const double expected_area_px = (M_PI / 4.0) * diameter_px * diameter_px;
    const double lo = expected_area_px / size_tolerance_factor_;
    const double hi = expected_area_px * size_tolerance_factor_;
    const double area = static_cast<double>(left_blob.area_px);

    return area >= lo && area <= hi;
}

auto StereoMatcher::match_all(const std::vector<Blob>& left,
                              const std::vector<Blob>& right) const
    -> std::vector<Point3D> {
    struct Candidate {
        size_t left_idx;
        size_t right_idx;
        Point3D point;
    };

    std::vector<Candidate> candidates;
    for (size_t li = 0; li < left.size(); ++li) {
        for (size_t ri = 0; ri < right.size(); ++ri) {
            if (!blobs_are_plausible_pair(left[li], right[ri])) {
                continue;
            }
            auto point = triangulate(left[li].centroid, right[ri].centroid);
            if (!point.has_value()) {
                continue;
            }
            if (!size_consistent_with_target(left[li], point->z)) {
                continue;
            }
            candidates.push_back(Candidate{li, ri, point.value()});
        }
    }

    // Per-cluster exclusivity. A blob appearing in more than one candidate has
    // no defensible pairing, so every candidate it touches is void. The count
    // is over raw candidates, not unique pairs: L1-R1 and L2-R1 sharing R1 is
    // precisely the case that must void both.
    std::vector<size_t> left_uses(left.size(), 0);
    std::vector<size_t> right_uses(right.size(), 0);
    for (const auto& c : candidates) {
        ++left_uses[c.left_idx];
        ++right_uses[c.right_idx];
    }

    std::vector<Point3D> points;
    for (const auto& c : candidates) {
        if (left_uses[c.left_idx] == 1 && right_uses[c.right_idx] == 1) {
            points.push_back(c.point);
        }
    }
    return points;
}

auto StereoMatcher::match(const std::vector<Blob>& left,
                          const std::vector<Blob>& right) const
    -> std::optional<Point3D> {
    auto points = match_all(left, right);
    if (points.size() != 1) {
        return std::nullopt;
    }
    return points.front();
}
