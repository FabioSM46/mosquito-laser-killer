#pragma once

#include "core/types.h"
#include <optional>
#include <vector>

// Establishes stereo correspondence between blobs and triangulates the result.
//
// Correspondence is validated, not assumed. In this system the aim angle is
// atan2(x, z) with x = (u_left - cx) * z / f, so z cancels and the aim depends
// only on the left pixel. That makes z's sole job the safety discriminator
// (is the target inside the engagement volume, or is it a face across the room?)
// — so a z fabricated from a bad correspondence defeats the primary guard.
class StereoMatcher {
public:
    StereoMatcher(const SystemConfig::Stereo& config,
                  const SystemConfig::Detection& detection,
                  const SystemConfig::BoundingBox& bounds);
    ~StereoMatcher() = default;

    // Returns a point only when exactly one plausible correspondence exists.
    // Zero candidates and multiple candidates both yield nullopt: an ambiguous
    // pairing is a guess, and a wrong guess aims a Class 4 beam at a point that
    // was never verified to hold the target.
    [[nodiscard]] auto match(const std::vector<Blob>& left,
                             const std::vector<Blob>& right) const
        -> std::optional<Point3D>;

    // Triangulates a single pair, applying the epipolar and disparity gates.
    [[nodiscard]] auto triangulate(const Pixel2D& left_point,
                                   const Pixel2D& right_point) const
        -> std::optional<Point3D>;

    // Disparity window implied by the configured z range: d = f * b / z.
    [[nodiscard]] auto min_disparity_px() const -> double { return min_disparity_px_; }
    [[nodiscard]] auto max_disparity_px() const -> double { return max_disparity_px_; }

private:
    [[nodiscard]] auto blobs_are_plausible_pair(const Blob& left, const Blob& right) const
        -> bool;

    double baseline_m_;
    double focal_length_px_;
    double cx_;
    double cy_;
    double epipolar_tolerance_px_;
    double min_disparity_px_{0.0};
    double max_disparity_px_{0.0};
};
