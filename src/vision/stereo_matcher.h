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

    // Returns every correspondence that survives all gates, with per-cluster
    // exclusivity: any blob participating in more than one candidate pair is
    // ambiguous, so every pair involving it is dropped. An ambiguous cluster
    // therefore produces nothing while a clean pair elsewhere in the same
    // frame still yields its target. A wrong pairing aims a Class 4 beam at a
    // point never verified to hold a target, so ambiguity is always resolved
    // toward silence, never toward a guess.
    [[nodiscard]] auto match_all(const std::vector<Blob>& left,
                                 const std::vector<Blob>& right) const
        -> std::vector<Point3D>;

    // Single-target view of match_all: a point only when exactly one
    // correspondence survives. Zero and multiple survivors both yield nullopt.
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

    // At a measured z, a target_size_m object projects to a known area in
    // pixels. A blob far outside that band is a glint, a fixture, or a
    // different animal — not the target, whatever its centroid says.
    [[nodiscard]] auto size_consistent_with_target(const Blob& left_blob,
                                                   double z) const -> bool;

    double baseline_m_;
    double focal_length_px_;
    double cx_;
    double cy_;
    double epipolar_tolerance_px_;
    double min_disparity_px_{0.0};
    double max_disparity_px_{0.0};
    double target_size_m_{0.0};
    double size_tolerance_factor_{1.0};
};
