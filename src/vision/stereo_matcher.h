#pragma once

#include "core/types.h"
#include <optional>
#include <Eigen/Dense>

class StereoMatcher {
public:
    explicit StereoMatcher(const SystemConfig::Stereo& config);
    ~StereoMatcher() = default;

    [[nodiscard]] auto triangulate(const Pixel2D& left_point,
                                    const Pixel2D& right_point)
        -> std::optional<Point3D>;

    [[nodiscard]] auto compute_disparity_from_positions(
        const Pixel2D& left, const Pixel2D& right) const -> std::optional<double>;

private:
    double baseline_m_;
    double focal_length_px_;
    double cx_;
    double cy_;
};
