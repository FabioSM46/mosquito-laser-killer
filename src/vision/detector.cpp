#include "vision/detector.h"
#include "core/print.h"
#include <algorithm>

Detector::Detector() {
    println("[DETECTOR] Initialized, threshold={}", threshold_);
}

auto Detector::detect(const std::array<uint8_t, 640 * 480>& frame)
    -> std::optional<Pixel2D> {
    int sum_x = 0;
    int sum_y = 0;
    int count = 0;

    for (int y = 0; y < 480; ++y) {
        for (int x = 0; x < 640; ++x) {
            uint8_t px = frame.at(y * 640 + x);
            if (px > threshold_) {
                sum_x += x;
                sum_y += y;
                ++count;
            }
        }
    }

    if (count < min_contour_area_) {
        return std::nullopt;
    }

    Pixel2D centroid;
    centroid.u = static_cast<double>(sum_x) / count;
    centroid.v = static_cast<double>(sum_y) / count;

    return centroid;
}
