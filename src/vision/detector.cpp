#include "vision/detector.h"
#include "core/print.h"
#include <algorithm>

Detector::Detector(int width, int height)
    : width_(width)
    , height_(height) {
    println("[DETECTOR] Initialized {}x{}, threshold={}", width_, height_, threshold_);
}

auto Detector::detect(const uint8_t* data, size_t size)
    -> std::optional<Pixel2D> {
    int sum_x = 0;
    int sum_y = 0;
    int count = 0;

    const size_t expected = static_cast<size_t>(width_) * static_cast<size_t>(height_);
    const size_t usable = std::min(size, expected);

    for (size_t i = 0; i < usable; ++i) {
        if (data[i] > threshold_) {
            const int x = static_cast<int>(i % width_);
            const int y = static_cast<int>(i / width_);
            sum_x += x;
            sum_y += y;
            ++count;
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
