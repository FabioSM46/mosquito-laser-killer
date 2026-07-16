#pragma once

#include "core/types.h"
#include <optional>
#include <cstdint>
#include <cstddef>

class Detector {
public:
    Detector(int width, int height);
    ~Detector() = default;

    [[nodiscard]] auto detect(const uint8_t* data, size_t size)
        -> std::optional<Pixel2D>;

    void set_threshold(uint8_t threshold) { threshold_ = threshold; }
    [[nodiscard]] auto threshold() const -> uint8_t { return threshold_; }
    [[nodiscard]] auto width() const -> int { return width_; }
    [[nodiscard]] auto height() const -> int { return height_; }

private:
    int width_;
    int height_;
    uint8_t threshold_{128};
    int min_contour_area_{50};
};
