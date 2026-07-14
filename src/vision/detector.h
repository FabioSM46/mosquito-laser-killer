#pragma once

#include "core/types.h"
#include <optional>
#include <array>
#include <cstdint>

class Detector {
public:
    Detector();
    ~Detector() = default;

    [[nodiscard]] auto detect(const std::array<uint8_t, 640 * 480>& frame)
        -> std::optional<Pixel2D>;

    void set_threshold(uint8_t threshold) { threshold_ = threshold; }
    [[nodiscard]] auto threshold() const -> uint8_t { return threshold_; }

private:
    uint8_t threshold_{128};
    int min_contour_area_{50};
};
