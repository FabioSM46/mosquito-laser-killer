#pragma once

#include "core/types.h"
#include <vector>
#include <cstdint>
#include <cstddef>

// Extracts bright connected regions from a single grayscale frame.
//
// Every blob is segmented independently: a frame-wide centroid would average
// unrelated objects into a point where nothing physically exists, and that
// phantom can satisfy every downstream safety gate.
class Detector {
public:
    Detector(int width, int height, const SystemConfig::Detection& config = {});
    ~Detector() = default;

    // Returns every blob whose area is within [min_blob_area_px, max_blob_area_px].
    // Returns empty (fail closed) on a short/null frame, or when the scene holds
    // more than max_blobs candidates — a frame that busy cannot be matched
    // unambiguously, so it must not produce a target.
    [[nodiscard]] auto detect_blobs(const uint8_t* data, size_t size) const
        -> std::vector<Blob>;

    // The config field is an int; narrowing through uint8_t here would wrap an
    // out-of-range value silently at the call site.
    void set_threshold(int threshold) { config_.threshold = threshold; }
    [[nodiscard]] auto threshold() const -> int { return config_.threshold; }
    [[nodiscard]] auto width() const -> int { return width_; }
    [[nodiscard]] auto height() const -> int { return height_; }

private:
    int width_;
    int height_;
    SystemConfig::Detection config_{};
};
