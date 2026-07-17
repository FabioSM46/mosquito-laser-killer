#pragma once

#include "core/types.h"
#include <vector>
#include <cstdint>
#include <cstddef>
#include <opencv2/core.hpp>

// Extracts bright connected regions from a single grayscale frame.
//
// Every blob is segmented independently: a frame-wide centroid would average
// unrelated objects into a point where nothing physically exists, and that
// phantom can satisfy every downstream safety gate.
//
// When the background model is enabled (background_learning_rate > 0) a blob
// must ALSO differ from the running background: static glints, fixtures and
// sensor dust merge into the model and stop being reported. The price is that
// a target which stops moving fades too — this system engages flying targets
// only, so that is the intended trade.
class Detector {
public:
    Detector(int width, int height, const SystemConfig::Detection& config = {});
    ~Detector() = default;

    // Returns every blob whose area is within [min_blob_area_px, max_blob_area_px]
    // and (when the model is enabled) that moves against the background.
    // Returns empty (fail closed) on a short/null frame, or when the scene holds
    // more than max_blobs candidates — a frame that busy cannot be matched
    // unambiguously, so it must not produce a target.
    //
    // Non-const: the background model advances on every accepted frame.
    [[nodiscard]] auto detect_blobs(const uint8_t* data, size_t size)
        -> std::vector<Blob>;

    // The config field is an int; narrowing through uint8_t here would wrap an
    // out-of-range value silently at the call site.
    void set_threshold(int threshold) { config_.threshold = threshold; }
    [[nodiscard]] auto threshold() const -> int { return config_.threshold; }
    [[nodiscard]] auto width() const -> int { return width_; }
    [[nodiscard]] auto height() const -> int { return height_; }
    [[nodiscard]] auto background_model_active() const -> bool {
        return learning_rate_ > 0.0;
    }

private:
    int width_;
    int height_;
    SystemConfig::Detection config_{};
    // Sanitized in the constructor: a NaN or out-of-range rate would make
    // accumulateWeighted produce a garbage model or none at all.
    double learning_rate_{0.0};

    cv::Mat background_;        // CV_32FC1 running average, same size as a frame
    bool background_seeded_{false};
};
