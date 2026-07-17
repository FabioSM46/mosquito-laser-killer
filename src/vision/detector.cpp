#include "vision/detector.h"
#include "core/print.h"
#include <cmath>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

Detector::Detector(int width, int height, const SystemConfig::Detection& config)
    : width_(width)
    , height_(height)
    , config_(config)
    // Sanitize at the boundary: accumulateWeighted(NaN) silently poisons the
    // model forever, and a rate outside [0, 1] is a config error. 0 disables
    // the motion gate outright (legacy bright-blob behaviour).
    , learning_rate_(std::isfinite(config.background_learning_rate) &&
                             config.background_learning_rate > 0.0 &&
                             config.background_learning_rate <= 1.0
                         ? config.background_learning_rate
                         : 0.0) {
    println("[DETECTOR] Initialized {}x{}, threshold={}, blob area=[{}, {}] px, "
            "max_blobs={}, motion gate={} (alpha={:.3f}, motion threshold={})",
            width_, height_, config_.threshold, config_.min_blob_area_px,
            config_.max_blob_area_px, config_.max_blobs,
            learning_rate_ > 0.0 ? "on" : "off",
            learning_rate_, config_.motion_threshold);
}

auto Detector::detect_blobs(const uint8_t* data, size_t size)
    -> std::vector<Blob> {
    std::vector<Blob> blobs;

    if (data == nullptr || width_ <= 0 || height_ <= 0) {
        return blobs;
    }

    const size_t expected = static_cast<size_t>(width_) * static_cast<size_t>(height_);
    // A short frame is a truncated capture, not a partial image to salvage.
    if (size < expected) {
        return blobs;
    }

    const cv::Mat frame(height_, width_, CV_8UC1, const_cast<uint8_t*>(data));

    // OpenCV reports allocation failures and internal errors by THROWING, and
    // this runs on the processing thread: an exception escaping the jthread
    // lambda calls std::terminate with no unwinding, so the RAII laser/galvo
    // shutdown never runs. Fail closed instead — an empty detection list only
    // ever means "no target this frame".
    try {
        cv::Mat mask;
        cv::threshold(frame, mask, static_cast<double>(config_.threshold), 255.0,
                      cv::THRESH_BINARY);

        if (learning_rate_ > 0.0) {
            if (!background_seeded_) {
                // There is nothing to diff against on the first frame; seed and
                // report nothing rather than treat the whole scene as motion.
                frame.convertTo(background_, CV_32F);
                background_seeded_ = true;
                println("[DETECTOR] Background model seeded, motion gate live");
                return blobs;
            }

            cv::Mat background_u8;
            background_.convertTo(background_u8, CV_8U);
            cv::Mat diff;
            cv::absdiff(frame, background_u8, diff);
            cv::Mat motion_mask;
            cv::threshold(diff, motion_mask,
                          static_cast<double>(config_.motion_threshold), 255.0,
                          cv::THRESH_BINARY);
            cv::bitwise_and(mask, motion_mask, mask);

            // Diff first, then fold the frame into the model: updating before
            // the diff would let the current frame suppress itself.
            cv::accumulateWeighted(frame, background_, learning_rate_);
        }

        cv::Mat labels;
        cv::Mat stats;
        cv::Mat centroids;
        const int n_labels =
            cv::connectedComponentsWithStats(mask, labels, stats, centroids, 8, CV_32S);

        // Label 0 is the background.
        for (int i = 1; i < n_labels; ++i) {
            const int area = stats.at<int>(i, cv::CC_STAT_AREA);
            if (area < config_.min_blob_area_px || area > config_.max_blob_area_px) {
                continue;
            }

            Blob blob;
            blob.centroid.u = centroids.at<double>(i, 0);
            blob.centroid.v = centroids.at<double>(i, 1);
            blob.area_px = area;
            blob.width_px = stats.at<int>(i, cv::CC_STAT_WIDTH);
            blob.height_px = stats.at<int>(i, cv::CC_STAT_HEIGHT);
            blobs.push_back(blob);

            // Bail out as soon as the scene is provably too busy to disambiguate.
            if (static_cast<int>(blobs.size()) > config_.max_blobs) {
                blobs.clear();
                return blobs;
            }
        }
    } catch (...) {
        println(stderr, "[DETECTOR] OpenCV failure, treating frame as empty");
        blobs.clear();
    }

    return blobs;
}
