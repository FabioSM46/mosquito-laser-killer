#include "vision/detector.h"
#include "core/print.h"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

Detector::Detector(int width, int height, const SystemConfig::Detection& config)
    : width_(width)
    , height_(height)
    , config_(config) {
    println("[DETECTOR] Initialized {}x{}, threshold={}, blob area=[{}, {}] px, "
            "max_blobs={}",
            width_, height_, config_.threshold, config_.min_blob_area_px,
            config_.max_blob_area_px, config_.max_blobs);
}

auto Detector::detect_blobs(const uint8_t* data, size_t size) const
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

    cv::Mat mask;
    cv::threshold(frame, mask, static_cast<double>(config_.threshold), 255.0,
                  cv::THRESH_BINARY);

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

    return blobs;
}
