#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

#include "core/types.h"
#include "vision/detector.h"

using namespace testing;

namespace {

constexpr int k_width = 640;
constexpr int k_height = 400;
constexpr size_t k_frame_size =
    static_cast<size_t>(k_width) * static_cast<size_t>(k_height);

auto make_dark_frame() -> std::vector<uint8_t> {
    return std::vector<uint8_t>(k_frame_size, 0);
}

// Paints a filled w x h rectangle with its top-left corner at (x0, y0). The
// centroid of the painted pixels is therefore (x0 + (w-1)/2, y0 + (h-1)/2).
void paint_rect(std::vector<uint8_t>& frame, int x0, int y0, int w, int h,
                uint8_t value = 255) {
    for (int y = y0; y < y0 + h; ++y) {
        for (int x = x0; x < x0 + w; ++x) {
            frame[static_cast<size_t>(y) * static_cast<size_t>(k_width) +
                  static_cast<size_t>(x)] = value;
        }
    }
}

// `count` well-separated 6x6 blobs (36 px each, comfortably inside the
// configured area window) laid out on a grid.
void paint_blob_grid(std::vector<uint8_t>& frame, int count) {
    for (int i = 0; i < count; ++i) {
        paint_rect(frame, 40 + (i % 5) * 120, 60 + (i / 5) * 120, 6, 6);
    }
}

// Blobs come back in connected-component label order, which is an artefact of
// the scan and not part of the contract. Sorting lets the assertions describe
// the scene rather than OpenCV's labelling.
auto sorted_by_u(std::vector<Blob> blobs) -> std::vector<Blob> {
    std::sort(blobs.begin(), blobs.end(), [](const Blob& a, const Blob& b) {
        return a.centroid.u < b.centroid.u;
    });
    return blobs;
}

class DetectorTest : public Test {
protected:
    void SetUp() override {
        detector_ = std::make_unique<Detector>(k_width, k_height, config_.detection);
    }

    SystemConfig config_{};
    std::unique_ptr<Detector> detector_;
};

TEST_F(DetectorTest, DarkFrameYieldsNoBlobs) {
    const auto frame = make_dark_frame();
    EXPECT_THAT(detector_->detect_blobs(frame.data(), frame.size()), IsEmpty());
}

TEST_F(DetectorTest, NullDataYieldsNoBlobs) {
    EXPECT_THAT(detector_->detect_blobs(nullptr, k_frame_size), IsEmpty());
}

TEST_F(DetectorTest, ShortFrameFailsClosed) {
    auto frame = make_dark_frame();
    paint_rect(frame, 300, 180, 10, 10);

    // The buffer genuinely holds a detectable blob; the only thing wrong here is
    // the declared length. A short frame means the capture was truncated, and a
    // truncated frame is torn: the rows that arrived may belong to one exposure
    // and the missing ones to the next, so the geometry is untrustworthy even
    // where the image looks intact. Scanning the part that arrived would hand the
    // matcher a confident centroid derived from a frame the camera never finished
    // sending, so the frame is dropped whole rather than salvaged.
    EXPECT_THAT(detector_->detect_blobs(frame.data(), frame.size() - 1), IsEmpty());

    // Declared honestly, the same buffer detects. Without this half, the
    // assertion above would also pass on a detector that never detects anything.
    EXPECT_EQ(detector_->detect_blobs(frame.data(), frame.size()).size(), 1u);
}

TEST_F(DetectorTest, SingleBlobReportsCentroidAndArea) {
    auto frame = make_dark_frame();
    paint_rect(frame, 300, 180, 10, 10);

    const auto blobs = detector_->detect_blobs(frame.data(), frame.size());

    ASSERT_EQ(blobs.size(), 1u);
    EXPECT_NEAR(blobs[0].centroid.u, 304.5, 0.5);
    EXPECT_NEAR(blobs[0].centroid.v, 184.5, 0.5);
    EXPECT_EQ(blobs[0].area_px, 100);
    EXPECT_EQ(blobs[0].width_px, 10);
    EXPECT_EQ(blobs[0].height_px, 10);
}

// This is the regression that motivated segmenting per blob at all.
//
// The old detector walked the whole frame accumulating one running sum of
// bright-pixel coordinates and divided once at the end. With a single target
// that is the right answer, which is why it survived so long. With two targets
// it reports neither of them: it reports their average. Two mosquitoes at u=200
// and u=440 collapsed into a single "detection" at u=320 — dead centre of the
// frame, and, at cx=320, dead centre of the optical axis too — where there was
// nothing but air.
//
// Nothing downstream can catch that. The phantom has a clean centroid, it
// triangulates to a real depth, it sits inside the bounding box, and it gets
// shot at. Worse, it sits between the two real targets, so the more targets the
// scene holds the more confidently the system aims at the gap between them.
TEST_F(DetectorTest, TwoBlobsKeepTheirOwnCentroids) {
    auto frame = make_dark_frame();
    paint_rect(frame, 196, 196, 9, 9);   // centroid (200, 200)
    paint_rect(frame, 436, 196, 9, 9);   // centroid (440, 200)

    const auto blobs =
        sorted_by_u(detector_->detect_blobs(frame.data(), frame.size()));

    ASSERT_EQ(blobs.size(), 2u);

    EXPECT_NEAR(blobs[0].centroid.u, 200.0, 0.5);
    EXPECT_NEAR(blobs[0].centroid.v, 200.0, 0.5);
    EXPECT_EQ(blobs[0].area_px, 81);

    EXPECT_NEAR(blobs[1].centroid.u, 440.0, 0.5);
    EXPECT_NEAR(blobs[1].centroid.v, 200.0, 0.5);
    EXPECT_EQ(blobs[1].area_px, 81);

    // The phantom the frame-wide mean used to invent. Neither real blob may land
    // anywhere near the midpoint of the two, because there is no object there.
    constexpr double k_phantom_u = 320.0;   // (200 + 440) / 2
    for (const auto& blob : blobs) {
        EXPECT_GT(std::abs(blob.centroid.u - k_phantom_u), 100.0)
            << "blob at u=" << blob.centroid.u
            << " sits on the frame-wide mean of the two targets";
    }
}

TEST_F(DetectorTest, BlobBelowMinAreaIsFiltered) {
    ASSERT_EQ(config_.detection.min_blob_area_px, 4);

    // 3 px. At this sensor size that is a hot pixel or a speck of dust on the
    // cover glass, not a target.
    auto too_small = make_dark_frame();
    paint_rect(too_small, 100, 100, 3, 1);
    EXPECT_THAT(detector_->detect_blobs(too_small.data(), too_small.size()),
                IsEmpty());

    // 4 px, exactly at the floor, and kept. The floor has to stay this low: a
    // 5 mm target at f=500 px subtends only ~5 px of area at z=1.0 m, so raising
    // the floor blinds the system at the far end of its own engagement volume.
    auto at_floor = make_dark_frame();
    paint_rect(at_floor, 100, 100, 2, 2);
    const auto blobs = detector_->detect_blobs(at_floor.data(), at_floor.size());
    ASSERT_EQ(blobs.size(), 1u);
    EXPECT_EQ(blobs[0].area_px, 4);
}

TEST_F(DetectorTest, BlobAboveMaxAreaIsFiltered) {
    ASSERT_EQ(config_.detection.max_blob_area_px, 400);

    // 40x40 = 1600 px: a lamp, a window, or a glint off something shiny. It is
    // bright and it has a perfectly well-defined centroid, which is exactly the
    // problem — the only thing distinguishing it from a mosquito is its size, so
    // the ceiling is the sole gate standing between the beam and a fixture on
    // the far wall.
    auto lamp = make_dark_frame();
    paint_rect(lamp, 100, 100, 40, 40);
    EXPECT_THAT(detector_->detect_blobs(lamp.data(), lamp.size()), IsEmpty());

    // 20x20 = 400 px, exactly at the ceiling, and kept.
    auto at_ceiling = make_dark_frame();
    paint_rect(at_ceiling, 100, 100, 20, 20);
    const auto blobs =
        detector_->detect_blobs(at_ceiling.data(), at_ceiling.size());
    ASSERT_EQ(blobs.size(), 1u);
    EXPECT_EQ(blobs[0].area_px, 400);
}

TEST_F(DetectorTest, TooManyBlobsFailsClosed) {
    ASSERT_EQ(config_.detection.max_blobs, 8);

    auto at_limit = make_dark_frame();
    paint_blob_grid(at_limit, 8);
    EXPECT_EQ(detector_->detect_blobs(at_limit.data(), at_limit.size()).size(), 8u);

    // One candidate more and the scene is past what the matcher can resolve: nine
    // blobs per camera is 81 candidate correspondences, and the matcher's whole
    // safety argument rests on there being exactly one plausible pairing. Return
    // nothing rather than the first eight that happened to be scanned, because
    // truncating the list would silently drop the very blob whose absence makes
    // some other pairing look unique.
    auto over_limit = make_dark_frame();
    paint_blob_grid(over_limit, 9);
    EXPECT_THAT(detector_->detect_blobs(over_limit.data(), over_limit.size()),
                IsEmpty());
}

TEST_F(DetectorTest, ThresholdIsStrictlyGreaterThan) {
    ASSERT_EQ(config_.detection.threshold, 128);

    // The gate is `>`, not `>=`, so a pixel sitting exactly on the threshold is
    // background. Pinning the boundary keeps a future "cleanup" from sliding the
    // comparison by one and quietly widening what the system will shoot at.
    auto at_threshold = make_dark_frame();
    paint_rect(at_threshold, 200, 200, 10, 10, 128);
    EXPECT_THAT(detector_->detect_blobs(at_threshold.data(), at_threshold.size()),
                IsEmpty());

    auto above_threshold = make_dark_frame();
    paint_rect(above_threshold, 200, 200, 10, 10, 129);
    EXPECT_EQ(
        detector_->detect_blobs(above_threshold.data(), above_threshold.size()).size(),
        1u);
}

TEST_F(DetectorTest, SetThresholdChangesWhatCountsAsBright) {
    auto frame = make_dark_frame();
    paint_rect(frame, 200, 200, 10, 10, 150);

    // Bright at the default gate of 128.
    ASSERT_EQ(detector_->detect_blobs(frame.data(), frame.size()).size(), 1u);

    detector_->set_threshold(200);
    EXPECT_EQ(detector_->threshold(), 200);
    // The identical frame, now below the gate.
    EXPECT_THAT(detector_->detect_blobs(frame.data(), frame.size()), IsEmpty());

    detector_->set_threshold(100);
    EXPECT_EQ(detector_->detect_blobs(frame.data(), frame.size()).size(), 1u);
}

TEST_F(DetectorTest, CustomResolutionIsHonoured) {
    constexpr int w = 320;
    constexpr int h = 240;
    Detector small(w, h, config_.detection);

    EXPECT_EQ(small.width(), w);
    EXPECT_EQ(small.height(), h);

    std::vector<uint8_t> frame(static_cast<size_t>(w) * static_cast<size_t>(h), 0);
    for (int y = 100; y < 110; ++y) {
        for (int x = 150; x < 160; ++x) {
            frame[static_cast<size_t>(y) * static_cast<size_t>(w) +
                  static_cast<size_t>(x)] = 255;
        }
    }

    const auto blobs = small.detect_blobs(frame.data(), frame.size());
    ASSERT_EQ(blobs.size(), 1u);
    EXPECT_NEAR(blobs[0].centroid.u, 154.5, 0.5);
    EXPECT_NEAR(blobs[0].centroid.v, 104.5, 0.5);
}

}   // namespace
