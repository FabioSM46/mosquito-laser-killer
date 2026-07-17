#pragma once

#include "core/types.h"
#include "vision/tracker.h"
#include <chrono>
#include <cstdint>
#include <optional>
#include <vector>

// A single track's outward state, as of the last update() call.
struct TrackedTarget {
    uint32_t id{0};
    Point3D position{};
    Point3D velocity{};
    int hits{0};
    int misses{0};
    // Enough consecutive matched frames to trust this is a real object.
    bool confirmed{false};
    // Confirmed AND flying at a plausible speed. Only engageable targets may
    // reach the control thread: ~0 m/s is a glint or a fixture, and beyond
    // max_speed is a correspondence artefact, not flight.
    bool engageable{false};
};

// Owns every live track and associates new triangulated measurements to them.
//
// Confirmation (hits >= confirm_hits) is what stands between a one-frame
// phantom and the firing path; coasting through a brief detection gap rides
// the Kalman prediction, bounded by KalmanTracker::k_max_predict_horizon_s,
// and every reported position — measured or coasted — still passes through
// the downstream bounding box, galvo cone and DAC range gates.
class MultiTracker {
public:
    explicit MultiTracker(const SystemConfig::Tracking& config = {});
    ~MultiTracker() = default;

    // Associates the frame's measurements to live tracks, creates tentative
    // tracks for the unmatched, and returns every surviving track. A track
    // whose prediction horizon has expired is deleted (fail closed).
    [[nodiscard]] auto update(const std::vector<Point3D>& measurements,
                              std::chrono::steady_clock::time_point timestamp)
        -> std::vector<TrackedTarget>;

    void reset();
    [[nodiscard]] auto track_count() const -> size_t;

private:
    struct Track {
        uint32_t id{0};
        KalmanTracker kf{};
        int hits{0};
        int misses{0};
    };

    int confirm_hits_{3};
    double association_gate_m_{0.15};
    double min_speed_mps_{0.0};
    double max_speed_mps_{3.0};
    int max_tracks_{16};

    std::vector<Track> tracks_;
    uint32_t next_id_{1};
};
