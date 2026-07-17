#include "vision/multi_tracker.h"
#include "core/print.h"
#include <cmath>

MultiTracker::MultiTracker(const SystemConfig::Tracking& config)
    // Every bound sanitized so a NaN from YAML lands on the conservative side:
    // immediate confirmation and an unbounded gate would both weaken the
    // discrimination this class exists to provide.
    : confirm_hits_(config.confirm_hits >= 1 ? config.confirm_hits : 1)
    , association_gate_m_(std::isfinite(config.association_gate_m) &&
                                  config.association_gate_m > 0.0
                              ? config.association_gate_m
                              : 0.15)
    , min_speed_mps_(std::isfinite(config.min_speed_mps) &&
                             config.min_speed_mps >= 0.0
                         ? config.min_speed_mps
                         : 0.0)
    , max_speed_mps_(std::isfinite(config.max_speed_mps) &&
                             config.max_speed_mps > min_speed_mps_
                         ? config.max_speed_mps
                         : 3.0)
    , max_tracks_(config.max_tracks >= 1 ? config.max_tracks : 1) {
    println("[MULTITRACKER] confirm_hits={}, gate={:.3f}m, speed=[{:.2f}, {:.2f}]m/s, "
            "max_tracks={}",
            confirm_hits_, association_gate_m_, min_speed_mps_, max_speed_mps_,
            max_tracks_);
}

auto MultiTracker::reset() -> void {
    tracks_.clear();
    println("[MULTITRACKER] All tracks cleared");
}

auto MultiTracker::track_count() const -> size_t {
    return tracks_.size();
}

auto MultiTracker::update(const std::vector<Point3D>& measurements,
                          std::chrono::steady_clock::time_point timestamp)
    -> std::vector<TrackedTarget> {
    // Predict each track to now. A track past its horizon is guessing wherever
    // the last velocity pointed; delete it rather than report that.
    std::vector<std::optional<Point3D>> predictions(tracks_.size());
    for (size_t t = 0; t < tracks_.size(); ++t) {
        predictions[t] = tracks_[t].kf.predict(timestamp);
    }

    // Mutual-nearest-neighbour association inside the gate. A pair binds only
    // when each is the other's best candidate, so a measurement that sits
    // between two tracks claims neither the closer-by-luck wrong one nor both:
    // it binds to its genuine nearest track and the other coasts.
    std::vector<int> track_assignment(tracks_.size(), -1);
    std::vector<int> measurement_assignment(measurements.size(), -1);

    auto distance = [](const Point3D& a, const Point3D& b) {
        const double dx = a.x - b.x;
        const double dy = a.y - b.y;
        const double dz = a.z - b.z;
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    };

    for (size_t t = 0; t < tracks_.size(); ++t) {
        if (!predictions[t].has_value()) {
            continue;
        }
        double best = association_gate_m_;
        int best_m = -1;
        for (size_t m = 0; m < measurements.size(); ++m) {
            const double d = distance(predictions[t].value(), measurements[m]);
            if (d < best) {
                best = d;
                best_m = static_cast<int>(m);
            }
        }
        if (best_m < 0) {
            continue;
        }
        // Reciprocity: the measurement must name this track as its nearest too.
        double m_best = association_gate_m_;
        int m_best_t = -1;
        for (size_t t2 = 0; t2 < tracks_.size(); ++t2) {
            if (!predictions[t2].has_value()) {
                continue;
            }
            const double d =
                distance(predictions[t2].value(), measurements[static_cast<size_t>(best_m)]);
            // <= because the track we started from is itself a candidate and
            // must win its own comparison to bind.
            if (d <= m_best) {
                m_best = d;
                m_best_t = static_cast<int>(t2);
            }
        }
        if (m_best_t == static_cast<int>(t) &&
            measurement_assignment[static_cast<size_t>(best_m)] < 0) {
            track_assignment[t] = best_m;
            measurement_assignment[static_cast<size_t>(best_m)] = static_cast<int>(t);
        }
    }

    std::vector<Track> survivors;
    survivors.reserve(tracks_.size() + measurements.size());

    for (size_t t = 0; t < tracks_.size(); ++t) {
        Track track = std::move(tracks_[t]);
        if (!predictions[t].has_value()) {
            continue;   // horizon expired: fail closed
        }
        if (track_assignment[t] >= 0) {
            const auto& m = measurements[static_cast<size_t>(track_assignment[t])];
            if (track.kf.update(m, timestamp).has_value()) {
                ++track.hits;
                track.misses = 0;
                survivors.push_back(std::move(track));
            }
            // A rejected update (non-finite measurement) leaves the track
            // coasting rather than corrected by garbage — but KalmanTracker
            // already guards that, so a nullopt here means the filter state
            // itself went bad; drop the track.
        } else {
            ++track.misses;
            survivors.push_back(std::move(track));
        }
    }

    for (size_t m = 0; m < measurements.size(); ++m) {
        if (measurement_assignment[m] >= 0) {
            continue;
        }
        if (static_cast<int>(survivors.size()) >= max_tracks_) {
            // Fail closed: refuse the new track, never evict a live one. A
            // scene that busy is noise, and noise must not manufacture IDs.
            println(stderr, "[MULTITRACKER] max_tracks={} reached, refusing new track",
                    max_tracks_);
            continue;
        }
        Track track;
        track.id = next_id_++;
        if (track.kf.update(measurements[m], timestamp).has_value()) {
            track.hits = 1;
            track.misses = 0;
            survivors.push_back(std::move(track));
        }
        // Non-finite measurement: no track is created at all.
    }

    tracks_ = std::move(survivors);

    std::vector<TrackedTarget> out;
    out.reserve(tracks_.size());
    for (auto& track : tracks_) {
        auto position = track.kf.predict(timestamp);
        if (!position.has_value()) {
            continue;
        }
        const Point3D velocity = track.kf.velocity();
        const double speed =
            std::sqrt(velocity.x * velocity.x + velocity.y * velocity.y +
                      velocity.z * velocity.z);
        const bool confirmed = track.hits >= confirm_hits_;
        out.push_back(TrackedTarget{
            track.id,
            position.value(),
            velocity,
            track.hits,
            track.misses,
            confirmed,
            confirmed && speed >= min_speed_mps_ && speed <= max_speed_mps_});
    }
    return out;
}
