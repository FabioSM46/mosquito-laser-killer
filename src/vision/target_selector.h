#pragma once

#include "vision/multi_tracker.h"
#include <cstdint>
#include <optional>
#include <vector>

// Chooses the one target the control thread will aim at this frame.
//
// The laser fires a single pulse at a time with a mandatory cooldown, so
// re-picking the "best" target every frame would ping-pong the galvo across
// the swarm and never settle long enough to fire. The policy is therefore
// sticky: hold the engaged target while it stays engageable, and only when
// it is lost (or killed) fall back to the nearest engageable track.
class TargetSelector {
public:
    TargetSelector() = default;
    ~TargetSelector() = default;

    [[nodiscard]] auto select(const std::vector<TrackedTarget>& targets)
        -> std::optional<TrackedTarget>;

    // Forget the engaged target (e.g. on disarm); next select() picks fresh.
    void reset();

private:
    std::optional<uint32_t> engaged_id_;
};
