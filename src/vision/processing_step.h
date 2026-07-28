#pragma once

#include "core/types.h"
#include "vision/detector.h"
#include "vision/multi_tracker.h"
#include "vision/stereo_matcher.h"
#include "vision/target_selector.h"

// The processing thread's per-frame pipeline dependencies. References, not
// ownership: main owns the components, exactly like ControlDeps.
struct ProcessingDeps {
    Detector& detector_left;
    Detector& detector_right;
    StereoMatcher& matcher;
    MultiTracker& tracker;
    TargetSelector& selector;
};

// One iteration of the processing thread AFTER a frame has been obtained:
// detect → match_all → track → select, producing exactly ONE TargetCommand
// (target_valid == false when nothing engageable survived the gates — the
// control thread relies on receiving that explicit "no target" to clear a
// stale aim). Extracted from main's lambda for the same reason as
// control_step(): a pipeline that only exists inside main() cannot be tested,
// and §7's standard demands the real code under test, not a copy.
//
// Queue draining, heartbeat stamping and the skew watermark stay in the
// thread loop — they are pacing and telemetry, not target logic.
[[nodiscard]] auto processing_step(ProcessingDeps& deps, const StereoFrame& frame)
    -> TargetCommand;
