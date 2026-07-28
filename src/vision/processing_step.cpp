#include "vision/processing_step.h"

auto processing_step(ProcessingDeps& deps, const StereoFrame& frame)
    -> TargetCommand {
    TargetCommand cmd;
    cmd.frame_id = frame.frame_id;
    cmd.timestamp = frame.timestamp;

    auto left_blobs = deps.detector_left.detect_blobs(frame.left_frame.data(),
                                                      frame.left_frame.size());
    auto right_blobs = deps.detector_right.detect_blobs(frame.right_frame.data(),
                                                        frame.right_frame.size());

    // Correspondence is established per blob and validated against the
    // epipolar constraint; an ambiguous cluster yields no target from that
    // cluster, while clean pairs elsewhere in the frame survive.
    auto targets_3d = deps.matcher.match_all(left_blobs, right_blobs);

    // Tracks coast through brief detection gaps on their Kalman predictions
    // (bounded by k_max_predict_horizon_s) instead of throwing the velocity
    // estimates away; a track past the horizon is deleted — fail closed.
    // Only confirmed, plausibly-flying tracks are engageable.
    auto tracks = deps.tracker.update(targets_3d, frame.timestamp);

    // One laser, one galvo: pick the sticky/nearest engageable target.
    auto chosen = deps.selector.select(tracks);

    cmd.target_valid = chosen.has_value();
    if (chosen.has_value()) {
        cmd.target_position = chosen->position;
    }

    return cmd;
}
