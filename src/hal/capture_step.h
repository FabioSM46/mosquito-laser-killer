#pragma once

#include "core/error.h"
#include "core/types.h"
#include "hal/icamera.h"
#include <chrono>
#include <cstdint>
#include <expected>

// One iteration of the capture thread's frame assembly, extracted from main
// for the same reason as control_step(): logic that lives in a lambda inside
// main() cannot be tested, and the stress suite was re-creating it by hand —
// the exact drift hazard §7 documents.
//
// Grabs left THEN right — sequential by design; the resulting temporal skew
// is §4.12's documented residual, which is why both cameras' driver
// timestamps are recorded in the frame. Returns a complete StereoFrame or the
// first camera error (the failing side is logged here; halting the system on
// that error stays with the caller, because the capture thread may only set
// atomics — §3).
//
// Pacing, the queue push, and shutdown-flag polling stay in the thread loop:
// they are plumbing between threads, not frame logic.
[[nodiscard]] auto capture_step(ICamera& left_camera, ICamera& right_camera,
                                uint64_t frame_id, int frame_width,
                                int frame_height,
                                std::chrono::steady_clock::time_point now)
    -> std::expected<StereoFrame, HardwareError>;
