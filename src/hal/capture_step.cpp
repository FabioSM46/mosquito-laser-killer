#include "hal/capture_step.h"
#include "core/print.h"

auto capture_step(ICamera& left_camera, ICamera& right_camera,
                  uint64_t frame_id, int frame_width, int frame_height,
                  std::chrono::steady_clock::time_point now)
    -> std::expected<StereoFrame, HardwareError> {
    StereoFrame frame;
    frame.frame_id = frame_id;
    frame.timestamp = now;

    const size_t frame_bytes =
        static_cast<size_t>(frame_width) * static_cast<size_t>(frame_height);
    frame.left_frame.resize(frame_bytes);
    frame.right_frame.resize(frame_bytes);

    auto left_result =
        left_camera.capture(frame.left_frame.data(), frame.left_frame.size());
    if (!left_result.has_value()) {
        println(stderr, "[CAPTURE] Left camera capture failed: {}",
                to_string(left_result.error()));
        return std::unexpected(left_result.error());
    }
    frame.left_timestamp = left_camera.last_frame_timestamp();

    auto right_result =
        right_camera.capture(frame.right_frame.data(), frame.right_frame.size());
    if (!right_result.has_value()) {
        println(stderr, "[CAPTURE] Right camera capture failed: {}",
                to_string(right_result.error()));
        return std::unexpected(right_result.error());
    }
    frame.right_timestamp = right_camera.last_frame_timestamp();

    return frame;
}
