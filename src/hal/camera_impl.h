#pragma once

#include "hal/icamera.h"
#include <chrono>
#include <string>
#include <vector>

// V4L2_CID_EXPOSURE_ABSOLUTE is denominated in units of 100 µs, not µs. The
// config keeps the honest microsecond unit its key name promises; this
// converts at the V4L2 boundary (nearest unit, floor of 1 — a request of 0 is
// driver-defined). Passing the µs value straight through, as the code once
// did, requested a 100× longer exposure than configured: 156 µs became
// 15.6 ms, longer than a whole frame at 120/210 fps.
[[nodiscard]] constexpr auto exposure_us_to_v4l2_units(int exposure_us) -> int {
    const int units = (exposure_us + 50) / 100;
    return units < 1 ? 1 : units;
}

// V4L2 capture using memory-mapped streaming I/O.
//
// uvcvideo — the driver for the OV9281 USB3 UVC modules this system targets —
// does not implement read()/write() I/O and does not advertise V4L2_CAP_READWRITE,
// and VIDIOC_STREAMON is rejected until buffers have been allocated via
// VIDIOC_REQBUFS. Streaming I/O is the only path that works on this hardware.
class CameraImpl final : public ICamera {
public:
    CameraImpl(const std::string& device, int width, int height, int fps,
               const SystemConfig::CameraControls& controls = {});

    ~CameraImpl() override;

    CameraImpl(const CameraImpl&) = delete;
    auto operator=(const CameraImpl&) -> CameraImpl& = delete;
    CameraImpl(CameraImpl&&) noexcept;
    auto operator=(CameraImpl&&) noexcept -> CameraImpl&;

    [[nodiscard]] auto open(int device_index) -> std::expected<void, HardwareError> override;
    [[nodiscard]] auto capture(uint8_t* buffer, size_t size)
        -> std::expected<void, HardwareError> override;
    [[nodiscard]] auto is_open() const -> bool override;
    void close() override;

    // Frame size the driver actually negotiated. Valid once open() succeeds.
    [[nodiscard]] auto frame_size_bytes() const -> size_t { return frame_size_bytes_; }

    // Driver-reported exposure timestamp of the most recent successful
    // capture() (uvcvideo stamps CLOCK_MONOTONIC, the clock steady_clock reads
    // on Linux), falling back to steady_clock::now() when the driver provides
    // none. The two cameras free-run without hardware sync, so the capture
    // thread records both sides' stamps to make the stereo pair's temporal
    // skew measurable — see AGENTS.md §4.12.
    [[nodiscard]] auto last_frame_timestamp() const
        -> std::chrono::steady_clock::time_point {
        return last_frame_timestamp_;
    }

private:
    struct MappedBuffer {
        void* start{nullptr};
        size_t length{0};
    };

    static constexpr unsigned int k_buffer_count = 4;

    auto apply_controls() -> void;
    auto negotiate_format() -> std::expected<void, HardwareError>;
    auto init_mmap_buffers() -> std::expected<void, HardwareError>;
    auto unmap_buffers() -> void;

    std::string device_;
    int width_;
    int height_;
    int fps_;
    SystemConfig::CameraControls controls_{};
    int fd_{-1};
    size_t frame_size_bytes_{0};
    std::vector<MappedBuffer> buffers_{};
    bool streaming_{false};
    std::chrono::steady_clock::time_point last_frame_timestamp_{};
};
