#pragma once

#include "hal/icamera.h"
#include <string>
#include <vector>

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
};
