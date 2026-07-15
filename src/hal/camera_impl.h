#pragma once

#include "hal/icamera.h"
#include <string>

class CameraImpl final : public ICamera {
public:
    CameraImpl(const std::string& device, int width, int height, int fps);
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

private:
    std::string device_;
    int width_;
    int height_;
    int fps_;
    int fd_{-1};
};
