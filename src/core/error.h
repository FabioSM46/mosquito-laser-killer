#pragma once

#include <cstdint>
#include <string>

enum class HardwareError : uint8_t {
    None = 0,
    GpioOpenFailed,
    GpioWriteFailed,
    GpioReadFailed,
    SpiOpenFailed,
    SpiTransferFailed,
    DacInvalidValue,
    CameraOpenFailed,
    CameraCaptureFailed,
    LaserEmergencyShutdown,
    LaserInitFailed,
    FileDescriptorError,
    Timeout,
    Unknown,
};

enum class MappingError : uint8_t {
    None = 0,
    OutOfBounds,
    GalvoAngleLimitExceeded,
    DacRangeInvalid,
    Invalid3DPoint,
    TargetBehindBaseline,
    ConversionError,
};

[[nodiscard]] constexpr auto to_string(HardwareError e) -> const char* {
    switch (e) {
    case HardwareError::None: return "None";
    case HardwareError::GpioOpenFailed: return "GpioOpenFailed";
    case HardwareError::GpioWriteFailed: return "GpioWriteFailed";
    case HardwareError::GpioReadFailed: return "GpioReadFailed";
    case HardwareError::SpiOpenFailed: return "SpiOpenFailed";
    case HardwareError::SpiTransferFailed: return "SpiTransferFailed";
    case HardwareError::DacInvalidValue: return "DacInvalidValue";
    case HardwareError::CameraOpenFailed: return "CameraOpenFailed";
    case HardwareError::CameraCaptureFailed: return "CameraCaptureFailed";
    case HardwareError::LaserEmergencyShutdown: return "LaserEmergencyShutdown";
    case HardwareError::LaserInitFailed: return "LaserInitFailed";
    case HardwareError::FileDescriptorError: return "FileDescriptorError";
    case HardwareError::Timeout: return "Timeout";
    case HardwareError::Unknown: return "Unknown";
    }
    return "InvalidError";
}

[[nodiscard]] constexpr auto to_string(MappingError e) -> const char* {
    switch (e) {
    case MappingError::None: return "None";
    case MappingError::OutOfBounds: return "OutOfBounds";
    case MappingError::GalvoAngleLimitExceeded: return "GalvoAngleLimitExceeded";
    case MappingError::DacRangeInvalid: return "DacRangeInvalid";
    case MappingError::Invalid3DPoint: return "Invalid3DPoint";
    case MappingError::TargetBehindBaseline: return "TargetBehindBaseline";
    case MappingError::ConversionError: return "ConversionError";
    }
    return "InvalidError";
}
