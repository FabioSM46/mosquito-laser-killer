#pragma once

#include "hal/icamera.h"
#include <gmock/gmock.h>

class MockCamera : public ICamera {
public:
    MOCK_METHOD((std::expected<void, HardwareError>), open, (int device_index), (override));
    MOCK_METHOD((std::expected<void, HardwareError>), capture,
                (uint8_t* buffer, size_t size), (override));
    MOCK_METHOD(bool, is_open, (), (const, override));
    MOCK_METHOD(void, close, (), (override));
};
