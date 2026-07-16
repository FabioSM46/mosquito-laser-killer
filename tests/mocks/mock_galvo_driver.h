#pragma once

#include "hal/igalvo_driver.h"
#include <gmock/gmock.h>

class MockGalvoDriver : public IGalvoDriver {
public:
    MOCK_METHOD((std::expected<void, HardwareError>), set_position,
                (uint16_t x, uint16_t y), (override));
    MOCK_METHOD((std::expected<void, HardwareError>), zero, (), (override));
    MOCK_METHOD(bool, is_initialized, (), (const, override));
};
