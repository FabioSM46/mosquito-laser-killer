#pragma once

#include "hal/ilaser.h"
#include <gmock/gmock.h>

class MockLaser : public ILaser {
public:
    MOCK_METHOD((std::expected<void, HardwareError>), fire, (bool enable), (override));
    MOCK_METHOD((std::expected<void, HardwareError>), emergency_shutdown, (), (override));
    MOCK_METHOD(bool, is_firing, (), (const, override));
};
