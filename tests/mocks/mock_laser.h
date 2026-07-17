#pragma once

#include "hal/ilaser.h"
#include <gmock/gmock.h>

class MockLaser : public ILaser {
public:
    MOCK_METHOD((std::expected<void, HardwareError>), fire, (bool enable), (override));
    MOCK_METHOD((std::expected<void, HardwareError>), emergency_shutdown, (), (override));
    MOCK_METHOD(bool, is_firing, (), (const, override));
    MOCK_METHOD(bool, is_initialized, (), (const, override));
    MOCK_METHOD(void, enforce_max_pulse, (std::chrono::steady_clock::time_point now),
                (override));
};
