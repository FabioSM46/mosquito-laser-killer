#pragma once

#include "hal/igpio.h"
#include <gmock/gmock.h>

class MockGpio : public IGpio {
public:
    MOCK_METHOD((std::expected<void, HardwareError>), write, (bool value), (override));
    MOCK_METHOD((std::expected<bool, HardwareError>), read, (), (override));
    MOCK_METHOD((std::expected<void, HardwareError>), set_direction_output, (), (override));
    MOCK_METHOD((std::expected<void, HardwareError>), set_direction_input, (), (override));
    MOCK_METHOD(bool, is_open, (), (const, override));
};
