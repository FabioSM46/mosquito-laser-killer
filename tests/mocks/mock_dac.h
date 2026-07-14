#pragma once

#include "hal/idac.h"
#include <gmock/gmock.h>

class MockDac : public IDac {
public:
    MOCK_METHOD((std::expected<void, HardwareError>), write, (DacValues values), (override));
    MOCK_METHOD((std::expected<void, HardwareError>), zero, (), (override));
    MOCK_METHOD(bool, is_initialized, (), (const, override));
};
