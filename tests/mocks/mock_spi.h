#pragma once

#include "hal/ispi.h"
#include <gmock/gmock.h>

class MockSpi : public ISpi {
public:
    MOCK_METHOD((std::expected<void, HardwareError>), transfer,
                (std::span<const uint8_t> tx, std::span<uint8_t> rx), (override));
    MOCK_METHOD((std::expected<void, HardwareError>), write16, (uint16_t value), (override));
    MOCK_METHOD(bool, is_open, (), (const, override));
};
