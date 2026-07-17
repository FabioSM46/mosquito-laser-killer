#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <memory>
#include <vector>
#include <cstdint>
#include <expected>

#include "mocks/mock_spi.h"
#include "hal/mcp4922.h"
#include "core/error.h"

using namespace testing;

namespace {
constexpr auto kOk = std::expected<void, HardwareError>{};
constexpr uint16_t DAC_A_CMD = 0x3000;
constexpr uint16_t DAC_B_CMD = 0xB000;
}

class MCP4922Test : public Test {
protected:
    void SetUp() override {
        mock_spi_ = std::make_unique<NiceMock<MockSpi>>();
        raw_spi_ = mock_spi_.get();
    }

    auto make_recorder(std::vector<uint16_t>& calls) {
        return [&calls](uint16_t v) -> std::expected<void, HardwareError> {
            calls.push_back(v);
            return {};
        };
    }

    std::unique_ptr<MockSpi> mock_spi_;
    MockSpi* raw_spi_{nullptr};
};

TEST_F(MCP4922Test, ConstructorCentersBothChannels) {
    std::vector<uint16_t> calls;
    EXPECT_CALL(*raw_spi_, write16(_)).WillRepeatedly(Invoke(make_recorder(calls)));

    MCP4922 dac(std::move(mock_spi_));

    constexpr uint16_t kCenter = 2048;
    EXPECT_THAT(calls, ElementsAre(DAC_A_CMD | kCenter, DAC_B_CMD | kCenter));
    EXPECT_TRUE(dac.is_initialized());
}

TEST_F(MCP4922Test, WriteFormatsCommandBitsCorrectly) {
    std::vector<uint16_t> calls;
    EXPECT_CALL(*raw_spi_, write16(_)).WillRepeatedly(Invoke(make_recorder(calls)));

    MCP4922 dac(std::move(mock_spi_));
    calls.clear();

    auto result = dac.write({1234, 567});
    ASSERT_TRUE(result.has_value());

    EXPECT_THAT(calls, ElementsAre(
        static_cast<uint16_t>(DAC_A_CMD | (1234 & 0x0FFF)),
        static_cast<uint16_t>(DAC_B_CMD | (567 & 0x0FFF))));
}

TEST_F(MCP4922Test, WriteAtMaxValueFormatsToFullScale) {
    std::vector<uint16_t> calls;
    EXPECT_CALL(*raw_spi_, write16(_)).WillRepeatedly(Invoke(make_recorder(calls)));

    MCP4922 dac(std::move(mock_spi_));
    calls.clear();

    auto result = dac.write({4095, 4095});
    ASSERT_TRUE(result.has_value());

    EXPECT_THAT(calls, ElementsAre(
        static_cast<uint16_t>(DAC_A_CMD | 0x0FFF),
        static_cast<uint16_t>(DAC_B_CMD | 0x0FFF)));
}

TEST_F(MCP4922Test, OutOfRangeValueRejected) {
    std::vector<uint16_t> calls;
    EXPECT_CALL(*raw_spi_, write16(_)).WillRepeatedly(Invoke(make_recorder(calls)));

    MCP4922 dac(std::move(mock_spi_));
    calls.clear();

    auto result = dac.write({4096, 0});
    EXPECT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), HardwareError::DacInvalidValue);
    EXPECT_TRUE(calls.empty());
}

TEST_F(MCP4922Test, SpiFailurePropagates) {
    EXPECT_CALL(*raw_spi_, write16(_))
        .WillOnce(Return(kOk))
        .WillOnce(Return(kOk))
        .WillOnce(Return(std::unexpected(HardwareError::SpiTransferFailed)))
        .WillRepeatedly(Return(kOk));

    MCP4922 dac(std::move(mock_spi_));

    auto result = dac.write({100, 100});
    EXPECT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), HardwareError::SpiTransferFailed);
}

TEST_F(MCP4922Test, ZeroWritesBothChannelsToMidScale) {
    std::vector<uint16_t> calls;
    EXPECT_CALL(*raw_spi_, write16(_)).WillRepeatedly(Invoke(make_recorder(calls)));

    MCP4922 dac(std::move(mock_spi_));
    calls.clear();

    auto result = dac.zero();
    ASSERT_TRUE(result.has_value());
    constexpr uint16_t kCenter = 2048;
    EXPECT_THAT(calls, ElementsAre(DAC_A_CMD | kCenter, DAC_B_CMD | kCenter));
}

TEST_F(MCP4922Test, NullSpiLeavesUninitialized) {
    MCP4922 dac(nullptr);
    EXPECT_FALSE(dac.is_initialized());

    auto result = dac.write({0, 0});
    EXPECT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), HardwareError::SpiTransferFailed);
}

// §4.6 RAII shutdown: the galvo must not be left holding the last commanded
// deflection when the process tears down — the destructor re-centres both
// channels to mid-scale (0 V differential) without any caller involvement.
TEST_F(MCP4922Test, DestructorCentersBothChannels) {
    std::vector<uint16_t> calls;
    EXPECT_CALL(*raw_spi_, write16(_)).WillRepeatedly(Invoke(make_recorder(calls)));

    {
        MCP4922 dac(std::move(mock_spi_));
        ASSERT_TRUE(dac.is_initialized());
        calls.clear();  // ignore the constructor's centring writes
        ASSERT_TRUE(dac.write({4095, 0}).has_value());
        calls.clear();  // ignore the deflection writes too
    }  // ~MCP4922 runs here

    constexpr uint16_t kCenter = 2048;
    EXPECT_THAT(calls, ElementsAre(DAC_A_CMD | kCenter, DAC_B_CMD | kCenter));
}
