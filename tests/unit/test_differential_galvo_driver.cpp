#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <memory>
#include <vector>
#include <expected>

#include "mocks/mock_dac.h"
#include "hal/differential_galvo_driver.h"
#include "core/error.h"
#include "core/types.h"

using namespace testing;

namespace {
constexpr auto kOk = std::expected<void, HardwareError>{};
constexpr DacValues kCenter{2048, 2047};
}

class DifferentialGalvoDriverTest : public Test {
protected:
    void SetUp() override {
        mock_dac_x_ = std::make_unique<NiceMock<MockDac>>();
        mock_dac_y_ = std::make_unique<NiceMock<MockDac>>();
        raw_x_ = mock_dac_x_.get();
        raw_y_ = mock_dac_y_.get();
    }

    auto make_recorder(std::vector<DacValues>& out) {
        return [&out](DacValues v) -> std::expected<void, HardwareError> {
            out.push_back(v);
            return {};
        };
    }

    std::unique_ptr<MockDac> mock_dac_x_;
    std::unique_ptr<MockDac> mock_dac_y_;
    MockDac* raw_x_{nullptr};
    MockDac* raw_y_{nullptr};
};

TEST_F(DifferentialGalvoDriverTest, InitializesAndCentersBothAxes) {
    EXPECT_CALL(*raw_x_, is_initialized()).WillOnce(Return(true));
    EXPECT_CALL(*raw_y_, is_initialized()).WillOnce(Return(true));

    std::vector<DacValues> x_writes, y_writes;
    EXPECT_CALL(*raw_x_, write(_)).WillRepeatedly(Invoke(make_recorder(x_writes)));
    EXPECT_CALL(*raw_y_, write(_)).WillRepeatedly(Invoke(make_recorder(y_writes)));

    DifferentialGalvoDriver driver(std::move(mock_dac_x_), std::move(mock_dac_y_));

    EXPECT_TRUE(driver.is_initialized());
    ASSERT_EQ(x_writes.size(), 1u);
    ASSERT_EQ(y_writes.size(), 1u);
    EXPECT_EQ(x_writes[0], kCenter);
    EXPECT_EQ(y_writes[0], kCenter);
}

TEST_F(DifferentialGalvoDriverTest, NullDacLeavesUninitialized) {
    DifferentialGalvoDriver driver(nullptr, std::move(mock_dac_y_));
    EXPECT_FALSE(driver.is_initialized());
}

TEST_F(DifferentialGalvoDriverTest, UninitializedDacLeavesDriverUninitialized) {
    EXPECT_CALL(*raw_x_, is_initialized()).WillOnce(Return(false));

    DifferentialGalvoDriver driver(std::move(mock_dac_x_), std::move(mock_dac_y_));
    EXPECT_FALSE(driver.is_initialized());
}

TEST_F(DifferentialGalvoDriverTest, InitZeroFailureLeavesUninitialized) {
    EXPECT_CALL(*raw_x_, is_initialized()).WillOnce(Return(true));
    EXPECT_CALL(*raw_y_, is_initialized()).WillOnce(Return(true));
    EXPECT_CALL(*raw_x_, write(_))
        .WillOnce(Return(std::unexpected(HardwareError::SpiTransferFailed)));

    DifferentialGalvoDriver driver(std::move(mock_dac_x_), std::move(mock_dac_y_));
    EXPECT_FALSE(driver.is_initialized());
}

TEST_F(DifferentialGalvoDriverTest, SetPositionWritesComplementaryChannels) {
    EXPECT_CALL(*raw_x_, is_initialized()).WillOnce(Return(true));
    EXPECT_CALL(*raw_y_, is_initialized()).WillOnce(Return(true));

    std::vector<DacValues> x_writes, y_writes;
    EXPECT_CALL(*raw_x_, write(_)).WillRepeatedly(Invoke(make_recorder(x_writes)));
    EXPECT_CALL(*raw_y_, write(_)).WillRepeatedly(Invoke(make_recorder(y_writes)));

    DifferentialGalvoDriver driver(std::move(mock_dac_x_), std::move(mock_dac_y_));
    ASSERT_TRUE(driver.is_initialized());

    auto result = driver.set_position(1024, 512);
    EXPECT_TRUE(result.has_value());

    ASSERT_GE(x_writes.size(), 2u);
    ASSERT_GE(y_writes.size(), 2u);
    EXPECT_EQ(x_writes[1], (DacValues{1024, 3071}));
    EXPECT_EQ(y_writes[1], (DacValues{512, 3583}));
}

TEST_F(DifferentialGalvoDriverTest, SetPositionRejectsOutOfRange) {
    EXPECT_CALL(*raw_x_, is_initialized()).WillOnce(Return(true));
    EXPECT_CALL(*raw_y_, is_initialized()).WillOnce(Return(true));

    std::vector<DacValues> x_writes, y_writes;
    EXPECT_CALL(*raw_x_, write(_)).WillRepeatedly(Invoke(make_recorder(x_writes)));
    EXPECT_CALL(*raw_y_, write(_)).WillRepeatedly(Invoke(make_recorder(y_writes)));

    DifferentialGalvoDriver driver(std::move(mock_dac_x_), std::move(mock_dac_y_));
    ASSERT_TRUE(driver.is_initialized());

    const size_t x_before = x_writes.size();
    const size_t y_before = y_writes.size();

    auto result = driver.set_position(4096, 0);
    EXPECT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), HardwareError::DacInvalidValue);

    EXPECT_EQ(x_writes.size(), x_before);
    EXPECT_EQ(y_writes.size(), y_before);
}

TEST_F(DifferentialGalvoDriverTest, SetPositionFailurePropagates) {
    EXPECT_CALL(*raw_x_, is_initialized()).WillOnce(Return(true));
    EXPECT_CALL(*raw_y_, is_initialized()).WillOnce(Return(true));

    std::vector<DacValues> x_writes, y_writes;
    int x_call = 0;
    auto x_action = [&](DacValues v) -> std::expected<void, HardwareError> {
        x_writes.push_back(v);
        ++x_call;
        if (x_call == 2) {
            return std::unexpected(HardwareError::SpiTransferFailed);
        }
        return {};
    };
    EXPECT_CALL(*raw_x_, write(_)).WillRepeatedly(Invoke(x_action));
    EXPECT_CALL(*raw_y_, write(_)).WillRepeatedly(Invoke(make_recorder(y_writes)));

    DifferentialGalvoDriver driver(std::move(mock_dac_x_), std::move(mock_dac_y_));
    ASSERT_TRUE(driver.is_initialized());

    auto result = driver.set_position(1000, 1000);
    EXPECT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), HardwareError::SpiTransferFailed);

    ASSERT_GE(y_writes.size(), 1u);
    EXPECT_EQ(y_writes[0], kCenter);
}

TEST_F(DifferentialGalvoDriverTest, ZeroCentersAtMidpoint) {
    EXPECT_CALL(*raw_x_, is_initialized()).WillOnce(Return(true));
    EXPECT_CALL(*raw_y_, is_initialized()).WillOnce(Return(true));

    std::vector<DacValues> x_writes, y_writes;
    EXPECT_CALL(*raw_x_, write(_)).WillRepeatedly(Invoke(make_recorder(x_writes)));
    EXPECT_CALL(*raw_y_, write(_)).WillRepeatedly(Invoke(make_recorder(y_writes)));

    DifferentialGalvoDriver driver(std::move(mock_dac_x_), std::move(mock_dac_y_));
    ASSERT_TRUE(driver.is_initialized());

    auto result = driver.zero();
    EXPECT_TRUE(result.has_value());

    for (const auto& v : x_writes) {
        EXPECT_EQ(v, kCenter);
    }
    for (const auto& v : y_writes) {
        EXPECT_EQ(v, kCenter);
    }
    ASSERT_GE(x_writes.size(), 2u);
    ASSERT_GE(y_writes.size(), 2u);
}
