#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <memory>

#include "mocks/mock_gpio.h"
#include "safety/e_stop.h"
#include "core/error.h"

using namespace testing;

class EStopTest : public Test {
protected:
    void SetUp() override {
        mock_gpio_ = std::make_unique<NiceMock<MockGpio>>();
    }

    std::unique_ptr<MockGpio> mock_gpio_;
};

TEST_F(EStopTest, InitializeSetsDirectionInput) {
    EXPECT_CALL(*mock_gpio_, set_direction_input())
        .WillOnce(Return(std::expected<void, HardwareError>{}));

    EStop e_stop(std::move(mock_gpio_), 3);
    auto result = e_stop.initialize();
    EXPECT_TRUE(result.has_value());
    EXPECT_TRUE(e_stop.is_initialized());
}

TEST_F(EStopTest, InitializeFailureReturnsError) {
    EXPECT_CALL(*mock_gpio_, set_direction_input())
        .WillOnce(Return(
            std::unexpected(HardwareError::GpioOpenFailed)));

    EStop e_stop(std::move(mock_gpio_), 3);
    auto result = e_stop.initialize();
    EXPECT_FALSE(result.has_value());
    EXPECT_FALSE(e_stop.is_initialized());
}

TEST_F(EStopTest, ReleasedInitially) {
    EXPECT_CALL(*mock_gpio_, set_direction_input())
        .WillOnce(Return(std::expected<void, HardwareError>{}));

    EStop e_stop(std::move(mock_gpio_), 3);
    e_stop.initialize();

    EXPECT_FALSE(e_stop.is_pressed());
}

TEST_F(EStopTest, ActiveLowDebouncedPress) {
    EXPECT_CALL(*mock_gpio_, set_direction_input())
        .WillOnce(Return(std::expected<void, HardwareError>{}));

    MockGpio* raw_ptr = mock_gpio_.get();
    EStop e_stop(std::move(mock_gpio_), 3);
    e_stop.initialize();

    EXPECT_CALL(*raw_ptr, read())
        .Times(3)
        .WillRepeatedly(Return(false));

    e_stop.update();
    EXPECT_FALSE(e_stop.is_pressed());
    e_stop.update();
    EXPECT_FALSE(e_stop.is_pressed());
    e_stop.update();
    EXPECT_TRUE(e_stop.is_pressed());
}

TEST_F(EStopTest, ActiveLowDebouncedRelease) {
    EXPECT_CALL(*mock_gpio_, set_direction_input())
        .WillOnce(Return(std::expected<void, HardwareError>{}));

    MockGpio* raw_ptr = mock_gpio_.get();
    EStop e_stop(std::move(mock_gpio_), 3);
    e_stop.initialize();

    EXPECT_CALL(*raw_ptr, read())
        .Times(3)
        .WillRepeatedly(Return(false));

    e_stop.update();
    e_stop.update();
    e_stop.update();
    EXPECT_TRUE(e_stop.is_pressed());

    EXPECT_CALL(*raw_ptr, read())
        .Times(3)
        .WillRepeatedly(Return(true));

    e_stop.update();
    EXPECT_TRUE(e_stop.is_pressed());
    e_stop.update();
    EXPECT_TRUE(e_stop.is_pressed());
    e_stop.update();
    EXPECT_FALSE(e_stop.is_pressed());
}

TEST_F(EStopTest, SingleGlitchDoesNotChangeState) {
    EXPECT_CALL(*mock_gpio_, set_direction_input())
        .WillOnce(Return(std::expected<void, HardwareError>{}));

    MockGpio* raw_ptr = mock_gpio_.get();
    EStop e_stop(std::move(mock_gpio_), 4);
    e_stop.initialize();

    EXPECT_CALL(*raw_ptr, read())
        .Times(4)
        .WillRepeatedly(Return(false));

    for (int i = 0; i < 4; ++i) {
        e_stop.update();
    }
    EXPECT_TRUE(e_stop.is_pressed());

    EXPECT_CALL(*raw_ptr, read())
        .Times(1)
        .WillOnce(Return(true));
    e_stop.update();
    EXPECT_TRUE(e_stop.is_pressed());

    EXPECT_CALL(*raw_ptr, read())
        .Times(4)
        .WillRepeatedly(Return(false));

    for (int i = 0; i < 4; ++i) {
        e_stop.update();
    }
    EXPECT_TRUE(e_stop.is_pressed());
}

TEST_F(EStopTest, UpdateWithoutInitDoesNotCrash) {
    EStop e_stop(std::move(mock_gpio_), 3);
    e_stop.update();
    EXPECT_FALSE(e_stop.is_pressed());
}

TEST_F(EStopTest, GpioReadFailureForcesPressed) {
    EXPECT_CALL(*mock_gpio_, set_direction_input())
        .WillOnce(Return(std::expected<void, HardwareError>{}));

    MockGpio* raw_ptr = mock_gpio_.get();
    EStop e_stop(std::move(mock_gpio_), 3);
    e_stop.initialize();

    EXPECT_CALL(*raw_ptr, read())
        .WillOnce(Return(std::unexpected(HardwareError::GpioReadFailed)));

    e_stop.update();
    EXPECT_TRUE(e_stop.is_pressed());
}
