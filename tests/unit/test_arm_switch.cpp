#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <memory>

#include "mocks/mock_gpio.h"
#include "safety/arm_switch.h"
#include "core/error.h"

using namespace testing;

class ArmSwitchTest : public Test {
protected:
    void SetUp() override {
        mock_gpio_ = std::make_unique<NiceMock<MockGpio>>();
    }

    std::unique_ptr<MockGpio> mock_gpio_;
};

TEST_F(ArmSwitchTest, InitializeSetsDirectionInput) {
    EXPECT_CALL(*mock_gpio_, set_direction_input())
        .WillOnce(Return(std::expected<void, HardwareError>{}));

    ArmSwitch arm_switch(std::move(mock_gpio_), 3);
    auto result = arm_switch.initialize();
    EXPECT_TRUE(result.has_value());
}

TEST_F(ArmSwitchTest, InitializeFailureReturnsError) {
    EXPECT_CALL(*mock_gpio_, set_direction_input())
        .WillOnce(Return(
            std::unexpected(HardwareError::GpioOpenFailed)));

    ArmSwitch arm_switch(std::move(mock_gpio_), 3);
    auto result = arm_switch.initialize();
    EXPECT_FALSE(result.has_value());
}

TEST_F(ArmSwitchTest, IsArmedReturnsFalseInitially) {
    EXPECT_CALL(*mock_gpio_, set_direction_input())
        .WillOnce(Return(std::expected<void, HardwareError>{}));

    ArmSwitch arm_switch(std::move(mock_gpio_), 3);
    arm_switch.initialize();

    EXPECT_FALSE(arm_switch.is_armed());
}

TEST_F(ArmSwitchTest, DebounceHighToArmed) {
    EXPECT_CALL(*mock_gpio_, set_direction_input())
        .WillOnce(Return(std::expected<void, HardwareError>{}));

    MockGpio* raw_ptr = mock_gpio_.get();
    ArmSwitch arm_switch(std::move(mock_gpio_), 3);
    arm_switch.initialize();

    EXPECT_CALL(*raw_ptr, read())
        .Times(3)
        .WillRepeatedly(Return(true));

    arm_switch.update();
    EXPECT_FALSE(arm_switch.is_armed());
    arm_switch.update();
    EXPECT_FALSE(arm_switch.is_armed());
    arm_switch.update();
    EXPECT_TRUE(arm_switch.is_armed());
}

TEST_F(ArmSwitchTest, DebounceLowToDisarmed) {
    EXPECT_CALL(*mock_gpio_, set_direction_input())
        .WillOnce(Return(std::expected<void, HardwareError>{}));

    MockGpio* raw_ptr = mock_gpio_.get();
    ArmSwitch arm_switch(std::move(mock_gpio_), 3);
    arm_switch.initialize();

    EXPECT_CALL(*raw_ptr, read())
        .Times(3)
        .WillRepeatedly(Return(true));

    arm_switch.update();
    arm_switch.update();
    arm_switch.update();
    EXPECT_TRUE(arm_switch.is_armed());

    EXPECT_CALL(*raw_ptr, read())
        .Times(3)
        .WillRepeatedly(Return(false));

    arm_switch.update();
    EXPECT_TRUE(arm_switch.is_armed());
    arm_switch.update();
    EXPECT_TRUE(arm_switch.is_armed());
    arm_switch.update();
    EXPECT_FALSE(arm_switch.is_armed());
}

TEST_F(ArmSwitchTest, SingleGlitchDoesNotChangeState) {
    EXPECT_CALL(*mock_gpio_, set_direction_input())
        .WillOnce(Return(std::expected<void, HardwareError>{}));

    MockGpio* raw_ptr = mock_gpio_.get();
    ArmSwitch arm_switch(std::move(mock_gpio_), 4);
    arm_switch.initialize();

    EXPECT_CALL(*raw_ptr, read())
        .Times(4)
        .WillRepeatedly(Return(true));

    for (int i = 0; i < 4; ++i) {
        arm_switch.update();
    }
    EXPECT_TRUE(arm_switch.is_armed());

    EXPECT_CALL(*raw_ptr, read())
        .Times(1)
        .WillOnce(Return(false));
    arm_switch.update();
    EXPECT_TRUE(arm_switch.is_armed());

    EXPECT_CALL(*raw_ptr, read())
        .Times(4)
        .WillRepeatedly(Return(true));

    for (int i = 0; i < 4; ++i) {
        arm_switch.update();
    }
    EXPECT_TRUE(arm_switch.is_armed());
}

TEST_F(ArmSwitchTest, UpdateWithoutInitDoesNotCrash) {
    ArmSwitch arm_switch(std::move(mock_gpio_), 3);
    arm_switch.update();
    EXPECT_FALSE(arm_switch.is_armed());
}
