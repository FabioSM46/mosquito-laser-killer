#include <gtest/gtest.h>

#include "hal/camera_impl.h"

// CameraImpl itself talks to real V4L2 and has no seam below the ICamera
// interface, so the capture path is exercised on hardware only (see AGENTS.md
// §6.2). What IS unit-testable is the exposure unit conversion: the V4L2
// EXPOSURE_ABSOLUTE control is denominated in 100 µs units, and passing the
// configured microseconds straight through requested a 100× longer exposure
// (156 µs became 15.6 ms — longer than a whole frame at 120/210 fps).
namespace {

TEST(ExposureConversionTest, ConfiguredDefaultMapsToTwoUnits) {
    // 156 µs → nearest 100 µs unit is 2 (200 µs effective).
    EXPECT_EQ(exposure_us_to_v4l2_units(156), 2);
}

TEST(ExposureConversionTest, RoundsToNearestUnit) {
    EXPECT_EQ(exposure_us_to_v4l2_units(100), 1);
    EXPECT_EQ(exposure_us_to_v4l2_units(149), 1);
    EXPECT_EQ(exposure_us_to_v4l2_units(150), 2);
    EXPECT_EQ(exposure_us_to_v4l2_units(1000), 10);
    EXPECT_EQ(exposure_us_to_v4l2_units(10000), 100);
}

TEST(ExposureConversionTest, NeverRequestsZeroOrNegativeUnits) {
    // A request of 0 is driver-defined; the floor keeps the shortest real
    // exposure instead. Garbage negative input also lands on the floor.
    EXPECT_EQ(exposure_us_to_v4l2_units(49), 1);
    EXPECT_EQ(exposure_us_to_v4l2_units(1), 1);
    EXPECT_EQ(exposure_us_to_v4l2_units(0), 1);
    EXPECT_EQ(exposure_us_to_v4l2_units(-500), 1);
}

}   // namespace
