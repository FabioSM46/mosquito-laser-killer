#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <optional>
#include <vector>

#include "core/types.h"
#include "vision/target_selector.h"

using namespace testing;

namespace {

auto make_target(uint32_t id, double z, bool engageable = true) -> TrackedTarget {
    TrackedTarget t;
    t.id = id;
    t.position = Point3D{0.0, 0.0, z};
    t.confirmed = true;
    t.engageable = engageable;
    return t;
}

class TargetSelectorTest : public Test {
protected:
    TargetSelector selector_;
};

TEST_F(TargetSelectorTest, NoTargetsYieldsNothing) {
    EXPECT_FALSE(selector_.select({}).has_value());
}

TEST_F(TargetSelectorTest, NonEngageableTargetsAreNeverSelected) {
    // Tentative tracks and stationary glints are not targets, whatever their
    // depth.
    EXPECT_FALSE(selector_.select({make_target(1, 0.6, false)}).has_value());
}

TEST_F(TargetSelectorTest, SingleEngageableTargetIsSelected) {
    const auto chosen = selector_.select({make_target(7, 0.8)});
    ASSERT_TRUE(chosen.has_value());
    EXPECT_EQ(chosen->id, 7u);
}

TEST_F(TargetSelectorTest, NearestEngageableTargetIsSelected) {
    const auto chosen = selector_.select({make_target(1, 0.9), make_target(2, 0.6)});
    ASSERT_TRUE(chosen.has_value());
    EXPECT_EQ(chosen->id, 2u);
}

// The anti-ping-pong property: while the engaged target stays engageable, a
// nearer target appearing must NOT steal the engagement. One laser plus a 10 s
// cooldown means switching costs an engagement; jitter costs every engagement.
TEST_F(TargetSelectorTest, EngagedTargetIsStickyAgainstNearerChallengers) {
    ASSERT_EQ(selector_.select({make_target(1, 0.9)})->id, 1u);

    const auto second = selector_.select({make_target(1, 0.9), make_target(2, 0.6)});
    ASSERT_TRUE(second.has_value());
    EXPECT_EQ(second->id, 1u);
}

TEST_F(TargetSelectorTest, LosingTheEngagedTargetFallsBackToNearestRemaining) {
    ASSERT_EQ(selector_.select({make_target(1, 0.9), make_target(2, 0.6)})->id, 2u);

    // Target 2 is gone (killed or escaped); target 1 and a new target 3 remain.
    const auto fallback = selector_.select({make_target(1, 0.9), make_target(3, 0.7)});
    ASSERT_TRUE(fallback.has_value());
    EXPECT_EQ(fallback->id, 3u);
}

TEST_F(TargetSelectorTest, EngagedTargetTurningNonEngageableReleasesIt) {
    ASSERT_EQ(selector_.select({make_target(1, 0.8)})->id, 1u);

    // Same track, no longer flying plausibly (e.g. it landed). The selector
    // must let go rather than hold aim on a non-target.
    const auto out = selector_.select({make_target(1, 0.8, false)});
    EXPECT_FALSE(out.has_value());

    // And the stickiness is gone with it: a fresh target is picked up next.
    EXPECT_EQ(selector_.select({make_target(2, 0.7)})->id, 2u);
}

TEST_F(TargetSelectorTest, ResetForgetsTheEngagedTarget) {
    ASSERT_EQ(selector_.select({make_target(1, 0.9)})->id, 1u);
    selector_.reset();

    // Without the memory of target 1, the nearer target 2 wins.
    EXPECT_EQ(selector_.select({make_target(1, 0.9), make_target(2, 0.6)})->id, 2u);
}

}   // namespace
