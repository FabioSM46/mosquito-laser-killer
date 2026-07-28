#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <string>

#include "core/config_loader.h"
#include "core/types.h"

namespace {

class ConfigLoaderTest : public ::testing::Test {
protected:
    // Unique per-test file under the gtest temp dir, removed on teardown.
    auto write_config(const std::string& body) -> std::string {
        path_ = std::filesystem::path(::testing::TempDir()) /
                (std::string("mlk_config_") +
                 ::testing::UnitTest::GetInstance()->current_test_info()->name() +
                 ".yaml");
        std::ofstream out(path_);
        out << body;
        out.close();
        return path_.string();
    }

    void TearDown() override {
        if (!path_.empty()) {
            std::error_code ec;
            std::filesystem::remove(path_, ec);
        }
    }

    std::filesystem::path path_{};
};

TEST_F(ConfigLoaderTest, MissingFileFailsClosed) {
    auto result = load_config("/nonexistent/mosquito/system_config.yaml");

    ASSERT_FALSE(result.has_value());
    // The error must name the path that was attempted: a wrong working
    // directory has to be diagnosable from the log alone.
    EXPECT_NE(result.error().find("/nonexistent/mosquito/system_config.yaml"),
              std::string::npos);
}

TEST_F(ConfigLoaderTest, MalformedValueFailsClosedNotPartial) {
    // The bad value sits mid-file, after a key that WOULD have parsed. The old
    // loader kept the early keys and silently defaulted everything after the
    // throw — a mixed config nobody wrote. The contract is all-or-nothing.
    auto path = write_config(
        "settle_delay_ms: 9.5\n"
        "cooldown_seconds: not_a_number\n"
        "max_pulse_duration_ms: 50.0\n");

    auto result = load_config(path);

    ASSERT_FALSE(result.has_value());
}

TEST_F(ConfigLoaderTest, ExplicitNullValueFailsClosed) {
    // `key:` with nothing after it is a defined null node; the operator
    // clearly meant to set the value, so treating it as "absent" would run
    // with a default they never chose.
    auto path = write_config("cooldown_seconds:\n");

    auto result = load_config(path);

    ASSERT_FALSE(result.has_value());
}

TEST_F(ConfigLoaderTest, ValidFileOverridesListedKeys) {
    auto path = write_config(
        "settle_delay_ms: 7.5\n"
        "bounding_box:\n"
        "  z_max: 0.9\n"
        "tracking:\n"
        "  confirm_hits: 5\n");

    auto result = load_config(path);

    ASSERT_TRUE(result.has_value());
    EXPECT_DOUBLE_EQ(result->settle_delay_ms, 7.5);
    EXPECT_DOUBLE_EQ(result->bounding_box.z_max, 0.9);
    EXPECT_EQ(result->tracking.confirm_hits, 5);
}

TEST_F(ConfigLoaderTest, AbsentKeysKeepTypesHDefaults) {
    auto path = write_config("settle_delay_ms: 7.5\n");

    auto result = load_config(path);

    ASSERT_TRUE(result.has_value());
    const SystemConfig defaults{};
    EXPECT_DOUBLE_EQ(result->cooldown_seconds, defaults.cooldown_seconds);
    EXPECT_DOUBLE_EQ(result->max_pulse_duration_ms,
                     defaults.max_pulse_duration_ms);
    EXPECT_EQ(result->tracking.max_tracks, defaults.tracking.max_tracks);
    EXPECT_DOUBLE_EQ(result->detection.background_learning_rate,
                     defaults.detection.background_learning_rate);
}

}   // namespace
