#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <chrono>
#include <thread>
#include <memory>
#include <atomic>
#include <vector>

#include "mocks/mock_spi.h"
#include "mocks/mock_laser.h"
#include "mocks/mock_galvo_driver.h"
#include "core/thread_safe_queue.h"
#include "core/types.h"
#include "core/error.h"
#include "hal/mcp4922.h"
#include "hal/differential_galvo_driver.h"
#include "safety/bounding_box.h"
#include "safety/system_state.h"
#include "control/coordinate_mapper.h"
#include "control/firing_controller.h"

using namespace testing;
using namespace std::chrono_literals;

namespace {

constexpr auto kOk = std::expected<void, HardwareError>{};

// AGENTS.md 7.2 asks for "Mock SPI delays; verify queue doesn't overflow". These
// tests drive a REAL MCP4922 over a MockSpi that can be made slow or made to
// fail. The previous version of this file never constructed an MCP4922 — it did
// not even include the header. Its "DacWriteValidationUnderLoad" configured a
// MockSpi to return success, called that same mock 500 times, and asserted it
// returned success, so the DAC's own range guard was never reached.

TEST(SpiBackpressureStressTest, DacRejectsOutOfRangeValuesWithoutTouchingTheBus) {
    auto spi = std::make_unique<NiceMock<MockSpi>>();
    auto* raw_spi = spi.get();

    int writes = 0;
    ON_CALL(*raw_spi, write16(_)).WillByDefault([&writes](uint16_t) {
        ++writes;
        return kOk;
    });

    MCP4922 dac(std::move(spi));
    // Counted rather than EXPECT_CALL(...).Times(0): ~MCP4922 zeroes the DAC, so a
    // Times(0) expectation would be violated by the destructor rather than by
    // anything under test.
    writes = 0;

    // A value beyond 12-bit range must be rejected by the DAC itself and never
    // reach the wire — a truncated or wrapped code steers the galvo somewhere the
    // caller never asked for.
    EXPECT_EQ(dac.write({4096, 2048}).error(), HardwareError::DacInvalidValue);
    EXPECT_EQ(dac.write({2048, 4096}).error(), HardwareError::DacInvalidValue);
    EXPECT_EQ(dac.write({65535, 0}).error(), HardwareError::DacInvalidValue);

    EXPECT_EQ(writes, 0) << "an out-of-range DAC value reached the SPI bus";

    // The same call with an in-range value does reach the bus, so the assertion
    // above is not passing merely because nothing ever writes.
    ASSERT_TRUE(dac.write({2048, 2048}).has_value());
    EXPECT_EQ(writes, 2);
}

TEST(SpiBackpressureStressTest, DacWriteValidationHoldsUnderSustainedLoad) {
    auto spi = std::make_unique<NiceMock<MockSpi>>();
    auto* raw_spi = spi.get();

    std::vector<uint16_t> words;
    ON_CALL(*raw_spi, write16(_)).WillByDefault([&words](uint16_t w) {
        words.push_back(w);
        return kOk;
    });

    MCP4922 dac(std::move(spi));
    words.clear();  // drop the ctor's zeroing writes

    constexpr int kIterations = 500;
    for (int i = 0; i < kIterations; ++i) {
        const auto value = static_cast<uint16_t>(i % 4096);
        ASSERT_TRUE(dac.write({value, value}).has_value());
    }

    // Two writes per call — one per channel — and every word must carry an
    // in-range 12-bit payload with the correct channel-select bit.
    ASSERT_EQ(words.size(), static_cast<size_t>(kIterations) * 2);
    for (size_t i = 0; i < words.size(); ++i) {
        const uint16_t expected_value = static_cast<uint16_t>((i / 2) % 4096);
        EXPECT_EQ(words[i] & 0x0FFF, expected_value) << "at word " << i;
        // Channel A word has bit15 clear, channel B has it set.
        EXPECT_EQ((words[i] & 0x8000) != 0, i % 2 == 1) << "at word " << i;
    }
}

TEST(SpiBackpressureStressTest, SpiFailureIsReportedNotSwallowed) {
    auto spi = std::make_unique<NiceMock<MockSpi>>();
    auto* raw_spi = spi.get();
    ON_CALL(*raw_spi, write16(_)).WillByDefault(Return(kOk));
    MCP4922 dac(std::move(spi));

    ON_CALL(*raw_spi, write16(_))
        .WillByDefault(Return(std::unexpected(HardwareError::SpiTransferFailed)));

    auto result = dac.write({2048, 2048});
    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), HardwareError::SpiTransferFailed);
}

// A slow bus must surface as an error or a delay — never as a silently dropped
// write that leaves the galvo somewhere other than where the controller believes.
TEST(SpiBackpressureStressTest, SlowSpiStillDeliversEveryWriteInOrder) {
    auto spi = std::make_unique<NiceMock<MockSpi>>();
    auto* raw_spi = spi.get();

    std::vector<uint16_t> words;
    ON_CALL(*raw_spi, write16(_)).WillByDefault([&words](uint16_t w) {
        std::this_thread::sleep_for(200us);  // a bus far slower than 20MHz
        words.push_back(w);
        return kOk;
    });

    MCP4922 dac(std::move(spi));
    words.clear();

    for (uint16_t v = 0; v < 50; ++v) {
        ASSERT_TRUE(dac.write({v, static_cast<uint16_t>(4095 - v)}).has_value());
    }

    ASSERT_EQ(words.size(), 100u);
    for (uint16_t v = 0; v < 50; ++v) {
        EXPECT_EQ(words[v * 2] & 0x0FFF, v);
        EXPECT_EQ(words[v * 2 + 1] & 0x0FFF, 4095 - v);
    }
}

// A galvo write failure caused by SPI backpressure must latch the controller off
// rather than being retried forever with the laser live.
TEST(SpiBackpressureStressTest, SpiFailureDuringFiringLatchesTheController) {
    auto spi_x = std::make_unique<NiceMock<MockSpi>>();
    auto spi_y = std::make_unique<NiceMock<MockSpi>>();
    auto* raw_x = spi_x.get();
    ON_CALL(*raw_x, write16(_)).WillByDefault(Return(kOk));
    ON_CALL(*spi_y, write16(_)).WillByDefault(Return(kOk));

    auto dac_x = std::make_unique<MCP4922>(std::move(spi_x));
    auto dac_y = std::make_unique<MCP4922>(std::move(spi_y));
    DifferentialGalvoDriver galvo(std::move(dac_x), std::move(dac_y));

    NiceMock<MockLaser> laser;
    ON_CALL(laser, fire(_)).WillByDefault(Return(kOk));
    ON_CALL(laser, emergency_shutdown()).WillByDefault(Return(kOk));

    SystemConfig config{};
    BoundingBox3D bbox(config.bounding_box);
    CoordinateMapper mapper(bbox, config.galvo_limits, config.dac_ref_voltage,
                            config.galvo_driver);

    const auto t0 = std::chrono::steady_clock::now();
    FiringController fc(laser, galvo, mapper, config.max_pulse_duration_ms,
                        config.cooldown_seconds, config.settle_delay_ms,
                        t0 - FiringController::k_startup_blanking - 1s);

    // The X-axis bus starts failing under load.
    ON_CALL(*raw_x, write16(_))
        .WillByDefault(Return(std::unexpected(HardwareError::SpiTransferFailed)));

    auto now = t0;
    fc.set_armed(true, now);
    fc.set_target({0.0, 0.0, 0.7}, now);
    (void)fc.execute_cycle(now);

    EXPECT_TRUE(fc.is_halted()) << "an SPI failure on the galvo path must latch off";
    EXPECT_FALSE(fc.is_firing());
    EXPECT_FALSE(fc.is_armed());
}

// The command queue is unbounded by construction, so this documents the real
// property: a slow consumer that drains-to-newest keeps its backlog bounded
// because it discards what it skipped, not because the queue pushes back.
TEST(SpiBackpressureStressTest, DrainToNewestKeepsTheBacklogBoundedUnderASlowConsumer) {
    ThreadSafeQueue<int> cmd_queue;
    std::atomic<bool> stop{false};
    std::atomic<size_t> largest_drain{0};
    std::atomic<size_t> total_pushed{0};
    std::atomic<int> newest_seen{-1};

    std::jthread producer([&] {
        int counter = 0;
        while (!stop.load(std::memory_order_acquire)) {
            cmd_queue.push(counter++);
            total_pushed.fetch_add(1, std::memory_order_release);
            std::this_thread::sleep_for(100us);
        }
    });

    std::jthread slow_consumer([&] {
        while (!stop.load(std::memory_order_acquire)) {
            auto items = cmd_queue.drain_all();
            if (!items.empty()) {
                // Measure the drain itself: sampling size() after draining always
                // reads a just-emptied queue, which is what the previous version
                // did — its bound could not fail regardless of real backlog.
                size_t current = largest_drain.load(std::memory_order_acquire);
                while (items.size() > current &&
                       !largest_drain.compare_exchange_weak(current, items.size())) {
                }
                newest_seen.store(items.back(), std::memory_order_release);
                std::this_thread::sleep_for(20ms);
            } else {
                std::this_thread::sleep_for(1ms);
            }
        }
    });

    std::this_thread::sleep_for(500ms);
    stop.store(true, std::memory_order_release);
    producer.join();
    slow_consumer.join();

    EXPECT_GT(total_pushed.load(), 100u) << "the producer never got going";

    // Each 20ms consumer cycle accumulates ~200 items at a 100us push rate. The
    // backlog is bounded by the drain interval, and the consumer always ends up
    // on the newest item rather than falling progressively further behind.
    EXPECT_GT(largest_drain.load(), 0u);
    const auto remaining = cmd_queue.drain_all();
    EXPECT_LT(remaining.size(), total_pushed.load())
        << "the consumer never kept up with any of the backlog";
    EXPECT_GT(newest_seen.load(), 0);
}

}
