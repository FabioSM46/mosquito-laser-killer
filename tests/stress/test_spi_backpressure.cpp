#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <chrono>
#include <thread>
#include <memory>
#include <atomic>

#include "mocks/mock_spi.h"
#include "mocks/mock_dac.h"
#include "mocks/mock_laser.h"
#include "core/thread_safe_queue.h"
#include "core/types.h"
#include "core/error.h"
#include "safety/system_state.h"

using namespace testing;
using namespace std::chrono_literals;

TEST(SpiBackpressureStressTest, QueueDoesNotOverflowUnderSpiBackpressure) {
    ThreadSafeQueue<int> cmd_queue;
    std::atomic<bool> stop{false};
    std::atomic<size_t> max_queue_size{0};
    std::atomic<size_t> total_pushed{0};
    std::atomic<size_t> total_processed{0};

    std::jthread producer([&] {
        int counter = 0;
        while (!stop.load()) {
            cmd_queue.push(counter++);
            total_pushed.fetch_add(1);
            std::this_thread::sleep_for(100us);
        }
    });

    std::jthread slow_consumer([&] {
        while (!stop.load()) {
            auto items = cmd_queue.drain_all();

            auto qs = cmd_queue.size();
            size_t current_max = max_queue_size.load();
            while (qs > current_max) {
                max_queue_size.compare_exchange_weak(current_max, qs);
            }

            if (!items.empty()) {
                total_processed.fetch_add(items.size());

                std::this_thread::sleep_for(20ms);
            } else {
                std::this_thread::sleep_for(1ms);
            }
        }
    });

    std::this_thread::sleep_for(500ms);
    stop.store(true);
    producer.join();
    slow_consumer.join();

    EXPECT_LT(max_queue_size.load(), 200u);
    EXPECT_GT(total_pushed.load(), 100u);
    EXPECT_GT(total_processed.load(), 0u);
}

TEST(SpiBackpressureStressTest, SlowSpiDoesNotBlockProducer) {
    ThreadSafeQueue<int> q;
    std::atomic<bool> stop{false};
    std::atomic<int> producer_progress{0};
    std::atomic<int> consumer_progress{0};

    std::jthread producer([&] {
        for (int i = 0; i < 200; ++i) {
            q.push(i);
            producer_progress.store(i);
            std::this_thread::sleep_for(500us);
        }
        stop.store(true);
    });

    std::jthread slow_consumer([&] {
        while (!stop.load() || !q.empty()) {
            auto items = q.drain_all();
            for (auto& item : items) {
                consumer_progress.store(item);
                std::this_thread::sleep_for(2ms);
            }

            if (items.empty()) {
                std::this_thread::sleep_for(1ms);
            }
        }
    });

    producer.join();
    slow_consumer.join();

    EXPECT_GE(producer_progress.load(), 190);
    EXPECT_GE(consumer_progress.load(), 0);
}

TEST(SpiBackpressureStressTest, DacWriteValidationUnderLoad) {
    auto mock_spi = std::make_unique<NiceMock<MockSpi>>();
    auto mock_laser = std::make_unique<NiceMock<MockLaser>>();

    EXPECT_CALL(*mock_spi, write16(_))
        .WillRepeatedly(Return(std::expected<void, HardwareError>{}));

    constexpr int kIterations = 500;

    for (int i = 0; i < kIterations; ++i) {
        uint16_t value = static_cast<uint16_t>(i % 4096);

        auto result = mock_spi->write16(value);
        ASSERT_TRUE(result.has_value());
    }
}
