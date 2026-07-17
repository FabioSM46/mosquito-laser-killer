#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <chrono>
#include <thread>
#include <atomic>
#include <vector>
#include <algorithm>

#include "core/thread_safe_queue.h"
#include "core/types.h"

using namespace testing;
using namespace std::chrono_literals;

TEST(FrameFloodingStressTest, OnlyNewestFrameProcessed) {
    ThreadSafeQueue<StereoFrame> q;
    std::atomic<uint64_t> newest_received{0};
    std::atomic<uint64_t> frames_dropped{0};

    std::jthread consumer([&](std::stop_token st) {
        while (!st.stop_requested()) {
            auto frames = q.drain_all();
            if (frames.empty()) {
                std::this_thread::sleep_for(1ms);
                continue;
            }

            if (frames.size() > 1) {
                frames_dropped.fetch_add(frames.size() - 1);
            }

            newest_received.store(frames.back().frame_id);
        }
    });

    uint64_t last_pushed = 0;
    for (int burst = 0; burst < 5; ++burst) {
        for (int i = 0; i < 100; ++i) {
            StereoFrame f;
            f.frame_id = last_pushed++;
            q.push(std::move(f));
        }
        std::this_thread::sleep_for(5ms);
    }

    std::this_thread::sleep_for(50ms);
    consumer.request_stop();
    consumer.join();

    EXPECT_GT(frames_dropped.load(), 0);
    EXPECT_EQ(newest_received.load(), last_pushed - 1);
}

TEST(FrameFloodingStressTest, DrainAllPreservesOrderUnderBackpressure) {
    ThreadSafeQueue<int> q;

    std::jthread producer([&] {
        for (int i = 0; i < 1000; ++i) {
            q.push(i);
        }
    });

    producer.join();

    auto drained = q.drain_all();
    ASSERT_EQ(drained.size(), 1000);

    for (int i = 0; i < 1000; ++i) {
        EXPECT_EQ(drained[i], i);
    }
}

// The property that matters under a sustained flood is freshness, not queue size.
// ThreadSafeQueue is an unbounded std::queue with no push-back and no capacity, so
// there is no growth guard to test: the backlog is bounded only because the
// consumer discards what it skipped. Asserting a size ceiling would be asserting
// an accident of timing — the previous version required max_size < 500 while the
// nominal rates (10us push, 8ms drain) imply ~800 items per drain. It passed only
// because sleep_for(10us) overshoots to ~50-100us on a typical scheduler, and
// would flip on a tickless high-resolution kernel.
//
// Freshness is measured after the producer has stopped, so the producer cannot
// run ahead during the measurement and be mistaken for consumer lag.
TEST(FrameFloodingStressTest, ConsumerEndsCurrentAndDropsTheBacklogUnderSustainedFlood) {
    ThreadSafeQueue<int> q;
    std::atomic<bool> stop_producer{false};
    std::atomic<bool> stop_consumer{false};
    std::atomic<int> latest_produced{-1};
    std::atomic<int> newest_consumed{-1};
    std::atomic<int> drain_count{0};

    std::jthread producer([&] {
        int counter = 0;
        while (!stop_producer.load(std::memory_order_acquire)) {
            q.push(counter);
            latest_produced.store(counter, std::memory_order_release);
            ++counter;
            std::this_thread::sleep_for(10us);
        }
    });

    std::jthread consumer([&] {
        while (!stop_consumer.load(std::memory_order_acquire)) {
            auto frames = q.drain_all();
            if (!frames.empty()) {
                // Latency over throughput: handle only the newest, drop the rest.
                newest_consumed.store(frames.back(), std::memory_order_release);
                drain_count.fetch_add(1, std::memory_order_release);
            }
            std::this_thread::sleep_for(8ms);
        }
    });

    std::this_thread::sleep_for(200ms);

    // Stop the producer first and let the consumer drain what is left, so the
    // final comparison is against a value that can no longer move.
    stop_producer.store(true, std::memory_order_release);
    producer.join();
    std::this_thread::sleep_for(50ms);
    stop_consumer.store(true, std::memory_order_release);
    consumer.join();

    const int produced = latest_produced.load();
    ASSERT_GT(produced, 100) << "the producer never flooded";

    // The consumer ends up exactly current: one drain takes the entire backlog.
    EXPECT_EQ(newest_consumed.load(), produced)
        << "the consumer did not end on the newest item";

    // And it got there by dropping, not by processing everything: far fewer drains
    // than items. A one-at-a-time pop would need one iteration per item and would
    // still be grinding through the backlog here.
    EXPECT_LT(drain_count.load(), produced / 4)
        << "the consumer processed roughly every item instead of skipping to the "
           "newest — drain-to-newest is not in effect";
}
