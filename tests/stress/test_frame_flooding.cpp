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

TEST(FrameFloodingStressTest, QueueDoesNotGrowUnboundedUnderFlood) {
    ThreadSafeQueue<int> q;
    std::atomic<bool> stop{false};
    std::atomic<size_t> max_size{0};

    std::jthread producer([&] {
        int counter = 0;
        while (!stop.load()) {
            q.push(counter++);
            std::this_thread::sleep_for(10us);
        }
    });

    std::jthread consumer([&] {
        while (!stop.load()) {
            auto frames = q.drain_all();
            max_size.store(std::max(max_size.load(), frames.size()));
            std::this_thread::sleep_for(8ms);
        }
    });

    std::this_thread::sleep_for(200ms);
    stop.store(true);
    producer.join();
    consumer.join();

    EXPECT_LT(max_size.load(), 500u);
}
