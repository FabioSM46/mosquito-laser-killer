#include <gtest/gtest.h>
#include <algorithm>
#include <chrono>
#include <thread>
#include <vector>
#include <atomic>

#include "core/thread_safe_queue.h"

using namespace testing;
using namespace std::chrono_literals;

TEST(ThreadSafeQueueTest, PushAndPopSingleItem) {
    ThreadSafeQueue<int> q;
    q.push(42);
    auto result = q.pop(100ms);
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result.value(), 42);
}

TEST(ThreadSafeQueueTest, PopEmptyReturnsNullopt) {
    ThreadSafeQueue<int> q;
    auto result = q.pop(10ms);
    EXPECT_FALSE(result.has_value());
}

TEST(ThreadSafeQueueTest, FifoOrdering) {
    ThreadSafeQueue<int> q;
    q.push(1);
    q.push(2);
    q.push(3);

    EXPECT_EQ(q.pop(0ms).value(), 1);
    EXPECT_EQ(q.pop(0ms).value(), 2);
    EXPECT_EQ(q.pop(0ms).value(), 3);
}

TEST(ThreadSafeQueueTest, DrainAllReturnsAllItems) {
    ThreadSafeQueue<int> q;
    q.push(10);
    q.push(20);
    q.push(30);

    auto drained = q.drain_all();
    EXPECT_EQ(drained.size(), 3);
    EXPECT_EQ(drained[0], 10);
    EXPECT_EQ(drained[1], 20);
    EXPECT_EQ(drained[2], 30);

    EXPECT_TRUE(q.empty());
}

TEST(ThreadSafeQueueTest, DrainAllOnEmptyReturnsEmpty) {
    ThreadSafeQueue<int> q;
    auto drained = q.drain_all();
    EXPECT_TRUE(drained.empty());
}

TEST(ThreadSafeQueueTest, ClearRemovesAllItems) {
    ThreadSafeQueue<int> q;
    q.push(1);
    q.push(2);
    q.push(3);

    q.clear();
    EXPECT_TRUE(q.empty());
    EXPECT_EQ(q.size(), 0);

    auto result = q.pop(0ms);
    EXPECT_FALSE(result.has_value());
}

TEST(ThreadSafeQueueTest, SizeReflectsItemCount) {
    ThreadSafeQueue<int> q;
    EXPECT_EQ(q.size(), 0);

    q.push(42);
    EXPECT_EQ(q.size(), 1);

    q.push(99);
    EXPECT_EQ(q.size(), 2);

    q.pop(0ms);
    EXPECT_EQ(q.size(), 1);
}

TEST(ThreadSafeQueueTest, ConcurrentProducerConsumer) {
    ThreadSafeQueue<int> q;
    std::atomic<int> sum{0};
    std::atomic<bool> done{false};

    std::jthread consumer([&] {
        while (!done.load()) {
            auto item = q.pop(10ms);
            if (item.has_value()) {
                sum.fetch_add(item.value());
            }
        }
    });

    for (int i = 1; i <= 100; ++i) {
        q.push(i);
    }

    std::this_thread::sleep_for(100ms);
    done.store(true);
    consumer.join();

    EXPECT_EQ(sum.load(), 5050);
}

TEST(ThreadSafeQueueTest, DrainAllUnderContentionPreservesEveryItem) {
    ThreadSafeQueue<int> q;

    std::jthread producer([&] {
        for (int i = 1; i <= 50; ++i) {
            q.push(i);
            std::this_thread::sleep_for(1ms);
        }
    });

    // Drain concurrently with production, then drain the remainder after the
    // producer finishes. Full accounting: everything drained, across both
    // drains, must be exactly {1..50} with no loss and no duplication — "at
    // least one item came back" would pass a queue that drops or repeats
    // items under contention.
    std::vector<int> seen;
    std::this_thread::sleep_for(25ms);
    for (const int v : q.drain_all()) {
        seen.push_back(v);
    }
    producer.join();
    for (const int v : q.drain_all()) {
        seen.push_back(v);
    }

    ASSERT_EQ(seen.size(), 50u);
    std::sort(seen.begin(), seen.end());
    for (int i = 1; i <= 50; ++i) {
        EXPECT_EQ(seen[static_cast<size_t>(i) - 1], i);
    }
    EXPECT_TRUE(q.empty());
}
