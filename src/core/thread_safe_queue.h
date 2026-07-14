#pragma once

#include <queue>
#include <mutex>
#include <condition_variable>
#include <vector>
#include <chrono>
#include <optional>

template <typename T>
class ThreadSafeQueue {
public:
    ThreadSafeQueue() = default;
    ~ThreadSafeQueue() = default;

    ThreadSafeQueue(const ThreadSafeQueue&) = delete;
    auto operator=(const ThreadSafeQueue&) -> ThreadSafeQueue& = delete;
    ThreadSafeQueue(ThreadSafeQueue&&) = delete;
    auto operator=(ThreadSafeQueue&&) -> ThreadSafeQueue& = delete;

    void push(T item) {
        {
            std::lock_guard lock(mutex_);
            queue_.push(std::move(item));
        }
        cv_.notify_one();
    }

    auto pop(std::chrono::milliseconds timeout) -> std::optional<T> {
        std::unique_lock lock(mutex_);
        if (!cv_.wait_for(lock, timeout, [this] { return !queue_.empty(); })) {
            return std::nullopt;
        }
        T item = std::move(queue_.front());
        queue_.pop();
        return item;
    }

    [[nodiscard]] auto drain_all() -> std::vector<T> {
        std::lock_guard lock(mutex_);
        std::vector<T> drained;
        while (!queue_.empty()) {
            drained.push_back(std::move(queue_.front()));
            queue_.pop();
        }
        return drained;
    }

    [[nodiscard]] auto empty() const -> bool {
        std::lock_guard lock(mutex_);
        return queue_.empty();
    }

    [[nodiscard]] auto size() const -> size_t {
        std::lock_guard lock(mutex_);
        return queue_.size();
    }

    void clear() {
        std::lock_guard lock(mutex_);
        std::queue<T> empty;
        std::swap(queue_, empty);
    }

private:
    std::queue<T> queue_;
    mutable std::mutex mutex_;
    std::condition_variable cv_;
};
