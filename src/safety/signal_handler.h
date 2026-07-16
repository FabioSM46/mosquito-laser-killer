#pragma once

#include <atomic>
#include <functional>

class SignalHandler {
public:
    using ShutdownCallback = std::function<void()>;

    SignalHandler() = default;
    ~SignalHandler();

    SignalHandler(const SignalHandler&) = delete;
    auto operator=(const SignalHandler&) -> SignalHandler& = delete;
    SignalHandler(SignalHandler&&) = delete;
    auto operator=(SignalHandler&&) -> SignalHandler& = delete;

    auto install() -> void;

    auto set_shutdown_callback(ShutdownCallback cb) -> void;

    [[nodiscard]] auto is_shutdown_requested() const -> bool;

    auto request_shutdown() -> void;

    auto reset() -> void;

    auto signal_shutdown() -> void;

private:
    std::atomic<bool> shutdown_requested_{false};
    ShutdownCallback callback_;
};
