#include "safety/signal_handler.h"
#include "core/print.h"
#include <csignal>

namespace {

std::atomic<SignalHandler*> g_active{nullptr};

extern "C" void raw_signal_handler(int signal) {
    auto* handler = g_active.load(std::memory_order_acquire);
    if (handler != nullptr) {
        handler->signal_shutdown();
    }
    (void)signal;
}

}

SignalHandler::~SignalHandler() {
    SignalHandler* expected = this;
    g_active.compare_exchange_strong(expected, nullptr, std::memory_order_acq_rel);
}

auto SignalHandler::install() -> void {
    g_active.store(this, std::memory_order_release);

    struct sigaction sa{};
    sa.sa_handler = raw_signal_handler;
    sa.sa_flags = SA_RESTART;
    sigemptyset(&sa.sa_mask);

    sigaction(SIGINT, &sa, nullptr);
    sigaction(SIGTERM, &sa, nullptr);

    signal(SIGPIPE, SIG_IGN);

    println("[SIGNAL] Handlers installed for SIGINT/SIGTERM");
}

auto SignalHandler::set_shutdown_callback(ShutdownCallback cb) -> void {
    callback_ = std::move(cb);
}

auto SignalHandler::is_shutdown_requested() const -> bool {
    return shutdown_requested_.load(std::memory_order_acquire);
}

auto SignalHandler::request_shutdown() -> void {
    shutdown_requested_.store(true, std::memory_order_release);
    if (callback_) {
        callback_();
    }
}

auto SignalHandler::reset() -> void {
    shutdown_requested_.store(false, std::memory_order_release);
}

auto SignalHandler::signal_shutdown() -> void {
    shutdown_requested_.store(true, std::memory_order_release);
}
