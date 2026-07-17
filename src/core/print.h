#pragma once

#include <format>
#include <cstdio>
#include <cstdint>
#include <mutex>
#include <string>
#include <string_view>
#include <atomic>
#include <cerrno>
#include <unistd.h>
#include <fcntl.h>

// Non-blocking, best-effort logging.
//
// The control thread logs while the laser pin is HIGH, and every path that can
// end a pulse — enforce_max_pulse, execute_cycle's duration check, Laser::fire's
// re-entry check, the watchdog, the E-stop — runs on that same thread. There is
// no hardware one-shot behind the GPIO. So if a write to a stalled consumer
// blocks (stdout piped to a `tee` that is paged out, a full pipe buffer, a slow
// serial console), the thread stops, and nothing turns the laser off: SIGPIPE is
// ignored and the signal handlers use SA_RESTART, so no signal breaks it out
// either.
//
// A dropped log line is always preferable to a wedged control thread, so
// log_init() marks the descriptors O_NONBLOCK and writes that would block are
// dropped and counted instead. std::println is deliberately not used: it offers
// no way to decline to block.
namespace mlk_log {

inline std::atomic<uint64_t> g_dropped_lines{0};
inline std::mutex g_mutex;

// Original flags, so the descriptors can be restored on exit. O_NONBLOCK is a
// property of the shared *open file description*, not of the fd, so when stdout
// is inherited from an interactive shell this flag is visible to the shell too —
// and a shell that starts getting EAGAIN on its own stdout misbehaves. Restoring
// on exit keeps the blast radius inside this process's lifetime.
inline int g_stdout_flags{-1};
inline int g_stderr_flags{-1};

// A previous line on that descriptor was truncated mid-write, so the next
// output ON THE SAME DESCRIPTOR must start with a newline to terminate the
// orphaned fragment rather than splice onto it. Tracked per descriptor: a
// fragment on stdout must not be "resynced" by a write to stderr.
inline bool g_needs_resync_stdout{false};
inline bool g_needs_resync_stderr{false};

// Call once from main before any worker thread starts. Without it, logging still
// works — it just blocks, which is fine for tests and tools but not for the
// process that owns the laser.
inline auto log_init() -> void {
    // Read BOTH descriptors' original flags BEFORE touching either. O_NONBLOCK
    // is a property of the shared open file description, and stdout/stderr
    // usually share one (same terminal, same log file, same pipe): reading
    // stderr's flags after setting stdout's would store the already-polluted
    // value, and log_shutdown would then "restore" O_NONBLOCK back ON.
    g_stdout_flags = ::fcntl(STDOUT_FILENO, F_GETFL, 0);
    g_stderr_flags = ::fcntl(STDERR_FILENO, F_GETFL, 0);

    bool failed = false;
    if (g_stdout_flags < 0 ||
        ::fcntl(STDOUT_FILENO, F_SETFL, g_stdout_flags | O_NONBLOCK) < 0) {
        failed = true;
    }
    if (g_stderr_flags < 0 ||
        ::fcntl(STDERR_FILENO, F_SETFL, g_stderr_flags | O_NONBLOCK) < 0) {
        failed = true;
    }

    // A failure here is not fatal, but it means logging can still block, which is
    // the condition this exists to prevent. Say so rather than fail silently.
    if (failed) {
        const char* msg = "[LOG] WARNING: could not set O_NONBLOCK on stdio; "
                          "logging may block the control thread\n";
        // Bound into a variable rather than (void)-cast: glibc marks write() with
        // warn_unused_result, which a cast does not silence under _FORTIFY_SOURCE.
        const auto ignored = ::write(STDERR_FILENO, msg, __builtin_strlen(msg));
        static_cast<void>(ignored);
    }
}

// Call once from main on the way out. Idempotent: after the first call the
// stored flags are reset to "unknown", so repeat calls are no-ops.
inline auto log_shutdown() -> void {
    const auto dropped = g_dropped_lines.exchange(0, std::memory_order_relaxed);
    if (dropped > 0) {
        // Best-effort, and worth knowing: dropped lines mean the log is an
        // incomplete record of a run that involved a Class 4 laser.
        auto msg = std::format("[LOG] {} log line(s) dropped because the output "
                               "consumer could not keep up\n", dropped);
        const auto ignored = ::write(STDERR_FILENO, msg.data(), msg.size());
        static_cast<void>(ignored);
    }

    if (g_stdout_flags >= 0) {
        (void)::fcntl(STDOUT_FILENO, F_SETFL, g_stdout_flags);
        g_stdout_flags = -1;
    }
    if (g_stderr_flags >= 0) {
        (void)::fcntl(STDERR_FILENO, F_SETFL, g_stderr_flags);
        g_stderr_flags = -1;
    }
}

[[nodiscard]] inline auto dropped_lines() -> uint64_t {
    return g_dropped_lines.load(std::memory_order_relaxed);
}

inline auto write_all(int fd, std::string_view s) -> void {
    // The lock only serialises writers so lines do not interleave. It cannot
    // block for long: the write() below never blocks once log_init() has run.
    std::lock_guard lock(g_mutex);

    bool& needs_resync =
        (fd == STDERR_FILENO) ? g_needs_resync_stderr : g_needs_resync_stdout;

    if (needs_resync) {
        // Terminate the previous truncated line. If this too fails to go out,
        // nothing is lost that was not already lost.
        const char nl = '\n';
        if (::write(fd, &nl, 1) == 1) {
            needs_resync = false;
        }
    }

    size_t offset = 0;
    while (offset < s.size()) {
        const auto n = ::write(fd, s.data() + offset, s.size() - offset);
        if (n > 0) {
            offset += static_cast<size_t>(n);
            continue;
        }
        if (n < 0 && errno == EINTR) {
            continue;
        }
        // EAGAIN means the consumer is not keeping up; anything else is a real
        // I/O error. Either way, drop the rest rather than wait — a truncated log
        // line is survivable, a stalled control thread holding a live pulse is not.
        g_dropped_lines.fetch_add(1, std::memory_order_relaxed);
        // A partial write left a fragment with no newline; flag it so the next
        // line does not splice onto it and read as a single corrupt entry. Only
        // ever SET here: clearing a still-pending resync (a fragment landed, the
        // resync newline itself failed, then this line failed at offset 0) would
        // splice the next successful line onto the older fragment.
        if (offset > 0) {
            needs_resync = true;
        }
        return;
    }
}

}

template <typename... Args>
void println(std::FILE* stream, std::format_string<Args...> fmt, Args&&... args) {
    auto s = std::format(fmt, std::forward<Args>(args)...);
    s.push_back('\n');
    mlk_log::write_all(::fileno(stream), s);
}

template <typename... Args>
void println(std::format_string<Args...> fmt, Args&&... args) {
    auto s = std::format(fmt, std::forward<Args>(args)...);
    s.push_back('\n');
    mlk_log::write_all(STDOUT_FILENO, s);
}
