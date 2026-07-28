#include <gtest/gtest.h>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <string>
#include <unistd.h>

#include "core/print.h"

using namespace testing;

namespace {

constexpr size_t k_pipe_page = 4096;

// A pipe pair with BOTH ends non-blocking (a blocking read end would hang
// drain() on an empty pipe), closed on scope exit.
struct Pipe {
    int fds[2]{-1, -1};

    Pipe() {
        EXPECT_EQ(::pipe(fds), 0);
        for (const int fd : fds) {
            if (fd < 0) {
                continue;
            }
            const int flags = ::fcntl(fd, F_GETFL, 0);
            EXPECT_GE(flags, 0);
            EXPECT_EQ(::fcntl(fd, F_SETFL, flags | O_NONBLOCK), 0);
        }
    }
    ~Pipe() {
        if (fds[0] >= 0) {
            ::close(fds[0]);
        }
        if (fds[1] >= 0) {
            ::close(fds[1]);
        }
    }

    // Write until the kernel buffer refuses more.
    void fill() {
        std::string chunk(4096, 'x');
        while (::write(fds[1], chunk.data(), chunk.size()) > 0) {
        }
        EXPECT_EQ(errno, EAGAIN);
    }

    // Read everything currently buffered. The read end is non-blocking, so an
    // empty pipe ends the loop with EAGAIN instead of hanging.
    auto drain() -> std::string {
        std::string out;
        char buf[4096];
        ssize_t n;
        while ((n = ::read(fds[0], buf, sizeof(buf))) > 0) {
            out.append(buf, static_cast<size_t>(n));
        }
        return out;
    }

    // Free one whole ring slot plus a sliver. A Linux pipe buffer is a ring of
    // page-sized buffers: reading a few bytes frees space at the READ head
    // only, which a writer never sees — a full ring slot must be consumed
    // before any further write can land.
    void free_one_slot() {
        std::string discard(k_pipe_page + 8, '\0');
        ASSERT_EQ(::read(fds[0], discard.data(), discard.size()),
                  static_cast<ssize_t>(discard.size()));
    }
};

// log_shutdown() restores the process's stdio flags even if the test body
// fails partway through log_init().
struct LogSession {
    LogSession() { mlk_log::log_init(); }
    ~LogSession() { mlk_log::log_shutdown(); }
};

class PrintTest : public Test {
protected:
    void TearDown() override {
        // Tests that exercise the truncation path leave the resync flags set;
        // clear them so no later line (in this or another suite) inherits a
        // stray leading newline.
        mlk_log::g_needs_resync_stdout = false;
        mlk_log::g_needs_resync_stderr = false;
    }
};

TEST_F(PrintTest, FullPipeDropsTheLineAndCountsIt) {
    Pipe p;
    p.fill();

    const auto before = mlk_log::dropped_lines();
    mlk_log::write_all(p.fds[1], "this line cannot fit\n");
    EXPECT_EQ(mlk_log::dropped_lines(), before + 1);
}

TEST_F(PrintTest, PartialWriteDropsTheRemainderAndCountsIt) {
    Pipe p;
    p.fill();
    p.free_one_slot();

    // Write a payload LARGER than PIPE_BUF (4096): smaller writes are atomic
    // on pipes and would just EAGAIN with nothing written. The kernel lands
    // the freed page and reports a short count.
    const std::string big(5000, 'A');
    const auto before = mlk_log::dropped_lines();
    mlk_log::write_all(p.fds[1], big);
    EXPECT_EQ(mlk_log::dropped_lines(), before + 1);
    // A fragment landed with no newline: the resync flag must be set.
    EXPECT_TRUE(mlk_log::g_needs_resync_stdout);
}

TEST_F(PrintTest, TruncatedLineIsTerminatedBeforeTheNextLine) {
    Pipe p;
    p.fill();
    p.free_one_slot();

    // Partial write (> PIPE_BUF so the kernel lands the freed page): the
    // fragment has no trailing newline.
    mlk_log::write_all(p.fds[1], std::string(5000, 'A'));

    // Drain everything and write the next line. It must begin with '\n' so it
    // does not splice onto the orphaned fragment and read as one corrupt entry.
    p.drain();
    mlk_log::write_all(p.fds[1], "NEXT\n");
    const auto out = p.drain();

    EXPECT_FALSE(out.empty());
    EXPECT_EQ(out.front(), '\n');
    EXPECT_EQ(out, "\nNEXT\n");
    // The resync consumed the flag.
    EXPECT_FALSE(mlk_log::g_needs_resync_stdout);
}

TEST_F(PrintTest, SuccessfulWriteDeliversTheWholeLine) {
    Pipe p;
    mlk_log::write_all(p.fds[1], "hello\n");
    EXPECT_EQ(p.drain(), "hello\n");
    EXPECT_FALSE(mlk_log::g_needs_resync_stdout);
}

TEST_F(PrintTest, ResyncFlagsAreTrackedPerDescriptor) {
    // A fragment stranded on "stdout" must not be resynced by a write to
    // stderr: the terminating newline belongs on the descriptor holding the
    // fragment. write_all keys the flag on fd == STDERR_FILENO, so the stderr
    // side has to go through the real fd 2, redirected into a pipe.
    Pipe out_pipe;
    out_pipe.fill();
    out_pipe.free_one_slot();
    mlk_log::write_all(out_pipe.fds[1], std::string(5000, 'A'));
    ASSERT_TRUE(mlk_log::g_needs_resync_stdout);
    ASSERT_FALSE(mlk_log::g_needs_resync_stderr);

    const int stderr_backup = ::dup(STDERR_FILENO);
    ASSERT_GE(stderr_backup, 0);
    Pipe err_pipe;
    ASSERT_EQ(::dup2(err_pipe.fds[1], STDERR_FILENO), STDERR_FILENO);
    mlk_log::write_all(STDERR_FILENO, "ERR\n");
    EXPECT_EQ(::dup2(stderr_backup, STDERR_FILENO), STDERR_FILENO);
    ::close(stderr_backup);

    // The stderr line went out clean — no stray leading newline — and it did
    // not consume stdout's pending resync. A single merged flag fails both.
    EXPECT_EQ(err_pipe.drain(), "ERR\n");
    EXPECT_TRUE(mlk_log::g_needs_resync_stdout)
        << "a write to stderr consumed stdout's pending resync";
}

TEST_F(PrintTest, LogShutdownReportsTheDroppedLineCount) {
    // The shutdown-time report is what tells an incident reader the log is an
    // incomplete record (§4.11); it was never asserted anywhere.
    mlk_log::g_dropped_lines.exchange(0, std::memory_order_relaxed);

    Pipe p;
    p.fill();
    mlk_log::write_all(p.fds[1], "lost\n");
    ASSERT_EQ(mlk_log::dropped_lines(), 1u);

    const int stderr_backup = ::dup(STDERR_FILENO);
    ASSERT_GE(stderr_backup, 0);
    Pipe err_pipe;
    ASSERT_EQ(::dup2(err_pipe.fds[1], STDERR_FILENO), STDERR_FILENO);
    mlk_log::log_shutdown();
    EXPECT_EQ(::dup2(stderr_backup, STDERR_FILENO), STDERR_FILENO);
    ::close(stderr_backup);

    const auto report = err_pipe.drain();
    EXPECT_NE(report.find("1 log line(s) dropped"), std::string::npos)
        << "report was: " << report;
    // The exchange consumed the count, so repeat shutdowns stay silent.
    EXPECT_EQ(mlk_log::dropped_lines(), 0u);
}

TEST_F(PrintTest, SharedStdoutStderrDescriptionSurvivesInitAndShutdown) {
    // stdout and stderr usually share ONE open file description (a terminal, a
    // shell pipe), and O_NONBLOCK lives on the description, not the fd.
    // log_init must read BOTH flag sets before setting either: reading
    // stderr's after polluting the shared description via stdout would save
    // O_NONBLOCK as "original", and shutdown would then restore the pollution
    // instead of removing it. Force the sharing explicitly — the test runner
    // may hand us separate descriptions, which would mask the bug.
    mlk_log::g_dropped_lines.exchange(0, std::memory_order_relaxed);

    const int stdout_backup = ::dup(STDOUT_FILENO);
    const int stderr_backup = ::dup(STDERR_FILENO);
    ASSERT_GE(stdout_backup, 0);
    ASSERT_GE(stderr_backup, 0);

    int raw[2];
    ASSERT_EQ(::pipe(raw), 0);              // deliberately blocking: no O_NONBLOCK
    ASSERT_EQ(::dup2(raw[1], STDOUT_FILENO), STDOUT_FILENO);
    ASSERT_EQ(::dup2(raw[1], STDERR_FILENO), STDERR_FILENO);
    ::close(raw[1]);

    const int before = ::fcntl(STDOUT_FILENO, F_GETFL, 0);
    const bool before_ok = before >= 0 && (before & O_NONBLOCK) == 0;

    mlk_log::log_init();
    const int during_out = ::fcntl(STDOUT_FILENO, F_GETFL, 0);
    const int during_err = ::fcntl(STDERR_FILENO, F_GETFL, 0);
    mlk_log::log_shutdown();

    const int after_out = ::fcntl(STDOUT_FILENO, F_GETFL, 0);
    const int after_err = ::fcntl(STDERR_FILENO, F_GETFL, 0);

    // Restore the real stdio before asserting so gtest's own output works.
    ASSERT_EQ(::dup2(stdout_backup, STDOUT_FILENO), STDOUT_FILENO);
    ASSERT_EQ(::dup2(stderr_backup, STDERR_FILENO), STDERR_FILENO);
    ::close(stdout_backup);
    ::close(stderr_backup);
    ::close(raw[0]);

    ASSERT_TRUE(before_ok) << "test setup: expected a blocking pipe";
    EXPECT_TRUE(during_out & O_NONBLOCK);
    EXPECT_TRUE(during_err & O_NONBLOCK);
    EXPECT_EQ(after_out & O_NONBLOCK, 0)
        << "shutdown restored O_NONBLOCK onto the shared description";
    EXPECT_EQ(after_err & O_NONBLOCK, 0);
}

TEST_F(PrintTest, LogInitSetsNonBlockingAndShutdownRestoresFlags) {
    const int stdout_before = ::fcntl(STDOUT_FILENO, F_GETFL, 0);
    const int stderr_before = ::fcntl(STDERR_FILENO, F_GETFL, 0);
    ASSERT_GE(stdout_before, 0);
    ASSERT_GE(stderr_before, 0);

    {
        LogSession session;
        const int stdout_during = ::fcntl(STDOUT_FILENO, F_GETFL, 0);
        const int stderr_during = ::fcntl(STDERR_FILENO, F_GETFL, 0);
        ASSERT_GE(stdout_during, 0);
        ASSERT_GE(stderr_during, 0);
        EXPECT_TRUE(stdout_during & O_NONBLOCK);
        EXPECT_TRUE(stderr_during & O_NONBLOCK);
    }

    // O_NONBLOCK is a property of the shared open file description, so leaving
    // it set would hand EAGAIN back to whatever owns our stdout (a shell, the
    // test runner). The original mask must be restored exactly.
    EXPECT_EQ(::fcntl(STDOUT_FILENO, F_GETFL, 0), stdout_before);
    EXPECT_EQ(::fcntl(STDERR_FILENO, F_GETFL, 0), stderr_before);
}

}
