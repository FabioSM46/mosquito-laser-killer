#pragma once

#if __has_include(<print>)
    #include <print>
    using std::println;
    using std::print;
#else
    #include <format>
    #include <cstdio>
    #include <mutex>
    #include <string>

    inline namespace mlk_print_impl {
    inline std::mutex g_print_mutex;

    template <typename... Args>
    void println(std::FILE* stream, std::format_string<Args...> fmt, Args&&... args) {
        auto s = std::format(fmt, std::forward<Args>(args)...);
        std::lock_guard lock(g_print_mutex);
        std::fprintf(stream, "%s\n", s.c_str());
        std::fflush(stream);
    }

    template <typename... Args>
    void println(std::format_string<Args...> fmt, Args&&... args) {
        auto s = std::format(fmt, std::forward<Args>(args)...);
        std::lock_guard lock(g_print_mutex);
        std::printf("%s\n", s.c_str());
        std::fflush(stdout);
    }

    template <typename... Args>
    void print(std::format_string<Args...> fmt, Args&&... args) {
        auto s = std::format(fmt, std::forward<Args>(args)...);
        std::lock_guard lock(g_print_mutex);
        std::printf("%s", s.c_str());
        std::fflush(stdout);
    }
    } 
#endif
