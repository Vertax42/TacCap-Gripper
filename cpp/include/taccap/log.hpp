// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// Shared spdlog logger handle for the SDK C++ side.
//
// The same logger name — "xense.taccap" — is also fetched by the Python
// binding (`python/bindings/log.cpp`) at module import time, so C++ and
// Python emissions land in the same logger with the same sinks. Both
// sides go through `logger()` here — Python no longer constructs its own
// logger, so the convention below is enforced in one place.
//
// Sink layout (both attached by default):
//   - stderr (color), level user-controllable via set_console_level()
//   - file (per-session), level always DEBUG, pattern FILE_LOG_PATTERN
//
// The logger itself sits at DEBUG so the file sink sees everything; the
// console sink applies its own level filter on top.

#pragma once

#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <system_error>
#include <vector>

namespace xense::taccap {

inline constexpr const char* SPDLOG_PATTERN =
    "[%D %T.%e] [%n] [%^%l%$] %v";
inline constexpr const char* FILE_LOG_PATTERN =
    "[%Y-%m-%d %H:%M:%S.%e] [%n] [%l] %v";

// Cap on the number of historical session logs kept in the log dir.
// Newest N are retained; older ones are removed at logger() init time.
inline constexpr std::size_t kMaxSessionLogs = 10;

namespace detail {

// Resolve the directory where session logs are written.
//   - $TACCAP_LOG_DIR if set (and non-empty)
//   - $HOME/.taccaplogs otherwise
//   - cwd/.taccaplogs as a last-resort sentinel (HOME missing is exotic
//     enough that we don't try harder)
inline std::filesystem::path default_log_dir() {
    if (const char* env = std::getenv("TACCAP_LOG_DIR"); env && *env) {
        return std::filesystem::path(env);
    }
    if (const char* home = std::getenv("HOME"); home && *home) {
        return std::filesystem::path(home) / ".taccaplogs";
    }
    return std::filesystem::path(".taccaplogs");
}

inline std::string session_timestamp() {
    auto now = std::chrono::system_clock::now();
    auto tt = std::chrono::system_clock::to_time_t(now);
    std::tm tm{};
    localtime_r(&tt, &tm);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y%m%d_%H%M%S");
    return oss.str();
}

// Delete the oldest *.log files in `dir` until at most `keep - 1` remain,
// leaving room for the new session file the caller is about to create.
// No-throw: any filesystem error just stops pruning silently.
inline void prune_old_logs(const std::filesystem::path& dir,
                           std::size_t keep) {
    namespace fs = std::filesystem;
    std::error_code ec;
    if (!fs::is_directory(dir, ec)) return;

    std::vector<fs::directory_entry> entries;
    for (const auto& e : fs::directory_iterator(dir, ec)) {
        std::error_code reg_ec;
        if (e.is_regular_file(reg_ec) && e.path().extension() == ".log") {
            entries.push_back(e);
        }
    }
    if (entries.size() < keep) return;

    std::sort(entries.begin(), entries.end(),
              [](const fs::directory_entry& a,
                 const fs::directory_entry& b) {
                  std::error_code ec_a, ec_b;
                  return fs::last_write_time(a, ec_a) <
                         fs::last_write_time(b, ec_b);
              });

    while (entries.size() >= keep) {
        std::error_code rm_ec;
        fs::remove(entries.front().path(), rm_ec);
        entries.erase(entries.begin());
    }
}

// Storage for the console sink so external set_console_*() helpers can
// reach it after logger() finished initializing.
inline std::shared_ptr<spdlog::sinks::sink>& console_sink_holder() {
    static std::shared_ptr<spdlog::sinks::sink> p;
    return p;
}

inline std::shared_ptr<spdlog::sinks::sink> try_make_file_sink() {
    namespace fs = std::filesystem;
    try {
        auto dir = default_log_dir();
        std::error_code ec;
        fs::create_directories(dir, ec);
        if (ec) return nullptr;
        prune_old_logs(dir, kMaxSessionLogs);
        auto path = dir / ("session_" + session_timestamp() + ".log");
        auto sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(
            path.string(), /*truncate=*/false);
        sink->set_level(spdlog::level::debug);
        sink->set_pattern(FILE_LOG_PATTERN);
        return sink;
    } catch (const std::exception&) {
        // Disk full, permission denied, exotic FS — file sink is best-
        // effort; console keeps working.
        return nullptr;
    }
}

}  // namespace detail

// Initialise the named spdlog logger if it isn't registered yet, then
// return it. Looking up by name on every call (instead of caching in a
// function-local static) is intentional: with -fvisibility-inlines-hidden
// each .so that includes this header gets its OWN copy of the inline-
// function static, and writes from the call_once lambda in one .so are
// invisible to the outer function in the same .so (the symbol resolves
// to a different storage location once the lambda is emitted as a
// separately-linked function). The spdlog registry already holds the
// canonical instance, so we lean on it instead of duplicating state.
inline std::shared_ptr<spdlog::logger> logger() {
    static std::once_flag flag;
    std::call_once(flag, [] {
        if (spdlog::get("xense.taccap")) return;

        auto console_sink =
            std::make_shared<spdlog::sinks::stderr_color_sink_mt>();
        console_sink->set_pattern(SPDLOG_PATTERN);
        console_sink->set_level(spdlog::level::info);
        detail::console_sink_holder() = console_sink;

        std::vector<spdlog::sink_ptr> sinks{console_sink};
        if (auto file_sink = detail::try_make_file_sink()) {
            sinks.push_back(file_sink);
        }
        auto created = std::make_shared<spdlog::logger>(
            "xense.taccap", sinks.begin(), sinks.end());
        created->set_level(spdlog::level::debug);
        created->flush_on(spdlog::level::warn);
        try {
            spdlog::register_logger(created);
        } catch (const spdlog::spdlog_ex&) {
            // Lost the race with another TU; the registry has whoever
            // got there first — that's fine, our `created` will simply
            // be dropped when this lambda returns.
        }
    });
    return spdlog::get("xense.taccap");
}

// Console-sink controls. The console sink takes the user-facing level
// filter; the file sink is intentionally not exposed (stays at DEBUG +
// FILE_LOG_PATTERN so historical greps stay parseable).
inline void set_console_level(spdlog::level::level_enum lvl) {
    (void)logger();
    if (auto& sink = detail::console_sink_holder(); sink) {
        sink->set_level(lvl);
    }
}

inline spdlog::level::level_enum get_console_level() {
    (void)logger();
    if (auto& sink = detail::console_sink_holder(); sink) {
        return sink->level();
    }
    return spdlog::level::off;
}

inline void set_console_pattern(const std::string& pattern) {
    (void)logger();
    if (auto& sink = detail::console_sink_holder(); sink) {
        sink->set_pattern(pattern);
    }
}

}  // namespace xense::taccap
