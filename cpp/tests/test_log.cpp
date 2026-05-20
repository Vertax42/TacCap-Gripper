// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// Unit tests for the logging helpers in taccap/log.hpp. We deliberately
// avoid calling logger() here — the singleton's call_once runs at most
// once per process and tests that mutate it would not be hermetic. The
// pure helpers (default_log_dir / prune_old_logs / session_timestamp)
// cover the policy bits worth pinning down (env override, count-based
// rotation, timestamp shape).

#include <gtest/gtest.h>
#include <taccap/log.hpp>

#include <unistd.h>  // getpid

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <regex>
#include <set>
#include <string>
#include <vector>

namespace fs = std::filesystem;
namespace tx = xense::taccap;

namespace {

// Scoped tmpdir helper — auto-removes on destruction. Uses pid + a
// counter so parallel test invocations don't collide.
class TmpDir {
public:
    TmpDir() {
        static std::atomic<int> counter{0};
        auto base = fs::temp_directory_path();
        path_ = base / ("taccap_log_test_" + std::to_string(getpid()) + "_" +
                        std::to_string(++counter));
        fs::create_directories(path_);
    }
    ~TmpDir() {
        std::error_code ec;
        fs::remove_all(path_, ec);
    }
    const fs::path& path() const { return path_; }
private:
    fs::path path_;
};

// Touch a .log file with a specific mtime (seconds offset from now).
void touch_log_with_age(const fs::path& dir, const std::string& name,
                       int seconds_old) {
    auto path = dir / name;
    { std::ofstream(path) << "stub"; }
    // Set mtime to N seconds in the past so prune_old_logs has a
    // deterministic ordering. fs::last_write_time accepts a file_time
    // value; we anchor on "now" then subtract.
    auto when = fs::file_time_type::clock::now() -
                std::chrono::seconds(seconds_old);
    fs::last_write_time(path, when);
}

}  // namespace

TEST(LogHelpers, DefaultLogDirEnvOverride) {
    setenv("TACCAP_LOG_DIR", "/tmp/xx_taccap_env_override_demo", /*overwrite=*/1);
    EXPECT_EQ(tx::detail::default_log_dir(),
              fs::path("/tmp/xx_taccap_env_override_demo"));
    unsetenv("TACCAP_LOG_DIR");
}

TEST(LogHelpers, DefaultLogDirEmptyEnvFallsBackToHome) {
    // Empty string in env var should be treated as "not set" — fall
    // through to $HOME/.taccaplogs.
    setenv("TACCAP_LOG_DIR", "", /*overwrite=*/1);
    const char* home = std::getenv("HOME");
    ASSERT_NE(home, nullptr);
    EXPECT_EQ(tx::detail::default_log_dir(),
              fs::path(home) / ".taccaplogs");
    unsetenv("TACCAP_LOG_DIR");
}

TEST(LogHelpers, DefaultLogDirHomeFallback) {
    unsetenv("TACCAP_LOG_DIR");
    const char* home = std::getenv("HOME");
    ASSERT_NE(home, nullptr);
    EXPECT_EQ(tx::detail::default_log_dir(),
              fs::path(home) / ".taccaplogs");
}

TEST(LogHelpers, SessionTimestampFormat) {
    auto stamp = tx::detail::session_timestamp();
    // "YYYYMMDD_HHMMSS" — 15 chars.
    EXPECT_EQ(stamp.size(), 15u);
    EXPECT_TRUE(std::regex_match(
        stamp, std::regex(R"(\d{8}_\d{6})")))
        << "got: " << stamp;
}

TEST(LogHelpers, PruneNoOpWhenBelowKeep) {
    TmpDir tmp;
    for (int i = 0; i < 5; ++i) {
        touch_log_with_age(tmp.path(),
                          "session_" + std::to_string(i) + ".log",
                          /*seconds_old=*/i);
    }
    // 5 files, keep=10 → nothing to prune (size < keep).
    tx::detail::prune_old_logs(tmp.path(), 10);

    int n = 0;
    for (auto& e : fs::directory_iterator(tmp.path())) {
        if (e.path().extension() == ".log") ++n;
    }
    EXPECT_EQ(n, 5);
}

TEST(LogHelpers, PruneTrimsOldestKeepingNewestKMinusOne) {
    TmpDir tmp;
    // 12 files, ages 0..11 seconds (older = larger age).
    for (int i = 0; i < 12; ++i) {
        touch_log_with_age(tmp.path(),
                          "session_" + std::to_string(i) + ".log",
                          /*seconds_old=*/i);
    }
    tx::detail::prune_old_logs(tmp.path(), 10);

    // Spec: leave room for one more file → at most keep-1 = 9 remain.
    std::set<std::string> survivors;
    for (auto& e : fs::directory_iterator(tmp.path())) {
        if (e.path().extension() == ".log") {
            survivors.insert(e.path().filename().string());
        }
    }
    EXPECT_EQ(survivors.size(), 9u);
    // Newest 9 are i=0..8 (smallest age). Oldest 3 (i=9,10,11) deleted.
    for (int i = 0; i <= 8; ++i) {
        EXPECT_TRUE(survivors.count("session_" + std::to_string(i) + ".log"))
            << "expected newest (age=" << i << ") survived";
    }
    for (int i = 9; i <= 11; ++i) {
        EXPECT_FALSE(survivors.count("session_" + std::to_string(i) + ".log"))
            << "expected oldest (age=" << i << ") pruned";
    }
}

TEST(LogHelpers, PruneSkipsNonLogFiles) {
    TmpDir tmp;
    // 12 .log files + 5 non-.log files. The non-logs must survive
    // regardless of keep policy.
    for (int i = 0; i < 12; ++i) {
        touch_log_with_age(tmp.path(),
                          "session_" + std::to_string(i) + ".log", i);
    }
    for (int i = 0; i < 5; ++i) {
        auto p = tmp.path() / ("note_" + std::to_string(i) + ".txt");
        std::ofstream(p) << "not a log";
    }
    tx::detail::prune_old_logs(tmp.path(), 10);

    int logs = 0, others = 0;
    for (auto& e : fs::directory_iterator(tmp.path())) {
        if (e.path().extension() == ".log") ++logs;
        else ++others;
    }
    EXPECT_EQ(logs, 9);
    EXPECT_EQ(others, 5);
}

TEST(LogHelpers, PruneMissingDirIsNoop) {
    // Nonexistent dir → silently return, no throw.
    EXPECT_NO_THROW(
        tx::detail::prune_old_logs(
            fs::path("/tmp/__definitely_does_not_exist_taccap__"), 10));
}

TEST(LogHelpers, PatternConstantsMatchSpec) {
    // Hard-pin the format strings — if anyone "tidies" these into
    // shorter forms, downstream grep over historical logs breaks.
    EXPECT_STREQ(tx::SPDLOG_PATTERN,
                 "[%D %T.%e] [%n] [%^%l%$] %v");
    EXPECT_STREQ(tx::FILE_LOG_PATTERN,
                 "[%Y-%m-%d %H:%M:%S.%e] [%n] [%l] %v");
    EXPECT_EQ(tx::kMaxSessionLogs, 10u);
}
