// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// Shared spdlog logger handle for the SDK C++ side.
//
// The same logger name — "xense.taccap" — is registered by the Python
// binding (`python/bindings/log.cpp`) at module import time, so C++ and
// Python emissions land in the same sink with the same format. If the
// SDK is used standalone (no Python), a stderr color sink is lazily
// created the first time `logger()` is called.

#pragma once

#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include <memory>
#include <mutex>

namespace xense::taccap {

inline std::shared_ptr<spdlog::logger> logger() {
    static std::shared_ptr<spdlog::logger> cached;
    static std::once_flag flag;
    std::call_once(flag, [] {
        cached = spdlog::get("xense.taccap");
        if (!cached) {
            // Standalone-C++ path: create + register the named logger so
            // any later module (Python binding, downstream linker) that
            // tries `spdlog::get("xense.taccap")` finds the same instance.
            auto sink = std::make_shared<spdlog::sinks::stderr_color_sink_mt>();
            cached = std::make_shared<spdlog::logger>("xense.taccap", sink);
            cached->set_level(spdlog::level::info);
            cached->set_pattern("[%H:%M:%S.%e] [%^%l%$] [%n] %v");
            try { spdlog::register_logger(cached); }
            catch (const spdlog::spdlog_ex&) {
                // Lost the race with another thread / Python init —
                // pick up whatever ended up registered.
                cached = spdlog::get("xense.taccap");
            }
        }
    });
    return cached;
}

}  // namespace xense::taccap
