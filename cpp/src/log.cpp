// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// Single definition of the shared "xense.taccap" logger and its console-sink
// holder. Kept out of the header (non-inline, one TU) so every .so that links
// taccap_core shares ONE logger instance, and so the logger is built WITHOUT
// spdlog's global registry — see the rationale in taccap/log.hpp.

#include <taccap/log.hpp>

#include <memory>
#include <vector>

namespace xense::taccap {

namespace detail {

std::shared_ptr<spdlog::sinks::sink>& console_sink_holder() {
    static std::shared_ptr<spdlog::sinks::sink> p;
    return p;
}

}  // namespace detail

std::shared_ptr<spdlog::logger> logger() {
    // Magic-static init: thread-safe, runs once. Cached for the process; never
    // touches spdlog's global registry (so it can't collide with another
    // spdlog version loaded elsewhere in the process).
    static std::shared_ptr<spdlog::logger> lg = [] {
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
        return created;
    }();
    return lg;
}

}  // namespace xense::taccap
