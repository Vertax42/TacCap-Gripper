// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// pybind11 bindings for spdlog. Exposes a single process-level logger
// named "xense.taccap" so Python and C++ can share format/sinks/level.
//
// Python surface (under module attribute `log`):
//   log.set_level(name) "trace"|"debug"|"info"|"warn"|"error"|"critical"|"off"
//   log.get_level() -> str
//   log.set_pattern(pattern)
//   log.trace/debug/info/warn/error/critical(msg)
//   log.flush()

#include <pybind11/pybind11.h>

#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

namespace py = pybind11;

namespace xense::taccap::python {

namespace {

constexpr const char *kLoggerName = "xense.taccap";

std::shared_ptr<spdlog::logger> get_or_create_logger() {
  static std::once_flag flag;
  std::call_once(flag, [] {
    auto existing = spdlog::get(kLoggerName);
    if (existing) {
      return;
    }
    auto sink = std::make_shared<spdlog::sinks::stderr_color_sink_mt>();
    auto logger = std::make_shared<spdlog::logger>(kLoggerName, sink);
    logger->set_level(spdlog::level::info);
    logger->set_pattern("[%H:%M:%S.%e] [%^%l%$] [%n] %v");
    logger->flush_on(spdlog::level::warn);
    spdlog::register_logger(logger);
  });
  return spdlog::get(kLoggerName);
}

spdlog::level::level_enum parse_level(const std::string &s) {
  auto lvl = spdlog::level::from_str(s);
  // from_str() returns `off` both for the literal "off" AND for unknown
  // strings, so disambiguate explicitly.
  if (lvl == spdlog::level::off && s != "off") {
    throw std::invalid_argument(
        "unknown log level '" + s +
        "' (expected one of: trace, debug, info, warn, error, critical, off)");
  }
  return lvl;
}

} // namespace

void bind_log(py::module_ &m) {
  auto log = m.def_submodule(
      "log",
      "spdlog-backed logger shared with the C++ core (name='xense.taccap').");

  log.def(
      "set_level",
      [](const std::string &name) {
        get_or_create_logger()->set_level(parse_level(name));
      },
      py::arg("name"),
      "Set the minimum level. One of: trace, debug, info, warn, error, "
      "critical, off.");

  log.def(
      "get_level",
      []() -> std::string {
        auto lvl = get_or_create_logger()->level();
        auto sv = spdlog::level::to_string_view(lvl);
        return std::string(sv.data(), sv.size());
      },
      "Return the current level as a lowercase string.");

  log.def(
      "set_pattern",
      [](const std::string &pattern) {
        get_or_create_logger()->set_pattern(pattern);
      },
      py::arg("pattern"),
      "Set the spdlog format pattern (e.g. '[%H:%M:%S.%e] [%^%l%$] [%n] %v').");

  log.def(
      "flush", []() { get_or_create_logger()->flush(); },
      "Flush the underlying sink.");

  // Thin per-level wrappers. Python-side formatting is the caller's job
  // (use f-strings) so we don't need spdlog's fmt arg pack here.
  log.def(
      "trace",
      [](const std::string &msg) { get_or_create_logger()->trace(msg); },
      py::arg("msg"));
  log.def(
      "debug",
      [](const std::string &msg) { get_or_create_logger()->debug(msg); },
      py::arg("msg"));
  log.def(
      "info", [](const std::string &msg) { get_or_create_logger()->info(msg); },
      py::arg("msg"));
  log.def(
      "warn", [](const std::string &msg) { get_or_create_logger()->warn(msg); },
      py::arg("msg"));
  log.def(
      "warning", // alias to match stdlib `logging` ergonomics
      [](const std::string &msg) { get_or_create_logger()->warn(msg); },
      py::arg("msg"));
  log.def(
      "error",
      [](const std::string &msg) { get_or_create_logger()->error(msg); },
      py::arg("msg"));
  log.def(
      "critical",
      [](const std::string &msg) { get_or_create_logger()->critical(msg); },
      py::arg("msg"));

  // Eagerly create the logger at import time so the first call doesn't
  // pay the registration cost mid-stream.
  (void)get_or_create_logger();
}

} // namespace xense::taccap::python
