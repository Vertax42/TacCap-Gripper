// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// pybind11 bindings for spdlog. Defers all logger construction to
// xense::taccap::logger() in taccap/log.hpp so C++ and Python share the
// same sink layout (stderr color + per-session file in ~/.taccaplogs/).
//
// Python surface (under module attribute `log`):
//   log.set_level(name)   trace|debug|info|warn|error|critical|off
//   log.get_level() -> str
//   log.set_pattern(pattern)
//   log.trace/debug/info/warn/error/critical(msg)
//   log.flush()
//
// Note: set_level / set_pattern affect the **console sink only**. The
// per-session file sink stays at DEBUG with FILE_LOG_PATTERN so the
// log archive format remains grep-stable.

#include <pybind11/pybind11.h>

#include <taccap/log.hpp>

#include <spdlog/spdlog.h>

#include <stdexcept>
#include <string>

namespace py = pybind11;

namespace xense::taccap::python {

namespace {

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
      "spdlog-backed logger shared with the C++ core (name='xense.taccap'). "
      "Console + per-session file sink at ~/.taccaplogs/ (override via "
      "TACCAP_LOG_DIR).");

  log.def(
      "set_level",
      [](const std::string &name) {
        xense::taccap::set_console_level(parse_level(name));
      },
      py::arg("name"),
      "Set the minimum console level. One of: trace, debug, info, warn, "
      "error, critical, off. Does not affect the file sink (always DEBUG).");

  log.def(
      "get_level",
      []() -> std::string {
        auto lvl = xense::taccap::get_console_level();
        auto sv = spdlog::level::to_string_view(lvl);
        return std::string(sv.data(), sv.size());
      },
      "Return the current console level as a lowercase string.");

  log.def(
      "set_pattern",
      [](const std::string &pattern) {
        xense::taccap::set_console_pattern(pattern);
      },
      py::arg("pattern"),
      "Set the spdlog format pattern for the console sink only. "
      "The file sink keeps its archive pattern unchanged.");

  log.def(
      "flush", []() { xense::taccap::logger()->flush(); },
      "Flush both sinks.");

  // Thin per-level wrappers. Python-side formatting is the caller's job
  // (use f-strings) so we don't need spdlog's fmt arg pack here.
  log.def(
      "trace",
      [](const std::string &msg) { xense::taccap::logger()->trace(msg); },
      py::arg("msg"));
  log.def(
      "debug",
      [](const std::string &msg) { xense::taccap::logger()->debug(msg); },
      py::arg("msg"));
  log.def(
      "info",
      [](const std::string &msg) { xense::taccap::logger()->info(msg); },
      py::arg("msg"));
  log.def(
      "warn",
      [](const std::string &msg) { xense::taccap::logger()->warn(msg); },
      py::arg("msg"));
  log.def(
      "warning", // alias to match stdlib `logging` ergonomics
      [](const std::string &msg) { xense::taccap::logger()->warn(msg); },
      py::arg("msg"));
  log.def(
      "error",
      [](const std::string &msg) { xense::taccap::logger()->error(msg); },
      py::arg("msg"));
  log.def(
      "critical",
      [](const std::string &msg) { xense::taccap::logger()->critical(msg); },
      py::arg("msg"));

  // Eagerly initialize so the first call doesn't pay the registration +
  // file-sink-setup cost mid-stream.
  (void)xense::taccap::logger();
}

} // namespace xense::taccap::python
