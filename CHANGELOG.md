# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added
- Initial repository skeleton: CMake + scikit-build-core + pybind11 namespace
  alias for `xense::taccap::` / `xense.taccap`.
- libxensesdk vendored as a git submodule pinned at commit `7d4687e`,
  configured in lite mode (no ML backends).
