# TacCap-Gripper — Claude working notes

Loaded into every Claude session in this repo. Keep terse — auto-memory
carries deep background; this file is *house rules*.

## Repo identity
- C++17 SDK + Python (pybind11) bindings. Apache-2.0. Long-term maintained.
- Sister repo `taccap_gripper_ros2` (under `~/taccap_ros2_ws/src/`) only
  *imports* the `xense.taccap` Python package; it does **not** reimplement
  any lower-layer comms. The two repos release independently.

## Build & test (C++)
- Build dir is `build/` (existing CMake/Ninja generator).
- Build a target: `cmake --build build --target <tgt>` (e.g. `taccap_unit_tests`).
- Run full test suite: `./build/cpp/tests/taccap_unit_tests`
- Run one suite: `./build/cpp/tests/taccap_unit_tests --gtest_filter='<Suite>.*'`
- After touching protocol / payload structs, verify the
  `static_assert(sizeof(...) == N)` lines in
  `cpp/include/taccap/protocol/payloads.hpp` — they fail the build the
  instant firmware-side layout drifts.

## Build & install (Python wheel)
- conda env `taccap` (py3.12, primary dev env):
  `pip install -e . --no-build-isolation`
- System py3.10 (used by ROS2 Humble):
  `/usr/bin/python3 -m pip install --user --no-build-isolation .`
- Examples are off by default; enable with `-DTACCAP_BUILD_EXAMPLES=ON`.
- Active conda env's python: `/home/ubuntu/miniforge3/envs/taccap/bin/python`.

## Hardware smoke test (when a gripper is plugged in)
```bash
python -c "from xense.taccap import scan_grippers, Side
for g in scan_grippers():
    s='L' if g.side==Side.Left else 'R'
    print(f'  [{s}] ch343={g.mcu_serial} fw_sn={g.firmware_sn!r}')"
```
Healthy output: `[L]` + `[R]`, both with non-empty `firmware_sn`. Empty SN
means firmware hasn't burned the SN yet, or firmware < V1.6 — fall back to
`python/examples/v4l2_probe.py` for raw V4L2 bringup.

## Commit convention
- Conventional commits with subsystem scope:
  `feat(protocol): ...`, `fix(parser): ...`, `test: ...`, `chore: ...`,
  `feat(examples): ...`
- Do **not** use `--no-verify`. If a hook fails, fix the underlying issue.
- Do **not** amend already-pushed commits — dual-remote sync becomes
  very painful afterwards.

## Push order (two remotes)
Remote naming in this repo is **counter-intuitive**:
`origin` = GitLab (internal), `github` = GitHub (public).
Default is push to GitHub first (it may have external contributions),
then GitLab:
```bash
git push github main
git push origin main
```
GitLab `main` is a protected branch. **Before any force-push**, go to the
GitLab web UI (`Settings → Repository → Protected branches`), temporarily
unprotect, push, then **immediately re-protect**. Never force-push to
`main` as a shortcut — ask first.

## Logging
- The entire SDK uses **one singleton logger**: `xense::taccap::logger()`,
  registered with spdlog under the name `"xense.taccap"`. C++ and Python
  share the same instance and the same sinks. Never construct an ad-hoc
  `std::make_shared<spdlog::logger>(...)` elsewhere.
- Do not use `std::cout` / `printf` / `print()` / `std::cerr` for diagnostic
  output — go through the logger. Examples and one-shot CLI tools (e.g. a
  `scan_grippers` table printed for a human) may use `print` / `std::cout`
  because that output *is* the program's feature, not logging.
- C++: `#include <taccap/log.hpp>`, then `xense::taccap::logger()->info(...)`.
- Python: `from xense.taccap import log; log.info(...)` /
  `log.set_level("debug")`.
- The ROS2 sibling repo is out of scope here — it uses `rclpy`'s
  `Node.get_logger()` instead.

### Sinks (both attached by default)

| Sink | Level | Pattern (constant in `log.hpp`) |
|---|---|---|
| stderr (color) | user-controllable (default INFO) | `SPDLOG_PATTERN` = `[%D %T.%e] [%n] [%^%l%$] %v` |
| file (per-session) | always DEBUG | `FILE_LOG_PATTERN` = `[%Y-%m-%d %H:%M:%S.%e] [%n] [%l] %v` |

The logger itself sits at DEBUG so the file sink sees everything; the
console sink filters via its own sink-level. `log.set_level(...)` and
`log.set_pattern(...)` affect the **console sink only**; the file sink's
archive format never changes (keeps historical greps parseable).

### File sink behavior
- Directory: `$TACCAP_LOG_DIR` if set, else `~/.taccaplogs/`.
- Filename: `session_YYYYMMDD_HHMMSS.log`. One new file per process start.
- On startup, files are sorted by mtime and the oldest are deleted so at
  most `kMaxSessionLogs` (= 10) session logs remain.
- File-sink creation failures (disk full, permission denied, missing path
  that can't be created) must **not** be fatal — the console sink keeps
  working regardless.

## Load-bearing constraints (don't touch unless asked explicitly)
- `third_party/libxensesdk/src/device/sensor.cpp` has
  `set_ctrl(V4L2_CID_BRIGHTNESS, 38, ...)`. The literal `38` is a
  calibration baseline, not a placeholder. A controlled experiment on
  2026-05-06 confirmed the V4L2 control propagates and that other values
  produce visibly wrong images. If a "make brightness configurable" task
  appears, push back before changing the literal.
- `third_party/firmware/` is a **clone-on-demand** firmware reference dir,
  **not** a submodule. `.gitignore` already excludes it. Never `git add -f`
  or convert it into a submodule.
- Side L/R detection reads the firmware-burned SN via `Cmd::GetSn`, **not**
  the CH343 USB chip SN.
- ROS2 nodes must run with `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` and QoS
  `BEST_EFFORT`. Fast-DDS drops 20–35% of image frames on this setup.
- ROS2 image bytes go through `msg.data = array.array('B', arr.tobytes())`
  — assigning a `bytes` object directly hits a slow rclpy octet[] path.

## Risky actions — confirm before
- Running `python/examples/ota_update.py` (flashes firmware; wrong artifact
  bricks the MCU).
- Any change under `third_party/firmware/` or to the firmware-protocol
  mirror headers in `cpp/include/taccap/protocol/`.
- `git push --force*` to `main` on either remote.
- Stopping the system ROS2 daemon or editing cyclonedds config files.

## When in doubt
Auto-memory in `~/.claude/projects/-home-ubuntu-TacCap-Gripper/memory/`
holds the deeper background (subsystem map, runtime gotchas, wire-protocol
notes, firmware stream-dup behavior, etc.) and is loaded automatically.
Reference it when relevant.
