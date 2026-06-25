# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.1.4] - 2026-06-26

### Removed
- **Dropped the `libxensesdk` dependency and the C++ visuotactile path.** The
  `third_party/libxensesdk` git submodule, the `vision.hpp` alias header, and the
  `TactileSensor` / `TactileFrame` classes are gone, along with the
  `LeaderGripper` / `FollowerGripper` `tactile_left()` / `tactile_right()`
  accessors and their `tactile_*_serial` / `rectify_tactile` config. Visuotactile
  (OG) capture and rectification now live at the Python level via the `xensesdk`
  wheel; `xense.taccap` is the gripper-protocol + wrist-camera surface only.
- Removed the now-unused `libxense_version` attribute.

### Changed
- Build no longer requires internal-network submodule access. CMake no longer
  pulls in `libxensesdk`; `taccap_core` links `opencv_core` + `opencv_videoio`
  directly (previously transitive through libxense). Dropped the
  `eigen` / `openssl` / `zlib` / `nlohmann_json` build deps (all libxense-only).

## [0.1.3] - 2026-06-25

### Added
- **MIT force-position control submission path** on `Motor`: no-ACK
  `submit()` overloads (impedance / position / velocity / torque) plus float
  `submit_impedance()` / `submit_position()` / `submit_velocity()` /
  `submit_torque()` wrappers, bound in Python. These send `CMD_NO_ACK` frames
  fire-and-forget for a host-driven realtime loop up to the firmware's 500 Hz
  slave-control rate — no ACK, retry, or throw (only `IoError` on a stopped
  transport). Health is out-of-band via `control_stats()` / `on_status()` /
  `SensorErrors`. The MIT impedance frame is the force-position hybrid primitive
  (kp/kd track `target_pos`; `feedforward_torque` adds the force term). The
  follow/teleop loop and grasp FSM stay in the upper layer.
- `python/examples/motor_mit_control.py`: primitive demo of the submission API
  with the out-of-band health channel.
- **Normalized gripper position** (0 = closed, 1 = open). New `GripperPosition`
  pure converter (raw shaft rad ↔ normalized [0,1], built from `GripperConfig`)
  and `FollowerGripper::position()` / `set_position(pos, kp, kd, ff)` /
  `pos_to_rad()` / `rad_to_pos()` / `position_map()` / `reload_config()`, bound
  in Python. `set_position()` is the normalized counterpart of
  `Motor::set_impedance` (fire-and-forget, no ACK). NOTE: `FollowerGripper::
  set_position()` is normalized [0,1] and distinct from `Motor::set_position()`
  (raw rad). Throws if the gripper isn't calibrated (`GripperConfig` not Valid).
  Validated on real follower hardware (max_open = 1.1802 rad, Reverse).
- **`ControlLoop`** — a fixed-rate send/receive loop for embodied control. A C++
  background thread submits the latest normalized position target as a MIT
  impedance frame at `hz` (fire-and-forget), while the firmware motor-status
  STREAM keeps a thread-safe `GripperObservation` fresh. The policy thread only
  touches `set_target(0..1)` and `observation()` (both non-blocking). Reads
  observations from the push stream instead of polling `GetMotorStatus` (polling
  > ~100 Hz can stall the firmware's status refresh). Bound in Python with
  context-manager support. Validated on hardware (200 Hz submit, ~100 Hz obs,
  obs age a few ms).
- `python/examples/gripper_control_test.py`: interactive open/close control test
  exercising both `set_position()` (one-shot) and `ControlLoop` (realtime).

### Changed
- Promoted the V1.7 follower / motor command surface from "reserved, pending
  hardware" to first-class, validated against firmware `hw_v1.1.0`. The leader
  mismatch behavior is unchanged: these NACK `SensorOffline` → `ProtocolError`
  on leader hardware.

### Fixed
- **Discovery never guesses a side from the CH343 chip SN.** Side now comes from
  firmware sources only — the burned SN (`Cmd::GetSn`, sequence-digit parity) with
  `GetDevType` as a secondary firmware fallback; when neither answers the side is
  reported as the new `Side::Unknown` (bound in Python) instead of the WCH chip
  SN's meaningless parity, which could confidently report the wrong side. The
  `GetSn` probe in `scan_all()` now retries on cold start (the first command(s)
  after a fresh plug-in could be dropped while the USB-CDC link settled, which
  previously left the side falling back to the chip SN on the very first scan).
  `McuEndpoint` drops its chip-parity `side` field. **Minor API addition**
  (`Side::Unknown`); existing `Left`/`Right` are unchanged.

## [0.1.1] - 2026-06-14

### Fixed
- **V1.8 global byte stuffing** in the framing layer (`pack_frame` escapes the
  body ADDR..CRC; `try_parse_frame` is TAIL-delimited and unstuffs before the
  CRC check, which stays over the unescaped HEAD..PAYLOAD). The firmware and PC
  tool escape the wire as of V1.8 — without this, `GetSn` and any frame whose
  body contains 0xAA/0x55/0x7D silently timed out.
- libxense (submodule): VID:PID `3938:1300` added to the device whitelist and
  the `GSPS` serial prefix mapped to the Omni sensor type.

### Changed
- **Discovery is MCU-only.** `scan_grippers()` no longer enumerates the wrist
  camera or visuotactile sensors (an external camera service owns them);
  `GripperEndpoints` drops `wrist_video` / `tactile_*_serial`. `LeaderGripper` /
  `FollowerGripper` no longer open cameras at construction — gated behind a new
  `open_cameras` flag (default off). **Breaking.**
- Examples reworked: `leader_demo` / `calibrate` / `ota_update` are MCU-only;
  `rerun_visualize` opens wrist/tactile only via `--wrist` / `--tactile-*`;
  `rerun_dual_with_tracker` drops the camera panels.

### Added
- **TacCap SN scheme** (`TCGU01A24Z0001m` / `GSPS01A24Z0001`): `parse_serial()`,
  a `Role` enum, `GripperEndpoints.role`, and `find_leader()` / `find_follower()`.
  Side comes from the SN sequence digit, with a `GetDevType` (firmware
  LEFT/RIGHT) fallback, then CH343 chip-SN parity.
- **V1.7 command set** (follower / motor — interfaces reserved, pending
  follower hardware): motor set-zero, CAN-id read/write, protocol switch/query,
  control-stats, and follower gripper-config get/set. New `GripperConfig`,
  `MotorControlStats`, `MotorProtocol`; `MotorStatus` grew 18→40 B and
  `MotorImpedanceCtrl` 16→20 B (lenient decode keeps legacy 18 B working).

## [0.1.0] - 2026-05-27

First usable release. Everything below landed on `main` since the
v0.0.1 bootstrap (c9e8267) — protocol, transport, components, both
gripper aggregates, Python bindings, six example scripts, logging,
encoder calibration ergonomics, and a dual-gripper + Pico-tracker
visualiser.

### Added

**Protocol layer**
- TC-GU-01 wire protocol mirror in C++17 — Cmd enum, FrameType, ErrorCode,
  POD payload structs (IMU / Encoder / KeyStatus / SensorError / OTA /
  IMU MagCal / EncoderConfig). 1:1 with firmware `protocol_cmd.h` /
  `protocol_data.h`, pinned by `static_assert(sizeof(...) == N)`.
- V1.6 mirror — OTA session commands, KeyStatus DATA, IMU MagCal,
  per-sensor calibration result flags, SensorError reports.

**Bus / transport**
- Async `bus::Transport` over termios serial with ACK matching, retry,
  and per-command DATA subscriber dispatch (single reader thread).
- Frame parser with HEAD/TAIL detection + CRC16 verification, including
  recovery from false-positive HEAD bytes mid-stream.

**Components**
- `IMU`, `Encoder`, `Camera`, `TactileSensor` (V4L2 + libxense XU
  rectify), `Motor` (follower-only, FDCAN-via-MCU).
- `LeaderGripper` / `FollowerGripper` aggregates with `read_once` +
  `on_data` callback patterns on every component.
- `IMU::set_mag_calibration(hard, soft)` — write hard-iron + soft-iron
  matrix to firmware (Cmd::SetImuMagCal, 48-byte payload).
- `Encoder::set_zero(timeout=500ms)` — latch current encoder reading
  as the new zero position (Cmd::SetEncoderZero). Throws on NACK /
  timeout.
- `Encoder::normalize()` post-process — clamp `position_rad` to ≥ 0
  to absorb post-calibration drift; rate-limited warning (1 / s per
  instance) when raw drift exceeds -0.1 rad. Raw firmware value
  preserved in `raw.position_rad`.
- `OtaSession` — full V1.3 OTA state machine, including the high-level
  `update_from_bytes()` orchestrator (start / write_block / verify /
  apply / status polling).
- `Key` + `SensorErrors` DATA subscribers (V1.4 / V1.6 streams).

**Discovery**
- `scan_grippers()` enumerates all plugged grippers; `find_left()` /
  `find_right()` / `find_one()` typed lookups.
- Bilateral discovery via USB hub-path grouping so two grippers sharing
  a hub are reliably split into left/right endpoints.
- Side detection reads firmware-burned SN via `Cmd::GetSn` (not the
  CH343 USB chip SN) — survives MCU swaps without relabeling sides.

**Python bindings (pybind11)**
- `xense.taccap` package exposing all components, `LeaderGripper` /
  `FollowerGripper`, `GripperEndpoints`, `Side`, `Cmd` enum,
  `EncoderSample.raw_position_rad` / `raw_velocity_rad_s` for
  pre-clamp diagnostics.
- `xense.taccap.log` submodule — `set_level` / `set_pattern` /
  `info` / `debug` / `warn` / `error` etc., shares the underlying
  C++ spdlog instance (one logger program-wide).
- Python 3.10 + 3.12 build paths (system py3.10 for ROS 2 Humble,
  conda py3.12 for primary dev).
- GIL-safe shared-ptr deleter for callbacks held across worker threads.

**Logging**
- Single-instance `xense::taccap::logger()` (header-only, spdlog
  registry-backed) shared between C++ and Python.
- Two sinks attached by default: stderr (colour) with user-controllable
  level, file (per-session) always DEBUG.
- File sink writes `session_YYYYMMDD_HHMMSS.log` under `$TACCAP_LOG_DIR`
  (default `~/.taccaplogs/`). At most `kMaxSessionLogs` (= 10) sessions
  retained; oldest mtime pruned at process start. File-sink failures
  degrade gracefully — console keeps working.
- Constants `SPDLOG_PATTERN` / `FILE_LOG_PATTERN` define the canonical
  console + archive formats.

**Examples**
- `python/examples/rerun_visualize.py` — single-gripper rerun-sdk
  multimodal viewer (wrist + 2× tactile raw/rect + IMU/encoder time
  series + observed FPS panel).
- `python/examples/v4l2_probe.py`, `v4l2_sweep.py` — manual V4L2
  bringup probes; useful when firmware SN isn't burned yet.
- `python/examples/ota_update.py` — firmware OTA CLI with progress +
  status verification.
- `python/examples/calibrate.py` — per-SN encoder zero calibration
  with raw/cooked side-by-side display, full-open angle sanity check,
  live readout.
- `python/examples/rerun_dual_with_tracker.py` — dual-leader viewer
  augmented with Pico4 motion-tracker 6-DoF poses. JPEG-compressed
  image streams + tight rerun flush knobs for low-latency teleop.

### Changed

- ACK wire format corrected — `cmd=0` is NACK; ACK frame's payload
  carries the cmd's return data, not a status struct.
- `cmd=0` ACK with `err=Ok` now treated as success (firmware quirk on
  some no-payload commands).
- Firmware reference repos (`tc-gu-01`, `tc-gu-01-pc`) relocated to
  `third_party/firmware/` and made clone-on-demand (gitignored, never
  submodules — they have separate release cadences).
- `LeaderGripper` constructor pre-stops any in-flight firmware stream
  before opening + bumps the ACK timeout to absorb the post-reset
  warm-up window.
- `LeaderGripper` ctor logs firmware version + SN — visible in every
  session log without needing extra scaffolding.

### Fixed

- `bus::Transport` parser rewinds past a false-positive HEAD byte
  when downstream framing reports NeedMoreData, preventing stuck
  frames on noisy serial.
- `Encoder` / `IMU` / `Camera` callbacks: GIL-safe `shared_ptr<py::function>`
  deleter so worker threads can release callbacks without UB.
- Logger: replaced the per-TU static cache (which produced split
  storage across `_taccap_native.so` and `libtaccap_core.so` under
  `-fvisibility-inlines-hidden`) with stateless `spdlog::get()`
  lookups — fixes a SIGSEGV in `LeaderGripper` construction.
- `rerun_visualize.py` summary FPS anchored to streaming-start instead
  of process-start so ~4 s of libxense/V4L2 init doesn't drag the
  reported rate.

### Tests

- gtest suite covers protocol codec round-trip, frame parser edge
  cases, transport ACK/NACK paths over PTY, all component decoders,
  V1.3-V1.6 end-to-end (Key / SensorErrors / IMU MagCal / OTA), CRC32
  boundary cases against reference zlib vectors, encoder set_zero
  wire format + NACK, encoder normalize clamp + warn behaviour.
- 126 tests pass; PTY-driven fake-firmware harness in
  `cpp/tests/pty_helper.hpp` for transport-level coverage.

### Docs

- `README.md` — Python + C++ install flows, hardware smoke test,
  example index, calibration walkthrough, logging behaviour.
- `docs/ARCHITECTURE.md` — layered stack, module map, data-flow
  diagrams, threading model, USB-topology discovery, boundary
  between this SDK and downstream consumers (dataset recording /
  ROS 2 / lerobot adapters).
- `CLAUDE.md` — house-rules file for AI-assisted maintenance.

## [0.0.1] - 2026-04-29

### Added
- Initial repository skeleton: CMake + scikit-build-core + pybind11
  with the `xense::taccap::` / `xense.taccap` namespace alias.
- libxensesdk vendored as a git submodule pinned at commit `7d4687e`,
  configured in lite mode (no ML backends).
