# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.1.7] - 2026-08-03

Sync to firmware protocol **V2.1** (`hw_v1.1.0` @ f5dd086; leader firmware
1.2.0, follower 1.1.0).

Validated on two leader grippers flashed from source to 1.2.0.0 over OTA.

> **Downstream on pybind11 2.x should update.** The Python IMU vector fix
> below changes the data affected consumers read — `accel_mps2` /
> `gyro_radps` / `mag_uT` were returning the x component three times. It only
> bites builds made against **pybind11 2.9**, which here means the system
> py3.10 wheel used by ROS 2 Humble; conda py3.12 builds (pybind11 3.0.x)
> resolved the overload correctly and were never affected. Check yours with
> `python -c "from xense.taccap import ...; print(sample.accel_mps2.strides)"`
> — `(0,)` means affected, `(4,)` means fine.

### Added
- **Fisheye camera calibration** (`Cmd 0x2B`). New `protocol::CameraFisheyeCal`
  (32 B body: `fx, fy, cx, cy, k1..k4`) and a `Calibration` component reachable
  as `g.calibration` on **both** leader and follower:
  `read_fisheye()` / `write_fisheye()`. Bound in Python, where
  `CameraFisheyeCal` also exposes OpenCV-shaped `.K` (3×3) and `.D` (4,)
  numpy views for `cv2.fisheye.undistortImage`.
- **Leader encoder max travel angle** (`Cmd 0x2C`, leader only). Same
  component: `read_encoder_max_rad()` / `write_encoder_max_rad()`. The
  follower NACKs `InvalidCmd` — it has no MT6816.
- **`ErrorCode::CalNotSet` (0x60)** — the firmware returns this instead of
  zeros when a calibration record has never been written, so "never
  calibrated" is distinguishable from "calibrated to exactly 0". Both read
  methods surface it as an empty `std::optional` / Python `None`; every other
  error still throws `ProtocolError`.
- **Normalized leader gripper position.** `EncoderSample` gains
  `position` — the opening in `[0,1]` (0 = closed, 1 = fully open), derived
  from the encoder-max calibration. `LeaderGripper::Config` gains
  `normalize_position` (installs the converter on the `Encoder`, so one-shot
  reads *and* streamed samples carry it) and `encoder_max_rad` (host-side
  override that skips the firmware read — needed on pre-V2.1 firmware).
  `LeaderGripper` also gains `position()` / `pos_to_rad()` / `rad_to_pos()` /
  `position_map()` / `reload_position_map()`, mirroring `FollowerGripper`.
- `Encoder::set_position_map()` / `clear_position_map()` /
  `has_position_map()` / `position_map()`; `GripperPosition::from_travel()`
  for calibration sources that aren't a `GripperConfig` record.
- `python/examples/fisheye_cal.py` (show / set-fisheye / set-encoder-max /
  guided `measure-encoder-max`) and
  `python/examples/leader_normalized_position.py`.

### Fixed
- **Python IMU vectors returned the x component three times.**
  `ImuSample.accel_mps2` / `.gyro_radps` / `.mag_uT` came back as `[x, x, x]`:
  `make_vec3` built its array with `py::array_t<float> arr(3)`, which on
  pybind11 2.9 picks a different overload and yields a shape-(3,) array with
  **stride 0** — a broadcast view of element [0], so the writes to `p[1]` and
  `p[2]` landed on the same address. No error, no crash, just silently wrong
  data on every Python IMU read.
  **Scope: builds made against pybind11 2.9 only.** Here that is the system
  py3.10 wheel (ROS 2 Humble); the conda py3.12 builds use pybind11 3.0.x,
  which resolves the overload correctly — verified against a pre-fix 3.0.4
  build that reported correct per-axis strides. The C++ side was never
  affected either way. Spelling the shape as a container makes it correct on
  every pybind11 version rather than relying on which one happens to be
  installed.
  Pre-existing, and independent of the V2.1 work — reproduced on a gripper
  still running the old firmware. Verified on two units: before
  `accel=[9.561, 9.561, 9.561]` (|a| = 16.56), after
  `accel=[9.660, 0.020, -2.412]` (|a| = 9.96 ≈ g). Same overload trap as
  `CameraFisheyeCal.D`; `make_vec3` was the last remaining instance.
  **Downstream consumers should update** — this changes the data they see.
- **A failed OTA reported success.** `OtaSession` only checked
  `ack.is_nack`, but firmware handler errors take the echoed-cmd path
  (`protocol_send_response(seq, cmd, err, NULL, 0)` — command byte intact,
  error as the whole payload), which the transport surfaces as a
  "successful" 1-byte response. A rejected `OtaWriteBlock` therefore did not
  throw: the loop wrote every remaining block, `verify()` and `apply()`
  swallowed their errors too, and `update_from_file()` returned normally on a
  firmware that had aborted the session. Nothing was bricked — the firmware
  refuses to swap banks — but the host lied about it. All OTA calls now go
  through the new `bus::ack_error_code()`.
- **Host retry could kill an otherwise fine OTA.** `Transport::send_cmd`
  allocated a fresh seq per retry attempt, so a merely-slow `OtaWriteBlock`
  ACK caused the same offset to be re-sent as a brand-new request. The
  firmware demands strictly sequential offsets and marks the whole session
  failed on a repeat (`ota_driver.c` — `offset != bytes_written` ⇒
  `OTA_STATE_ERROR`). New `bus::RetryMode::SameSeq` reuses the seq so the
  firmware recognises the repeat and replays its cached response instead of
  re-running the handler — which is exactly what firmware V2.1's
  `protocol_resend_cached_response` was added for, and which the SDK could
  never reach while it bumped the seq. `OtaSession` uses it for every
  command; `RetryMode::NewSeq` stays the default everywhere else, so no other
  call site changes behaviour.
- **`OtaStart` could time out on the follower.** Firmware >= V2.1 stops the
  motor and switches it to MIT inside the handler (300 ms feedback confirm +
  CAN-id probes + parameter waits + two flash writes), which can exceed the
  old 1000 ms default; the retry then found the session already open and got
  `OtaBusy`. Default raised to 5000 ms, and `write_block` 500 → 1000 ms
  (every 8th block triggers an 8 KB erase + program).
- **Process abort at interpreter shutdown** (`FATAL: exception not rethrown`)
  when a Python `on_data` / `on_status` subscriber was still registered at
  exit. The transport reader thread is a plain `std::thread` that nothing
  stops at teardown, so a DATA frame arriving after `Py_Finalize` — easy to
  hit, because the firmware keeps flushing queued frames for a while after
  `StopStream` — called into a finalized interpreter and aborted inside
  `PyGILState_Ensure`. Pre-existing; reproduced at 0.1.6 with none of the
  V2.1 work applied. Three parts:
  - `bus::Transport::stop()` now drops all subscriptions **before** joining
    the reader, so no callback can be entered once shutdown has begun and
    every callback object is destroyed on the caller's thread. The callbacks
    are destroyed outside `sub_mu_` — their destructors take the GIL, and
    holding the subscription mutex across that invites a lock-order inversion
    against the reader.
  - The Python binding's callback wrappers and the GIL-acquiring `shared_ptr`
    deleter now check for a finalizing/finalized interpreter and drop the
    late event (the deleter leaks the object rather than aborting — the
    interpreter is tearing down anyway).
  - `Motor.on_status` used a bare `make_shared` instead of the GIL-safe
    wrapper every other component uses — the exact hazard. Fixed.

### Changed
- `EncoderSample::position_rad` is **unchanged** — it still always reports
  radians. Normalization adds the `position` field rather than repurposing an
  existing one; `position` is NaN while no map is installed.
- **`calibrate.py` now stores the travel span instead of checking it.** It
  used to compare the measured full-open angle against a 1.7 rad "design
  baseline" and warn on deviation, while never storing the value anywhere
  (it predates `Cmd::EncoderMaxCal`). Both halves were wrong: the span *is*
  the calibration, and the baseline was stale — three measurements across two
  units gave 1.1582 / 1.1589 / 1.1486 rad (~66°), all outside its ±0.5 band.
  `--expected-max-open-rad` and `--open-tolerance-rad` are **removed**; step 2
  now persists the measurement, and the firmware-capability probe runs
  *before* the zero is latched so a pre-V2.1 gripper is never left
  half-calibrated. New shared `python/examples/_calib_flow.py` backs
  `calibrate.py`, `fisheye_cal.py` and `leader_normalized_position.py`; the
  latter now offers the guided flow when it finds no calibration at startup.
  It lives in `examples/`, not the package: it prompts on stdin, and the SDK
  must stay usable headless.
- **`LeaderGripper.__exit__` / `FollowerGripper.__exit__` now stop the
  transport**, not just the stream, so the reader thread and its callbacks are
  gone at the end of the `with` block. A gripper used after its `with` block
  now raises `IoError("send_cmd on stopped transport")` instead of appearing
  to work; re-entering a `with` block on the same object is no longer
  supported. This matches `ControlLoop.__exit__`, which already stopped.

### Notes
- The firmware reports handler errors via
  `protocol_send_response(seq, cmd, err, NULL, 0)`, which echoes the command
  byte, so `bus::Transport` surfaces them as a "successful" 1-byte response
  rather than a NACK. `Calibration` resolves that locally (its success
  responses are 33 B / 5 B / `[0x00]`, so a non-zero 1-byte payload is
  unambiguously an error). Transport semantics are unchanged; commands whose
  legitimate success response *is* a single byte (`MotorGetCanId`,
  `MotorGetProtocol`) would need per-command knowledge to disambiguate.
- Firmware V2.1 also raised the auto-cal stall-torque defaults (0.22/0.20 →
  0.35/0.35 Nm) and shortened `stall_hold_ms` (120 → 30) /
  `post_zero_delay_ms` (100 → 30). `gripper_auto_cal_config_t` is still 32 B
  and the SDK reads the config from the device, so no SDK change — but old
  devices get migrated to the new defaults on flash.
- Firmware V2.0 made the cal-result commands (`0x27`/`0x28`/`0x29`) available
  on the follower as well as the leader.

## [0.1.6] - 2026-07-06

Sync to firmware `hw_v1.1.0` @ ab3f98c.

### Added
- **Private-protocol single-parameter access** (`Cmd 0x38/0x39`). New
  `Motor::get_private_param(index)` / `set_private_param(index, raw_value)` and
  `MotorPrivateParam` (8 B: index / type / access / raw_value), bound in Python.
  Only valid when the motor runs the Private CAN protocol — under MIT (the SDK's
  assumed mode) these NACK `InvalidParam`. The firmware whitelists index + R/W.

### Notes
- Firmware changed the *default* auto-cal speeds / confirm count (close 0.15→0.25,
  open 0.20→0.35, confirm 3→1). These are firmware-side `#define`s applied on the
  device; the SDK reads `GripperAutoCalConfig` from the device, so no SDK change.

## [0.1.5] - 2026-07-03

Sync to firmware protocol **V1.9** (`hw_v1.1.0` @ 94273b4).

### Changed
- **BREAKING (wire): `motor_status_t` shrank 40 → 31 bytes.** Firmware V1.9
  dropped `actual_current` / `target_current` / `current_source`; the SDK's
  `protocol::MotorStatus` and `MotorStatusSample` drop them too, and
  `control_mode` now follows `target_torque` directly. The first 18 bytes
  (`actual_pos`..`status`) are unchanged, so position / torque / status stay
  correct across firmware versions — only `target_*` / `control_mode` require
  V1.9 firmware (a V1.9 SDK reading an older 40-byte status gets the wrong
  `target_*`). Decode targets the 31-byte layout.

### Added
- **Gripper power-on auto-calibration config** (`Cmd 0x68/0x69`). New
  `GripperAutoCalConfig` (32 B) + `FollowerGripper::get_auto_cal_config()` /
  `set_auto_cal_config()`, bound in Python. When enabled, the firmware
  self-calibrates on power-up (close-to-stall ⇒ zero, open-to-stall ⇒
  max_open), automating the manual zero + max_open capture.
- **WS2812 LED control** (`Cmd 0x0A/0x0B`). New `Led` component
  (`set()` / `off()` / `effect()` / `effect_off()`) on both `LeaderGripper` and
  `FollowerGripper` (`g.led`), with `Ws2812Set` / `Ws2812Effect` payloads and
  `Ws2812Mode` / `Ws2812EffectType` enums (blink / breathe / HSV / LERP +
  presets). Bound in Python.
- Documented the V1.9 external-control gate: while the firmware owns the motor
  (e.g. power-on auto-cal), `enable`/`set_*` NACK `SysBusy` and `submit_*` are
  silently dropped; there is no query for the state — watch
  `control_stats().applied_seq`.

## [0.1.4] - 2026-06-26

### Fixed
- **Logger no longer crashes when another spdlog copy is loaded in-process.**
  `xense::taccap::logger()` used to fetch the logger from spdlog's global
  registry (`spdlog::get`), whose singleton is a process-wide `STB_GNU_UNIQUE`
  symbol. When a consumer also loads a different spdlog version (e.g. the one
  bundled in `xensesdk`'s `libxense_c.so`), the two share one mismatched-layout
  registry and `get()` returns `nullptr` → the first `logger()->...()` call
  segfaults. The logger is now built once and cached in `cpp/src/log.cpp`
  without touching the registry, making it independent of any other spdlog in
  the process.

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
