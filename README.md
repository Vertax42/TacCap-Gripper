# taccap-gripper

C++17 / Python SDK for the **TacCap-Gripper** —— XenseRobotics' multimodal
tactile data-collection gripper. Exposes a single namespace
(`xense::taccap::` / `xense.taccap`) for:

- IMU + encoder readout via the TC-GU-01 serial protocol
- Motor control (follower side, FDCAN→灵足 transparently routed via MCU)
- Leader / follower gripper objects that aggregate the MCU sensors and
  expose zero-config discovery
- Standalone `Camera` (wrist UVC, plain OpenCV V4L2) — **opt-in**: an
  external camera service owns the V4L2 devices now, so the gripper
  aggregates do **not** open it unless constructed with `open_cameras=True`.
  The **visuotactile (OG) sensors are not handled in this SDK**; capture and
  rectification live at the Python level via the `xensesdk` wheel.

Two adapter repos build on it and are released independently:

- `taccap_gripper_ros2` — ROS2 (Humble + Jazzy) hardware interface package
- a fork of `lerobot-xense` with a `taccap_gripper` robot class

Both only *import* the `xense.taccap` package; neither reimplements device
access, and neither is required to use this SDK.

## Status

**v0.1.9.** Command set **V2.2**, wire framing **V1.8**. Hardware-validated on
bilateral leader setups and on real follower grippers — including the V2.2
follower diagnostics, the MIT force-position control path, and `ControlLoop`
under a full production load (all cameras streaming, motor cycling).

**Firmware minimums:** leader >= 1.2.0, follower >= 1.1.0. V2.2 follower
diagnostics need follower >= 1.1.2. These are floors, not exact matches —
newer commands fail loudly with `ProtocolError(InvalidCmd)` rather than
misbehaving, and payload length is never a version probe. Check what a device
answers with `python python/examples/fisheye_cal.py show`.

> **[`firmware/`](firmware/) ships leader 1.2.2 and follower 1.1.5**, both local
> builds, both hardware-validated on two units each. They carry three fixes that
> live in code the two roles share: a command-channel livelock under sustained
> high-rate input, a blocking-log path that stalled realtime tasks, and an
> out-of-bounds write on every boot. Note that leader 1.2.2 replaces an
> *official* 1.2.1, so it trades that provenance for the fixes — see
> [`firmware/README.md`](firmware/README.md). **Power-cycle after any flash.**

### What's in

- **TC-GU-01 protocol** — async transport with ACK matching, per-command DATA
  subscribers, byte-stuffed framing.
- **Follower motor control (MIT force-position).** `Motor` enable / disable /
  clear-fault + four control modes. Blocking-ACK `set_*` and no-ACK `submit_*`.
- **`ControlLoop`** — the recommended way to drive a follower. Submits the
  latest normalized target **in phase with the motor-status stream** and keeps
  a thread-safe observation fresh, so your policy only touches
  `set_target(0..1)` and `observation()`, both non-blocking. See
  [Follower gripper control](#follower-gripper-control-mit-force-position) for
  why the phase matters.
- **`ForcePositionController`** — contact-aware hybrid grasping for a follower:
  velocity-damped close, then a pure bounded `tau_ff` hold with `kp=kd=0` so
  blocked-jaw position error cannot keep increasing torque. Runtime
  `set_target(0..1, torque)` supports arbitrary opening targets. It separates
  the device/motion transient limit (up to 6 Nm) from the pure force-hold
  software ceiling (up to 1.8 Nm).
- **Normalized position** on both roles — `[0, 1]`, 0 = closed, 1 = open, on
  one-shot reads and on every streamed sample.
- **`Diagnostics`** (`g.diagnostics`) — the firmware's own UART counters and a
  runtime log switch. Answers a question the host cannot answer alone: when a
  frame arrives short, did the MCU fail to send it, or was it lost afterwards?
- **Wrist fisheye undistortion** — firmware-stored intrinsics turned into
  cached remap tables, standalone or wired in via `undistort_wrist=True`.
- **Calibration** (`g.calibration`) — the flash-persisted fisheye and
  encoder-max records. See [docs/CALIBRATION.md](docs/CALIBRATION.md).
- **LEDs, power-on auto-calibration, OTA, zero-config discovery** by
  firmware-burned SN (never the CH343 chip SN).

Visuotactile (OG) capture lives at the Python level via the `xensesdk` wheel —
`xense.taccap` is the gripper-protocol + wrist-camera surface only.

Full per-commit history in [CHANGELOG.md](CHANGELOG.md).

---

## Install

```bash
mamba env create -f environment.yml && mamba activate xense-taccap
pip install -e . --no-build-isolation
python -c "import xense.taccap as t; print(t.__version__)"
```

Two gotchas that cost people an afternoon each:

- Call `pip` from an **activated** env, not by absolute path. Otherwise cmake
  finds a `ninja` on `PATH` that is actually GNU Make and fails with a
  confusing version error.
- A C++ or bindings change is **not live in a consumer env until you reinstall
  there** — the editable install redirects Python sources to the checkout but
  keeps serving the compiled extension from `site-packages`.

Prerequisites, device permissions, C++-only builds, rebuild/clean and the
`PYTHONPATH` / `LD_LIBRARY_PATH` traps: **[docs/INSTALL.md](docs/INSTALL.md)**.

---

## Quick start

### Single gripper

```python
import xense.taccap as t

# Auto-discover the one connected gripper (left or right) by its MCU serial.
# Throws IoError if 0 or >1 grippers are plugged in — use the explicit
# constructor (below) for bilateral setups.
gripper = t.LeaderGripper.open()      # MCU-only; cameras stay off
gripper.start_streaming(imu_hz=100, encoder_hz=100)

enc_sub = gripper.encoder.on_data(lambda s: print("enc", s.position_rad))
imu_sub = gripper.imu.on_data(lambda s: print(s))

# ... do work ...
gripper.stop_streaming()
```

The wrist camera is owned by an external camera service, so `open()` does
not touch it. To have a gripper drive the wrist UVC camera, construct it
explicitly with `open_cameras=True` and the device path:

```python
g = t.LeaderGripper(mcu_device, wrist_video="/dev/video2", open_cameras=True)
g.wrist_camera.start(lambda f: print("wrist", f.frame_index))
```

Or open it on its own with the standalone `t.Camera` class — independent of
any gripper. The visuotactile (OG) sensors are read separately via the
`xensesdk` wheel, not through this SDK.

### Bilateral (left + right in one process)

```python
from xense.taccap import LeaderGripper, scan_grippers, Side

# scan_grippers() returns all endpoints in one USB sweep — no re-probe
# race when you ask for both sides.
endpoints = scan_grippers()
left  = next(e for e in endpoints if e.side == Side.Left)
right = next(e for e in endpoints if e.side == Side.Right)

# Alternatively: t.find_left() / t.find_right() are typed wrappers
# that throw if the requested side isn't visible.

def _open(eps):
    return LeaderGripper(eps.mcu_device)   # MCU-only; cameras off by default

g_left, g_right = _open(left), _open(right)
g_left.start_streaming(imu_hz=100, encoder_hz=100)
g_right.start_streaming(imu_hz=100, encoder_hz=100)
# ... attach callbacks, stop_streaming() on exit ...
```

### Serial numbers (TacCap SN scheme)

The firmware-burned SN encodes both the side and the leader/follower role:

```
  TCGU01 A24 Z 0001 m        gripper      GSPS01 A24 Z 0001   visuotactile
  └─┬──┘ └┬┘ │ └┬─┘ │                                          (no patch suffix)
 product batch│  seq patch    product : TCGU01 gripper / GSPS01 sensor
              line            line    : Z = R&D/test, A = production
                              seq     : last digit odd → Left, even → Right
                              patch   : m = Master (leader), s = Slave (follower)
```

`scan_grippers()` parses this for every gripper; each `GripperEndpoints`
carries `.side` (`Side.Left/Right`) and `.role` (`Role.Leader/Follower/
Unknown`). Pick a unit by side **or** role:

```python
from xense.taccap import find_left, find_right, find_leader, find_follower, parse_serial

eps = find_leader()              # the gripper whose SN patch suffix is 'm'
p   = parse_serial("TCGU01A24Z0001m")
print(p.side, p.role, p.valid)   # Side.Left  Role.Leader  True
```

`parse_serial()` degrades gracefully: a legacy (`SN000002`) or empty SN
still yields a best-effort `side` (last digit) with `role = Role.Unknown`
and `valid = False`.

### Encoder zero calibration

```python
# Hold the gripper at the desired zero pose (usually fully closed) first.
g_right.encoder.set_zero()                      # throws ProtocolError on NACK
s = g_right.encoder.read_once()
print(s.position_rad, s.raw_position_rad)       # cooked (clamped >= 0) vs raw
```

See `python/examples/calibrate.py` for the full interactive walkthrough
(side selection by SN, pre/post drift display, full-open angle sanity
check, live readout).

### Normalized leader position (0 = closed, 1 = open)

`normalize_position=True` reads the firmware's encoder-max calibration
(`Cmd::EncoderMaxCal`, firmware ≥ V2.1) at open() time and installs the
converter on the encoder, so every sample carries `.position` in `[0,1]`:

```python
g = LeaderGripper(mcu_device=dev, normalize_position=True)

s = g.encoder.read_once()
s.position_rad      # 0.65  — always radians, meaning never changes
s.position          # 0.50  — normalized; float('nan') when the flag is off

g.position()        # 0.50  — one-shot read + convert
g.pos_to_rad(1.0)   # 1.30  — full open, in raw radians
g.rad_to_pos(0.325) # 0.25

# Streamed samples are normalized too, including subscribers registered
# before the map existed.
g.encoder.on_data(lambda s: print(s.position))
g.start_streaming(imu_hz=0, encoder_hz=100)
```

The travel span is the encoder shaft angle at full open, measured from the
encoder zero (fully closed) — so **zero the encoder first**, then store the
span. `measure-encoder-max` walks both steps:

```bash
python python/examples/fisheye_cal.py measure-encoder-max
```

Construction raises `ProtocolError` when the span has never been calibrated
(the firmware answers `CalNotSet` rather than returning a bogus zero) or when
the firmware predates V2.1. Pass `encoder_max_rad=<rad>` to supply the span
from the host and skip the firmware read entirely.

`position()` / `pos_to_rad()` / `rad_to_pos()` / `position_map` work without
the flag — it only controls whether `EncoderSample.position` gets filled in.
Call `reload_position_map()` after re-calibrating.

### Fisheye camera calibration

Fisheye intrinsics + distortion live in MCU flash and are readable from both
leader and follower:

```python
from xense.taccap import CameraFisheyeCal, FisheyeUndistorter

cal = g.calibration.read_fisheye()     # None when never calibrated
if cal is not None:
    # Prefer FisheyeUndistorter over calling cv2.fisheye.undistortImage with
    # cal.K / cal.D by hand: it builds the remap tables once, resamples with
    # INTER_CUBIC and applies the same focal-length balance as the PC
    # calibration tool, so a frame rectified here matches one rectified there.
    # `cal.K` / `cal.D` remain exposed for code that must do its own thing.
    undistorter = FisheyeUndistorter(cal, width=640, height=480, balance=0.0)
    undistorted = undistorter.apply(img)

g.calibration.write_fisheye(CameraFisheyeCal(
    fx=320.5, fy=321.0, cx=319.5, cy=240.2,
    k1=-0.031, k2=0.0072, k3=-0.0013, k4=0.0002))
```

The firmware stores the values verbatim — no unit conversion, no clamping,
only NaN/Inf rejection. `python/examples/fisheye_cal.py show` prints both
records; `set-fisheye --from-npz` loads `K`/`D` straight from an OpenCV
calibration file.

### Follower gripper control (MIT force-position)

The follower drives a FDCAN motor. Control is the **MIT impedance frame** — the
force-position hybrid primitive: the `kp`/`kd` terms track a target position,
the feed-forward torque adds a force component. The SDK exposes it three ways.

```python
import xense.taccap as t
g = t.FollowerGripper.open()
g.motor.clear_fault()
g.motor.enable()                 # required before anything moves

# Raw motor control (rad). set_* block on an ACK; submit_* are no-ACK
# (fire-and-forget) for a host realtime loop. See the note below before
# picking a submission rate — 500 Hz is not a budget to spend.
g.motor.set_impedance(target_pos_rad=-0.5, kp_nm_per_rad=8, kd_nm_s_per_rad=1,
                      feedforward_torque_nm=0.0)
st = g.motor.read_status()       # actual_pos/vel/torque, target_*, control_mode
```

**Normalized position** — work in `[0, 1]` (0 = closed, 1 = open) instead of raw
radians. Requires a calibrated gripper (`GripperConfig` Valid); throws otherwise.
Note this is distinct from `g.motor.set_position()` (raw rad).

```python
print(g.position())                       # -> 0.97   (nearly open)
g.set_position(0.5, kp_nm_per_rad=8, kd_nm_s_per_rad=1)   # 50% open (no-ACK, realtime)
g.pos_to_rad(0.5), g.rad_to_pos(-0.59)    # explicit conversions
```

**`ControlLoop`** — the recommended way to drive a follower. A C++ background
thread submits the latest normalized target **in phase with the motor-status
stream** while that stream keeps a thread-safe observation fresh. Your policy
only touches `set_target(0..1)` and `observation()`, both non-blocking (no GIL
fights, no status polling).

```python
loop = t.ControlLoop(g, kp=8, kd=1)        # SubmitPhase.STREAM_LOCKED by default
loop.start()                              # seeds target = current pos (no jump)
try:
    while running:
        obs = loop.observation()          # .position [0,1], .velocity, .torque, .age_ms
        loop.set_target(policy(obs))      # your action, 0..1
finally:
    loop.stop()
g.motor.disable()
```

> **Why the phase matters, and why 500 Hz is not a budget.** The firmware
> applies the latest target at 500 Hz, but that says nothing about what
> submitting at that rate costs. Every host→MCU frame that lands while the MCU
> is transmitting makes it drop bytes out of the frame it is sending, and that
> frame is discarded whole. A 41-byte status frame at 3 Mbps fills only ~137 µs
> of each 10 ms period, so whether a submit collides depends on **when** it
> lands, not how many you send: 250 Hz lost 154 status frames on one 60 s run
> and none on the next.
>
> `SubmitPhase.STREAM_LOCKED` removes the collision instead of making it rarer —
> one submit per received status frame, landing in the ~9.86 ms the MCU is known
> to be idle. Measured with every camera on both grippers streaming and the
> motor cycling: **6000 submits : 6000 frames : 0 missing**, four runs, both
> units. Free-running at 100 Hz on the same bench lost 156–308 frames per run.
>
> It does not protect ACK responses — the loop knows when the MCU emits
> telemetry, not when it is answering somebody's command. Those survive because
> commands retry, at ~31 ms of latency each.
>
> **Feedback rate.** The motor's `actual_*` telemetry refreshes at ~50–100 Hz.
> Read observations from the **stream**, not by polling `read_status()` —
> polling `GetMotorStatus` above ~100 Hz can stall the firmware's refresh.

**LEDs and power-on auto-calibration (V1.9):**

```python
g.led.set(t.Ws2812Mode.Override, 0, 255, 0, brightness=120)   # solid green
g.led.effect(t.Ws2812EffectType.ColorBreathe, 0, 0, 255)      # blue breathe
g.led.off()

cfg = g.get_auto_cal_config()             # if enabled, the firmware self-zeros
g.set_auto_cal_config(cfg)                # (close-to-stall) + captures max_open
                                          # (open-to-stall) on power-up
```

Runnable demos: `python/examples/gripper_control_test.py` (interactive
open/close via both `set_position` and `ControlLoop`) and
`python/examples/motor_mit_control.py` (raw `submit_impedance` + health).

## Diagnostics

`g.diagnostics` wraps the firmware's own UART counters (firmware 1.1.3+) and a
runtime log switch (1.1.4+). Both work on either gripper role.

```python
s = g.diagnostics.uart_stats()
# tx_calls_ok / tx_bytes_ok  — what the firmware's transmit call accepted,
#                              i.e. what actually reached the MCU's TX register
# tx_fail_timeout            — the firmware truncated frames itself
# rx_overflow                — the firmware's command task fell behind the host
# debug_tx_bytes             — bytes out the DEBUG UART; quantifies logging cost
```

The point of `tx_calls_ok` is attribution. Compare it against what the host
decoded over the same window: if the counts agree but bytes are missing, the
loss happened **after** the bytes left the MCU (cable or USB-serial bridge) and
no firmware change reaches it.

```python
from xense.taccap import LogLevel, LOG_OUTPUT_UART
g.diagnostics.set_log_config(LogLevel.DEBUG, LOG_OUTPUT_UART)   # on
g.diagnostics.disable_logging()                                  # off again
```

> **Logging is a diagnostic lever, not a setting.** Firmware 1.1.4 ships with it
> off because the log sink is a blocking polled UART write (~0.5 ms per line at
> 921600) that stalls whichever task emitted the line — logging on every received
> command is what livelocked the command channel. Output also goes to the MCU's
> DEBUG UART, which is **not routed over USB**: without a probe on that pin you
> pay the realtime cost and see nothing.

## Examples

All scripts live under `python/examples/`; the table is in
**[docs/EXAMPLES.md](docs/EXAMPLES.md)**. Enable C++ examples with
`-DTACCAP_BUILD_EXAMPLES=ON` (off by default).

## Logging

The SDK uses **one singleton logger** named `"xense.taccap"` —
registered with spdlog, shared by every C++ TU and the Python
binding. Don't construct your own `std::make_shared<spdlog::logger>`
elsewhere, and don't reach for `std::cout` / `print` / `printf`
for diagnostic output — they bypass the file sink.

- **C++**: `#include <taccap/log.hpp>`, then `xense::taccap::logger()->info(...)`.
- **Python**: `from xense.taccap import log; log.info(...)` /
  `log.set_level("debug")` / `log.set_pattern(...)`. `set_level` and
  `set_pattern` affect the **console sink only** — the file sink keeps
  its archive format for grep stability.

Two sinks attached by default:

| Sink               | Level                             | Pattern                               |
| ------------------ | --------------------------------- | ------------------------------------- |
| stderr (colour)    | user-controllable, default `INFO` | `[%D %T.%e] [%n] [%^%l%$] %v`         |
| file (per-session) | always `DEBUG`                    | `[%Y-%m-%d %H:%M:%S.%e] [%n] [%l] %v` |

File-sink behaviour:

- Directory: `$TACCAP_LOG_DIR` if set, else `~/.taccaplogs/`.
- Filename: `session_YYYYMMDD_HHMMSS.log` — one new file per process start.
- At most **10** session logs retained; oldest mtime pruned at startup.
- File-sink creation failures (disk full / permission denied) are not
  fatal — the console sink keeps working.

## Layout

```
taccap-gripper/
├── cpp/
│   ├── include/taccap/        # Public C++ headers
│   ├── src/                   # SDK implementation (protocol, bus, components, ...)
│   ├── examples/              # C++ example programs (leader_demo)
│   └── tests/                 # gtest unit tests
├── python/
│   ├── bindings/              # pybind11 module sources
│   ├── examples/              # Python examples
│   └── xense/taccap/          # Python package (PEP 420 namespace under `xense`)
├── third_party/
│   └── firmware/              # Clone-on-demand reference repos (gitignored)
│       ├── tc-gu-01/          #   STM32 firmware that runs on the gripper
│       └── tc-gu-01-pc/       #   PyQt debug GUI (operator-side)
├── docs/                      # Architecture & API docs
├── environment.yml            # mamba env (Python 3.12, conda-forge only)
├── pyproject.toml             # scikit-build-core wheel config
└── CMakeLists.txt             # Top-level build orchestrator
```


## Documentation

- **[docs/USAGE.md](docs/USAGE.md)** — 使用文档(中文):把触觉(OG)、视觉
  (腕相机)、夹爪读数与控制三路数据分别开起来的端到端步骤。
- **[docs/INSTALL.md](docs/INSTALL.md)** — prerequisites, C++-only builds,
  device permissions, rebuild/clean, environment traps.
- **[docs/CALIBRATION.md](docs/CALIBRATION.md)** — encoder zero + travel span,
  the flash-persisted records, drift handling.
- **[docs/FIRMWARE.md](docs/FIRMWARE.md)** — reference repos, building the
  firmware, and flashing over OTA. **Read the power-cycle note before you
  measure anything after a flash.**
- **[docs/EXAMPLES.md](docs/EXAMPLES.md)** — what each example script does.
- **[docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)** — layered stack, module map,
  threading model, and the boundary between this SDK and downstream consumers.

## License

Apache-2.0. Copyright (c) 2026 XenseRobotics Co., Ltd.
