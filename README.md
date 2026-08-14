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

This repository is the foundation for two adapter repos that will follow:

- `taccap-gripper-ros2` — ROS2 (Humble + Jazzy) hardware interface package
- a fork of `lerobot-xense` (`feature/v5.1_dev`) with a `taccap_gripper`
  robot class

Both adapters consume this SDK; they do not reimplement device access.

## Status

**v0.1.7 — firmware V2.2 sync.** Tested on bilateral leader setups (left +
right, ~280 MB/s outbound, both flashed to leader 1.2.1) and on a real follower
gripper (MIT force-position control, normalized grasp, LED, auto-cal) against
V2.1 firmware. The V2.2 additions below are protocol-complete and unit-tested
but **not yet hardware-validated**.

> **Firmware you need.** Command set V2.1 needs **leader >= 1.2.0** /
> **follower >= 1.1.0**; the V2.2 follower diagnostics need **follower >=
> 1.1.2**. Those are minimums, not exact matches. The images shipped in
> `firmware/` are leader 1.2.1 (`6b4605a`) and follower 1.1.2 (`bf0a06e`) —
> the two roles no longer share a source commit, since every V2.2 command is
> follower-only. Check with `python python/examples/fisheye_cal.py show`, which
> prints the version and whether each V2.0/V2.1 command answers.
>
> The shipped follower image is a **local build, not yet hardware-validated** —
> see [`firmware/README.md`](firmware/README.md) before flashing it.
>
> **V2.2 is purely additive.** `Cmd::GetMotorStatus` (0x50) and the motor-status
> DATA stream still carry the same 31-byte payload, so `read_status()`,
> `on_status()` and `ControlLoop` behave identically on 1.1.1 and 1.1.2. Only
> the new 0x3A/0x3B/0x52/0x53 commands need the newer firmware, and they fail
> loudly with `ProtocolError(InvalidCmd)`. Payload length is *not* a version
> probe — a 31-byte status frame says nothing about firmware age.
>
> The SDK stays usable on older firmware — everything up to command set V1.9
> behaves identically. Only the V2.0/V2.1 calibration commands are affected,
> and they fail loudly with `ProtocolError(InvalidCmd)` rather than silently
> misbehaving. `LeaderGripper(..., encoder_max_rad=<rad>)` supplies the travel
> span from the host when the firmware cannot store it.
>
> To build and flash: see [Firmware / PC GUI reference repos](#firmware--pc-gui-reference-repos).

What's in:

- TC-GU-01 protocol: **wire framing V1.8** (global byte stuffing) +
  **command set V2.2**. Async transport with ACK matching, per-cmd DATA
  subscribers.
- **Follower motor control (MIT), validated on hardware.** `Motor` enable /
  disable / clear-fault + four control modes (position / velocity / torque /
  impedance). The MIT impedance frame *is* the force-position hybrid primitive
  (kp/kd track position, feed-forward torque adds force). Two send paths:
  blocking-ACK `set_*` and no-ACK `submit_*` for a host realtime loop up to the
  firmware's 500 Hz control rate.
- **Normalized gripper position** (`FollowerGripper.position()` /
  `set_position(pos, kp, kd)`, 0 = closed, 1 = open) via `GripperPosition`,
  plus **`ControlLoop`** — a C++ fixed-rate send/receive loop for embodied
  control (`set_target(0..1)` + `observation()`, both non-blocking; obs from the
  motor-status stream, not polling).
- **Normalized leader position** — `LeaderGripper(..., normalize_position=True)`
  fills `EncoderSample.position` with the opening in `[0,1]` (0 = closed,
  1 = open) on one-shot reads *and* on every streamed sample, using the
  firmware's encoder-max calibration. `position_rad` keeps reporting radians.
  Same `position()` / `pos_to_rad()` / `rad_to_pos()` surface as the follower.
- **V1.9 additions:** `motor_status_t` is 31 bytes; power-on auto-calibration
  config (`get/set_auto_cal_config`); WS2812 `Led` control (`g.led`); private-
  protocol single-parameter access (`get/set_private_param`, private-mode only).
- **V2.2 additions (follower only, firmware >= 1.1.2):** motor diagnostics —
  `motor.read_status_ext()` returns the 72-byte `MotorStatusExt` (the 31-byte
  status plus the motor fault word, its power-on latched OR, a stop-time
  snapshot and the raw CAN evidence); `motor.fault_report(force=False)` returns
  the 64-byte `MotorFaultReport` that also folds in the MCU's own firmware-level
  faults. Plus `motor.get/set_startup_limit_torque()` (the boot-time `0x700B`
  limit torque, replacing the old hard-coded 6 Nm) and
  `follower.set_auto_cal_stall_param()` for patching auto-cal stall settings
  without a read-modify-write.
- **V2.0/V2.1 additions:** `Calibration` component (`g.calibration`) for the
  two flash-persisted calibration records — fisheye camera intrinsics +
  distortion (leader *and* follower) and the leader's encoder max travel angle.
  Never-calibrated records read back as `None` (firmware `CalNotSet`), not
  zeros.
- MCU sensor components: IMU @ 100 Hz, encoder @ 100 Hz, motor status; opt-in
  wrist UVC (@ 30 Hz) `Camera` (off by default — owned by an external service).
- `LeaderGripper` / `FollowerGripper` aggregates, zero-config MCU discovery
  (`scan_grippers` / `find_left` / `find_right` / `find_leader` /
  `find_follower`). Side **and** role come from the firmware-burned SN only
  (`Cmd::GetSn` / `parse_serial()`, e.g. `TCGU01A24Z0001m`), never the CH343
  chip SN; `Side.Unknown` when neither firmware source answers.
- Python bindings on 3.10 + 3.12 (system py3.10 for ROS 2 Humble, conda py3.12
  for primary dev); single-instance spdlog logger shared with C++
  (`~/.taccaplogs/`); OTA via `OtaSession`; encoder zero calibration.

Visuotactile (OG) capture now lives at the Python level via the `xensesdk`
wheel — `xense.taccap` is the gripper-protocol + wrist-camera surface only.

Full per-commit changelog in [CHANGELOG.md](CHANGELOG.md).

---

## Install

The SDK has two consumable surfaces — the C++ shared library
(`libtaccap_core.so`) and the Python extension (`xense.taccap`). Both are
produced by the **same** top-level CMake project; you choose which surface
to build.

### 1. Prerequisites

|                       | Required                                                                                            |
| --------------------- | --------------------------------------------------------------------------------------------------- |
| OS                    | Linux (Ubuntu 22.04+ tested). The capture path is V4L2 + UVC XU; macOS / Windows are not supported. |
| Toolchain             | gcc/g++ ≥ 13, CMake ≥ 3.20, Ninja, pkg-config                                                       |
| Python (for bindings) | CPython 3.12                                                                                        |
| Recommended           | `mamba` / `conda` — `environment.yml` pins the entire toolchain & C++ deps to a known-good set      |

> **Why mamba is recommended.** `environment.yml` ships gcc-14, OpenCV
> 4.12, spdlog, gtest, pybind11 and scikit-build-core at a known-good set
> of versions. If you build against system packages instead, you are on
> your own for ABI compatibility.

### 2. Clone

```bash
git clone <repo-url> taccap-gripper
cd taccap-gripper
```

There are no git submodules — the SDK builds standalone.

### 3. Create the development environment

```bash
mamba env create -f environment.yml
mamba activate taccap

# Or, if you already have a conda env you want to add this to:
mamba env update -f environment.yml -n <your-env>
```

This installs gcc-14, the C++ deps, Python 3.12, pybind11,
scikit-build-core, numpy, pyserial, opencv-python==4.12.0.88 and
rerun-sdk in one shot. After activation you should see:

```bash
which cmake     # → .../envs/taccap/bin/cmake
which python    # → .../envs/taccap/bin/python
gcc --version   # → 14.x
```

### 4. Device permissions (one-time)

Plugged-in TacCap devices appear as `/dev/ttyACM*` (MCU) and
`/dev/video*` (UVC cameras). Your user needs to be in the matching
groups:

```bash
sudo usermod -aG dialout,video "$USER"
# log out and back in (or `newgrp dialout && newgrp video`) for it to apply
```

### 5a. Python install (recommended for most users)

`pyproject.toml` uses **scikit-build-core** as the build backend, which
drives CMake under the hood with `TACCAP_BUILD_PYTHON=ON` and
`TACCAP_BUILD_EXAMPLES=OFF`. A single `pip` invocation builds the C++
core and the pybind11 extension, then co-locates them inside the wheel
under `xense/taccap/`:

```bash
# Editable / development install (re-runs CMake on every `pip install -e .`):
pip install -e .

# Or a regular install (builds a wheel, installs it):
pip install .
```

What ends up where (editable build):

```
python/xense/taccap/
├── _taccap_native.cpython-312-x86_64-linux-gnu.so   # pybind11 module
└── libtaccap_core.so.0.1.7   (+ .so.0 symlink)      # SDK core
```

These two are co-located on purpose — the rpath is set to `$ORIGIN`,
so loading `xense.taccap` just works without `LD_LIBRARY_PATH`.

Build artefacts for editable installs land under `build/{wheel_tag}/`
(see `[tool.scikit-build] build-dir` in `pyproject.toml`). Delete that
directory if you want a clean rebuild; `pip install -e .` will regenerate it.

### 5b. C++-only build (no Python)

If you don't need the Python bindings — e.g. you are integrating
`libtaccap_core.so` into a ROS 2 package or another CMake project —
build directly with CMake/Ninja:

```bash
cmake -B build -G Ninja \
    -DCMAKE_BUILD_TYPE=Release \
    -DTACCAP_BUILD_PYTHON=OFF \
    -DTACCAP_BUILD_EXAMPLES=ON \
    -DTACCAP_BUILD_TESTS=ON

cmake --build build -j
```

Output:

```
build/
├── cpp/libtaccap_core.so(.0)(.0.1.7)
├── cpp/examples/leader_demo
└── cpp/tests/...                # gtest binaries; run via `ctest`
```

CMake options (top-level `CMakeLists.txt:19-21`):

| Option                  | Default | Effect                                          |
| ----------------------- | ------- | ----------------------------------------------- |
| `TACCAP_BUILD_PYTHON`   | `ON`    | Build the `_taccap_native` pybind11 module      |
| `TACCAP_BUILD_EXAMPLES` | `OFF`   | Build the `leader_demo` smoke binary            |
| `TACCAP_BUILD_TESTS`    | `OFF`   | Build the gtest suite under `cpp/tests/`        |

### 6. Verify

```bash
# Python — note `env -u PYTHONPATH`, see the note below
env -u PYTHONPATH python -c "import xense.taccap as t; print(t.hello()); print(t.__version__)"
# → taccap-gripper OK; version 0.1.7
# → 0.1.7

# Python tests (hardware-free cases always run; IMU cases skip without a gripper)
env -u PYTHONPATH pytest python/tests

# C++ tests (only if TACCAP_BUILD_TESTS=ON)
ctest --test-dir build --output-on-failure
```

> **If `xense.taccap` resolves somewhere unexpected, check `PYTHONPATH`.**
> Stacked conda activations can export another env's `site-packages` into
> *every* interpreter, which then shadows this repo with whatever editable
> install lives there — you end up testing a different checkout without any
> error. `env -u PYTHONPATH python -c "import xense.taccap as t; print(t.__file__)"`
> tells you which tree you are actually running.

### 7. Rebuild / clean

```bash
# Python: blow away scikit-build-core's build dir
rm -rf build/ && pip install -e .

# Pure C++: incremental rebuild is fine
cmake --build build -j

# Full reset
rm -rf build/
```

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
cal = g.calibration.read_fisheye()     # None when never calibrated
if cal is not None:
    undistorted = cv2.fisheye.undistortImage(img, cal.K, cal.D)

from xense.taccap import CameraFisheyeCal
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

# Raw motor control (rad). set_* block on an ACK; submit_* are no-ACK (fire-
# and-forget) for a host realtime loop up to the firmware's 500 Hz rate.
g.motor.set_impedance(target_pos_rad=-0.5, kp_nm_per_rad=8, kd_nm_s_per_rad=1,
                      feedforward_torque_nm=0.0)
st = g.motor.read_status()       # actual_pos/vel/torque, target_*, control_mode
```

**Normalized position** — work in `[0, 1]` (0 = closed, 1 = open) instead of raw
radians. Requires a calibrated gripper (`GripperConfig` Valid); throws otherwise.
Note this is distinct from `g.motor.set_position()` (raw rad).

```python
print(g.position())                       # -> 0.97   (nearly open)
g.set_position(0.5, kp=8, kd=1)           # go to 50% open (no-ACK, realtime)
g.pos_to_rad(0.5), g.rad_to_pos(-0.59)    # explicit conversions
```

**`ControlLoop`** — a C++ background thread submits the latest normalized target
at a fixed rate while the motor-status stream keeps a thread-safe observation
fresh. Ideal for embodied policies: your loop only touches `set_target(0..1)`
and `observation()`, both non-blocking (no GIL fights, no status polling).

```python
loop = t.ControlLoop(g, hz=200, kp=8, kd=1)
loop.start()                              # seeds target = current pos (no jump)
try:
    while running:
        obs = loop.observation()          # .position [0,1], .velocity, .torque, .age_ms
        loop.set_target(policy(obs))      # your action, 0..1
finally:
    loop.stop()
g.motor.disable()
```

> **Feedback rate.** The motor's `actual_*` telemetry refreshes at ~50–100 Hz
> (firmware reads it back over CAN periodically). Read observations from the
> **stream** (`ControlLoop` / `motor.on_status`), not by polling
> `read_status()` — polling `GetMotorStatus` above ~100 Hz can stall the
> firmware's refresh.

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

## Examples

All scripts live under `python/examples/`. Enable C++ examples with
`-DTACCAP_BUILD_EXAMPLES=ON` (they're off by default).

| Script                           | What it does                                                                                                                                                                                                                                                                                                                                                                                |
| -------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `rerun_dual_with_tracker.py`     | Dual-gripper IMU/encoder + Pico4 motion-tracker 6-DoF poses in one viewer. Requires [`xensevr_pc_service_sdk`](https://github.com/Vertax42/Xense-Pico-Teleop-Interface) and the XenseVR PC Service running. Use `--left-tracker-sn` / `--right-tracker-sn` to map tracker SNs to sides. (Cameras are owned by the external camera service and not shown here.)                                  |
| `calibrate.py`                   | Per-gripper encoder calibration CLI, selected by `left` / `right` (or an explicit SN) — latches the zero **and stores the measured travel span** (`Cmd::EncoderMaxCal`), which is what unlocks normalized position. Shows raw + cooked side-by-side, then a live `raw \| cooked \| position 0..1` readout. Checks firmware support before writing anything. See [Calibration](#calibration).                                                          |
| `gripper_control_test.py`        | Interactive follower open/close test — steps through positions via both one-shot `set_position(0..1)` and the realtime `ControlLoop`, reading position back. See [Follower gripper control](#follower-gripper-control-mit-force-position).                                                                                                                                                    |
| `motor_mit_control.py`           | Primitive demo of the raw MIT submission API (`submit_impedance`) with the out-of-band health channel (`control_stats` / `read_status`).                                                                                                                                                                                                                                                    |
| `fisheye_cal.py`                 | Read/write the flash-persisted calibration records (V2.0/V2.1): `show`, `set-fisheye` (flags or an OpenCV `.npz` holding `K`/`D`), `set-encoder-max`, and `measure-encoder-max` — the guided close-zero → open-sample → store flow that unlocks normalized leader position.                                                                                                                     |
| `leader_normalized_position.py`  | Streams a leader gripper's opening as `0..1` via `normalize_position=True`, with a live bar. Needs the encoder-max record (or `--encoder-max-rad` to bypass the firmware read).                                                                                                                                                                                                              |
| `ota_update.py`                  | Firmware OTA flashing CLI with progress + post-flash status probe. **Risky — wrong artefact bricks the MCU.**                                                                                                                                                                                                                                                                               |
| `v4l2_probe.py`, `v4l2_sweep.py` | Manual V4L2 bringup probes for the wrist / OG cameras (discovery is MCU-only and no longer enumerates them). Also handy when a firmware SN isn't burned yet.                                                                                                                                                                                                                                  |
| `leader_demo` (C++)              | Reports streaming rates for a single leader gripper over 5 seconds.                                                                                                                                                                                                                                                                                                                         |

### Bench-specific tracker ↔ gripper binding (this checkout)

`rerun_dual_with_tracker.py` needs explicit `--left-tracker-sn` /
`--right-tracker-sn` because the Pico4 trackers are physically glued to a
specific gripper — software can't re-derive which is which. We maintain
two bilateral pairs on **this bench**; figure out which one is plugged in
(`scan_grippers` reports the firmware SNs) and use the matching row.

> **Note — legacy SNs.** The firmware-SN column below predates the TacCap
> SN scheme (`TCGU01A24…`); these units still report the old `SN0000NN`
> strings until they're re-burned. The **CH343 SN** column is the stable
> key that never changes, so match on that. Once re-burned, `.role`
> (leader/follower) becomes available via the new SN too.

**Pair A** — verified 2026-05-27 by shaking each gripper and watching the
matching ellipsoid move in the rerun 3D view:

| Side  | Gripper firmware SN | Gripper CH343 SN | Pico4 tracker SN    |
| ----- | ------------------- | ---------------- | ------------------- |
| LEFT  | `SN000001`          | `5C2C247734`     | `PC2310MLK7080553G` |
| RIGHT | `SN000002`          | `5C2C247736`     | `PC2310MLL1091974G` |

**Pair B** — verified 2026-05-29 by the same shake-test:

| Side  | Gripper firmware SN | Gripper CH343 SN | Pico4 tracker SN    |
| ----- | ------------------- | ---------------- | ------------------- |
| LEFT  | `SN000003`          | `5C2C246526`     | `PC2310MLL3200579G` |
| RIGHT | `SN000004`          | `5C2C246523`     | `PC2310MLL3200496G` |

Canonical invocations:


```bash
# Pair A
python python/examples/rerun_dual_with_tracker.py --left-tracker-sn  PC2310MLK7080553G --right-tracker-sn PC2310MLL1091974G

# Pair B
python python/examples/rerun_dual_with_tracker.py \
    --left-tracker-sn  PC2310MLL3200579G \
    --right-tracker-sn PC2310MLL3200496G
```

> **Heads-up for forks / other benches.** These SNs identify _our_
> hardware, not yours. If you clone this repo onto a different setup,
> replace them with whatever `xensevr_pc_service_sdk` reports for your
> trackers, then re-verify by shaking one gripper at a time. Also note:
> the C SDK's "device found" log line may list a third SN — that's the
> Pico headset itself, not a tracker.

## Calibration

Mechanical slop and small post-zero drift can make the encoder report
~0.05–0.10 rad when the gripper is held "fully closed". The SDK
absorbs this two ways:

- **Auto-clamp**: `Encoder::read_once()` and `on_data` callbacks return
  `position_rad >= 0`. Negative raw drift becomes `0.0` to keep
  downstream consumers' math sane. The unclamped value is preserved
  in `raw.position_rad` (C++) / `raw_position_rad` (Python) for
  diagnostics.
- **Drift warning**: if the raw negative drift exceeds **-0.1 rad** the
  logger emits a rate-limited warning (1 / s per `Encoder` instance)
  pointing at calibration or mechanical issues.

To calibrate a gripper, run `calibrate.py` against the side you want to fix
(or its SN, if you'd rather be explicit):

```bash
python python/examples/calibrate.py left               # by side
python python/examples/calibrate.py TCGU01A28Z0023m    # by firmware SN
```

The script:

1. Resolves `left`/`right` (or the SN you passed) to one `mcu_device`, and
   prints the firmware SN it picked plus every gripper it can see, so the
   pick is verifiable. Side comes from the firmware-burned SN read over the
   wire (`Cmd::GetSn`), not the CH343 chip serial.
2. **Pre-flight:** checks the firmware implements `Cmd::EncoderMaxCal`
   (0x2C). This runs *before* anything is written — step 4 persists a new
   zero, so a pre-V2.1 gripper is refused while still untouched rather than
   left half-calibrated with a new zero and no span.
3. Prints the current encoder reading (both `raw` and clamped) so the
   existing drift is visible.
4. Prompts "hold the gripper **FULLY CLOSED**, press [Enter]", sends
   `Cmd::SetEncoderZero`, re-reads, and validates the new raw reading is
   within ± 0.01 rad.
5. Prompts "open to the **MECHANICAL LIMIT**, press [Enter]" and **stores**
   the measured angle as the travel span (`Cmd::EncoderMaxCal`). That span
   is what `normalize_position=True` divides by.
6. Live 10 Hz readout (`raw | cooked | position 0..1`) until Ctrl+C.

There is deliberately **no expected full-open angle** to check against. The
measured span *is* the calibration — it is whatever the mechanism does, and
it is what the SDK normalizes by. (An earlier 1.7 rad "design baseline" was
stale and fired false alarms on healthy hardware: three measurements across
two units gave 1.1582 / 1.1589 / 1.1486 rad, i.e. ~66°.) The only checks
left are the ones that catch a genuinely broken measurement — a non-positive
span, a zero that did not take, a write that did not stick.

`--skip-open-probe` latches only the zero; normalized position then stays
unavailable until the span is measured.

The firmware latches whatever raw count it sees the moment it
processes the command, so the gripper must already be at the target
pose before pressing Enter.

### Flash-persisted calibration records (V2.0 / V2.1)

Two more records live in MCU flash behind `g.calibration`. Note the encoder
zero above is **also** flash-persisted — `cmd_handler_set_encoder_zero` calls
`storage_write_encoder_calibration()`, so none of these need a power cycle
and none of them are lost on reboot:

| Record | Command | Scope | Accessor |
| --- | --- | --- | --- |
| Fisheye camera `fx, fy, cx, cy, k1..k4` | `0x2B` | leader + follower | `read_fisheye()` / `write_fisheye()` |
| Encoder max travel angle (rad) | `0x2C` | **leader only** | `read_encoder_max_rad()` / `write_encoder_max_rad()` |

Both survive power cycles. A record that was never written reads back as
`None` — the firmware answers `ErrorCode.CalNotSet` instead of returning
zeros, so "never calibrated" is distinguishable from "calibrated to exactly
0". Every other firmware error still raises `ProtocolError`; on a follower or
on pre-V2.1 firmware the encoder-max methods raise with `InvalidCmd`.

```bash
python python/examples/fisheye_cal.py show                  # print both records
python python/examples/fisheye_cal.py measure-encoder-max   # guided: zero, open, store
```

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

## Firmware / PC GUI reference repos

The wire protocol this SDK speaks is defined by the firmware that runs
on the gripper's STM32H562 MCU. The protocol PDF + Python prototype
(in PyQt) live in two **internal** repos that we **read but don't
ship** — they have separate release cadences and build toolchains and
shouldn't be linked into this SDK's git history. Ask the firmware team
for access if you are working on the wire format.

If you have them, clone into `third_party/firmware/tc-gu-01` and
`third_party/firmware/tc-gu-01-pc`: both paths are in `.gitignore`, so
they sit next to the SDK for easy `grep` / IDE discovery but never
appear in `git status`.

You do **not** need either to flash a gripper — the released images ship
in [`firmware/`](firmware/).

What's in them:

- `tc-gu-01/App/protocol/protocol_cmd.h` + `protocol_data.h` — canonical
  command enum + POD payload layouts. The SDK's
  `cpp/include/taccap/protocol/{commands.hpp,payloads.hpp}` mirror these
  1:1 with `static_assert(sizeof(...) == ...)` size checks. Currently
  mirrored from branch `hw_v1.1.0` @ `bf0a06e` (command set V2.2).
- `tc-gu-01/App/protocol/PROTOCOL_SPEC.md` + `tc-gu-01/docs/PROTOCOL.md` —
  the human-readable spec, including the §10 offset table for the 72-byte
  extended motor status that `test_codec_v22.cpp` is transcribed from.
- `tc-gu-01/App/tasks/task_data_stream.c` + `task_imu.c` +
  `task_encoder.c` — explains why IMU/encoder unique-data rate caps at
  ~60 Hz even when you request 100 (see the SDK's stream-dup note in
  the Claude memory).
- `tc-gu-01-pc/core/protocol.py` + `core/serial_worker.py` — Python
  reference implementation of the same wire protocol; useful as a
  cross-check when debugging the C++ codec.

### Building the firmware (Ubuntu) and flashing it over OTA

The firmware builds with a plain Makefile — no CubeIDE needed. `GRIPPER` is
mandatory; it selects `-DENABLE_MASTER_GRIPPER` / `-DENABLE_SLAVE_GRIPPER`,
which is what splits the command table and the version constant.

```bash
sudo apt install gcc-arm-none-eabi

cd third_party/firmware/tc-gu-01
env -u CFLAGS -u CXXFLAGS -u CPPFLAGS -u LDFLAGS make GRIPPER=master -j"$(nproc)"
env -u CFLAGS -u CXXFLAGS -u CPPFLAGS -u LDFLAGS make GRIPPER=slave  -j"$(nproc)"
# -> build/master/tc-gu-01-master.bin   (leader 1.2.1)
# -> build/slave/tc-gu-01-slave.bin     (follower 1.1.2 at bf0a06e)
```

> **`env -u CFLAGS ...` is load-bearing.** The `taccap` conda env exports host
> x86 build flags (`-march=nocona -mtune=haswell -isystem <env>/include`), and
> the firmware Makefile uses `CFLAGS +=`, so they get appended to the ARM
> cross-compile and it fails with `unrecognized -march target: nocona`.
> `conda deactivate` works too.

Then flash over the wire — no SWD probe. The plain `.bin` is the OTA artifact:
the image always links at `0x08000000`, and the firmware writes it to the
inactive bank and uses the STM32H5 bank swap, so one build serves both banks.

```bash
python python/examples/ota_update.py \
    third_party/firmware/tc-gu-01/build/master/tc-gu-01-master.bin \
    --side left --target-version 1.2.1
```

> Only builds you made yourself need that path. To flash the **released**
> images, name them and let the script find them in [`firmware/`](firmware/) —
> that resolves from any working directory, including a parent repo that
> vendors this one as a submodule:
>
> ```bash
> python python/examples/ota_update.py tc-gu-01-master.bin \
>     --side left --target-version 1.2.1
> ```

Notes:

- **`make` succeeding does not mean it will flash.** The linker script declares
  the full 2048K, but OTA caps a single bank at 456 KB — check
  `ls -l build/*/tc-gu-01-*.bin` (builds at `bf0a06e` with
  `arm-none-eabi-gcc 13.2.1`: master 117,612 B, slave 156,048 B, i.e. 25% / 33%
  of the cap). Sizes vary by several hundred bytes across toolchains, so treat
  these as approximate.
- Flash the artifact matching the *role*, not the side. A gripper's role is the
  `m` / `s` suffix on its firmware SN; both leaders take the `master` build.
- `make download` is Windows-only (`STM32_Programmer_CLI.exe`, and it flashes
  the `.elf`). On Ubuntu use the OTA path above.
- `Cmd::GetVersion` returns the **compiled-in** constant, not the OTA bank
  metadata, so `--target-version` is bookkeeping only — the version you read
  back afterwards is proof of what actually got flashed.

## Documentation

- [Architecture overview](docs/ARCHITECTURE.md) — layered stack, module
  map, data-flow diagrams, threading model, USB-topology discovery, and
  the explicit boundary between this SDK and downstream consumers
  (dataset recording / ROS 2 / lerobot adapters).

## License

Apache-2.0. Copyright (c) 2026 XenseRobotics Co., Ltd.
