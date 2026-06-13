# taccap-gripper

C++17 / Python SDK for the **TacCap-Gripper** —— XenseRobotics' multimodal
tactile data-collection gripper. Exposes a single namespace
(`xense::taccap::` / `xense.taccap`) for:

- IMU + encoder readout via the TC-GU-01 serial protocol
- Motor control (follower side, FDCAN→灵足 transparently routed via MCU)
- Leader / follower gripper objects that aggregate the MCU sensors and
  expose zero-config discovery
- Standalone `Camera` (wrist UVC) and `TactileSensor` (visuotactile: V4L2 +
  on-sensor calibration → rectify, vendored from
  [libxensesdk](http://192.168.1.61/Vertax42/libxensesdk) in **lite mode** —
  no ONNX / CUDA / MIGraphX runtime dependency). These are **opt-in**: an
  external camera service owns the V4L2 devices now, so the gripper
  aggregates do **not** open them unless constructed with
  `open_cameras=True`.

This repository is the foundation for two adapter repos that will follow:

- `taccap-gripper-ros2` — ROS2 (Humble + Jazzy) hardware interface package
- a fork of `lerobot-xense` (`feature/v5.1_dev`) with a `taccap_gripper`
  robot class

Both adapters consume this SDK; they do not reimplement device access.

## Status

**v0.1.0 — first usable release.** Hardware-tested on bilateral TacCap
gripper setups (left + right simultaneously, ~280 MB/s outbound).
What's in:

- TC-GU-01 wire protocol up to firmware V1.6 (OTA, MagCal, KeyStatus,
  sensor errors), async transport with ACK matching, per-cmd DATA
  subscribers
- MCU sensor components: IMU @ 100 Hz, encoder @ 100 Hz, motor status;
  plus opt-in visuotactile (+ rectify @ 30 Hz) and wrist UVC (@ 30 Hz)
  camera classes (off by default — owned by an external camera service)
- `LeaderGripper` / `FollowerGripper` aggregates, zero-config MCU
  discovery (`scan_grippers` / `find_left` / `find_right`)
- Side detection from firmware-burned SN (one MCU board = one gripper)
- Python bindings on 3.10 + 3.12 (system py3.10 for ROS 2 Humble,
  conda py3.12 for primary dev)
- Single-instance spdlog logger shared with C++; per-session file
  log under `~/.taccaplogs/`
- Encoder zero calibration (`Encoder::set_zero`) with raw +
  auto-clamped position exposure for negative drift
- OTA firmware updates via `OtaSession`
- Six example scripts including dual-gripper + Pico4-tracker
  rerun visualisation

Full per-commit changelog in [CHANGELOG.md](CHANGELOG.md).

---

## Install

The SDK has two consumable surfaces — the C++ shared library
(`libtaccap_core.so` plus the vendored `libxensesdk.so`) and the Python
extension (`xense.taccap`). Both are produced by the **same** top-level
CMake project; you choose which surface to build.

### 1. Prerequisites

|                       | Required                                                                                            |
| --------------------- | --------------------------------------------------------------------------------------------------- |
| OS                    | Linux (Ubuntu 22.04+ tested). The capture path is V4L2 + UVC XU; macOS / Windows are not supported. |
| Toolchain             | gcc/g++ ≥ 13, CMake ≥ 3.20, Ninja, pkg-config                                                       |
| Python (for bindings) | CPython 3.12                                                                                        |
| Recommended           | `mamba` / `conda` — `environment.yml` pins the entire toolchain & C++ deps to a known-good set      |

> **Why mamba is recommended.** `environment.yml` ships gcc-14, OpenCV
> 4.12, Eigen, OpenSSL, zlib, nlohmann_json, gtest, pybind11 and
> scikit-build-core at the exact versions libxensesdk-lite is verified
> against. If you build against system packages instead, you are on your
> own for ABI compatibility.

### 2. Clone (with the libxensesdk submodule)

The vendored `third_party/libxensesdk` is a git submodule. **Without it
the top-level CMake configure aborts.**

```bash
git clone --recurse-submodules <repo-url> taccap-gripper
cd taccap-gripper

# If you forgot --recurse-submodules:
git submodule update --init --recursive
```

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
core, the libxensesdk-lite shared library, and the pybind11 extension,
then co-locates them inside the wheel under `xense/taccap/`:

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
├── libtaccap_core.so.0.0.1   (+ .so.0 symlink)      # SDK core
└── libxensesdk.so.0.0.2      (+ .so.0 symlink)      # vendored lite SDK
```

These three are co-located on purpose — the rpath is set to `$ORIGIN`,
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
├── third_party/libxensesdk/libxensesdk.so(.0)(.0.0.2)
├── cpp/libtaccap_core.so(.0)(.0.0.1)
├── cpp/examples/taccap_hello
├── cpp/examples/leader_demo
└── cpp/tests/...                # gtest binaries; run via `ctest`
```

The shared libraries' build-tree rpath points at the libxensesdk build
output, so the example binaries run in place without `LD_LIBRARY_PATH`.

CMake options (top-level `CMakeLists.txt:19-21`):

| Option                  | Default | Effect                                                |
| ----------------------- | ------- | ----------------------------------------------------- |
| `TACCAP_BUILD_PYTHON`   | `ON`    | Build the `_taccap_native` pybind11 module            |
| `TACCAP_BUILD_EXAMPLES` | `OFF`   | Build `taccap_hello` and `leader_demo` smoke binaries |
| `TACCAP_BUILD_TESTS`    | `OFF`   | Build the gtest suite under `cpp/tests/`              |

`XENSE_LITE_BUILD=ON` is forced before the submodule's CMakeLists is
loaded, so the libxense lite path is always selected — no ML deps will
be searched for or linked.

### 6. Verify

```bash
# Python
python -c "import xense.taccap as t; print(t.hello()); print(t.__version__)"
# → taccap-gripper version: 0.0.1, libxense lite version: ...
# → 0.0.1

# C++ (only if TACCAP_BUILD_EXAMPLES=ON)
./build/cpp/examples/taccap_hello
# → taccap-gripper 0.0.1 (libxense lite ...)
# → xense::taccap::Context created OK

# C++ tests (only if TACCAP_BUILD_TESTS=ON)
ctest --test-dir build --output-on-failure
```

### 7. Rebuild / clean

```bash
# Python: blow away scikit-build-core's build dir
rm -rf build/ && pip install -e .

# Pure C++: incremental rebuild is fine
cmake --build build -j

# Full reset (incl. the libxensesdk submodule build)
rm -rf build/
git submodule foreach --recursive 'git clean -xdf'
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

The wrist camera and OG tactile sensors are owned by an external camera
service, so `open()` does not touch them. To have a gripper drive them,
construct it explicitly with `open_cameras=True` and the device IDs:

```python
g = t.LeaderGripper(mcu_device, wrist_video="/dev/video2",
                    tactile_left_serial="OG000477",
                    tactile_right_serial="OG000478",
                    open_cameras=True)
g.tactile_left.start(lambda f: print("L", f.frame_index, f.rectified.shape))
g.wrist_camera.start(lambda f: print("wrist", f.frame_index))
```

Or open them on their own with the standalone `t.Camera` / `t.TactileSensor`
classes — independent of any gripper.

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

## Examples

All scripts live under `python/examples/`. Enable C++ examples with
`-DTACCAP_BUILD_EXAMPLES=ON` (they're off by default).

| Script                           | What it does                                                                                                                                                                                                                                                                                                                                                                                |
| -------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `rerun_visualize.py`             | Single-gripper rerun viewer — IMU/encoder time-series + observed FPS panel. Wrist + tactile are opt-in: pass `--wrist` / `--tactile-left` / `--tactile-right` to open them standalone (off by default).                                                                                                                                                                                       |
| `rerun_dual_with_tracker.py`     | Dual-gripper IMU/encoder + Pico4 motion-tracker 6-DoF poses in one viewer. Requires [`xensevr_pc_service_sdk`](https://github.com/Vertax42/Xense-Pico-Teleop-Interface) and the XenseVR PC Service running. Use `--left-tracker-sn` / `--right-tracker-sn` to map tracker SNs to sides. (Cameras are owned by the external camera service and not shown here.)                                  |
| `calibrate.py`                   | Per-SN encoder zero calibration CLI. Shows raw + cooked side-by-side, latches zero, sanity-checks max-open angle, then enters a live readout. See [Calibration](#calibration).                                                                                                                                                                                                              |
| `ota_update.py`                  | Firmware OTA flashing CLI with progress + post-flash status probe. **Risky — wrong artefact bricks the MCU.**                                                                                                                                                                                                                                                                               |
| `v4l2_probe.py`, `v4l2_sweep.py` | Manual V4L2 bringup probes for the wrist / OG cameras (discovery is MCU-only and no longer enumerates them). Also handy when a firmware SN isn't burned yet.                                                                                                                                                                                                                                  |
| `taccap_hello` (C++)             | Smoke test for the C++ install path — prints the SDK + libxense versions and constructs a `Context`.                                                                                                                                                                                                                                                                                        |
| `leader_demo` (C++)              | Reports streaming rates for a single leader gripper over 5 seconds.                                                                                                                                                                                                                                                                                                                         |

### Bench-specific tracker ↔ gripper binding (this checkout)

`rerun_dual_with_tracker.py` needs explicit `--left-tracker-sn` /
`--right-tracker-sn` because the Pico4 trackers are physically glued to a
specific gripper — software can't re-derive which is which. We maintain
two bilateral pairs on **this bench**; figure out which one is plugged in
(`scan_grippers` reports the firmware SNs) and use the matching row.

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

To actually re-zero the gripper, run `calibrate.py` against the SN
you want to fix:

```bash
python python/examples/calibrate.py SN000002      # right gripper
```

The script:

1. Resolves the firmware SN to the right `mcu_device`.
2. Prints the current encoder reading (both `raw` and clamped) so the
   existing drift is visible.
3. Prompts "hold the gripper **FULLY CLOSED**, press [Enter]".
4. Sends `Cmd::SetEncoderZero`, re-reads, validates that the new raw
   reading is within tolerance (default ± 0.01 rad).
5. Optional `Step 2/2` — probe the mechanical full-open angle and
   compare against the expected envelope (default 1.7 rad ≈ 97°,
   tunable with `--expected-max-open-rad`).
6. Live 10 Hz readout (`raw | cooked`) until Ctrl+C.

The firmware latches whatever raw count it sees the moment it
processes the command, so the gripper must already be at the target
pose before pressing Enter.

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
│   ├── include/taccap/        # Public C++ headers (namespace alias + new types)
│   ├── src/                   # SDK implementation (protocol, bus, components, ...)
│   ├── examples/              # C++ example programs (taccap_hello, leader_demo)
│   └── tests/                 # gtest unit tests
├── python/
│   ├── bindings/              # pybind11 module sources
│   ├── examples/              # Python examples (rerun_visualize.py)
│   └── xense/taccap/          # Python package (PEP 420 namespace under `xense`)
├── third_party/
│   ├── libxensesdk/           # Submodule, lite mode (no ML deps)
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
(in PyQt) live in two upstream repos on the company GitLab that we
**read but don't ship** — they have separate release cadences and
build toolchains and shouldn't be linked into this SDK's git history.

```
# Optional but recommended for any work that touches the wire format:
mkdir -p third_party/firmware
cd third_party/firmware
git clone -b master git@192.168.110.140:xense/tc-gu-01.git      # STM32 firmware
git clone -b master git@192.168.110.140:xense/tc-gu-01-pc.git   # PyQt GUI
```

Both paths are listed in `.gitignore` — they live next to the SDK for
easy `grep` / IDE discovery but never appear in `git status`.

What's where:

- `tc-gu-01/App/protocol/protocol_cmd.h` + `protocol_data.h` — canonical
  command enum + POD payload layouts. The SDK's
  `cpp/include/taccap/protocol/{commands.hpp,payloads.hpp}` mirror these
  1:1 with `static_assert(sizeof(...) == ...)` size checks.
- `tc-gu-01/App/tasks/task_data_stream.c` + `task_imu.c` +
  `task_encoder.c` — explains why IMU/encoder unique-data rate caps at
  ~60 Hz even when you request 100 (see the SDK's stream-dup note in
  the Claude memory).
- `tc-gu-01-pc/core/protocol.py` + `core/serial_worker.py` — Python
  reference implementation of the same wire protocol; useful as a
  cross-check when debugging the C++ codec.

## Documentation

- [Architecture overview](docs/ARCHITECTURE.md) — layered stack, module
  map, data-flow diagrams, threading model, USB-topology discovery, and
  the explicit boundary between this SDK and downstream consumers
  (dataset recording / ROS 2 / lerobot adapters).

## License

Apache-2.0. Copyright (c) 2026 XenseRobotics Co., Ltd.
