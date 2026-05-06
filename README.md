# taccap-gripper

C++17 / Python SDK for the **TacCap-Gripper** —— XenseRobotics' multimodal
tactile data-collection gripper. Exposes a single namespace
(`xense::taccap::` / `xense.taccap`) for:

- Two visuotactile cameras (V4L2 + on-sensor calibration → rectify) — vendored
  from [libxensesdk](http://192.168.1.61/Vertax42/libxensesdk) in **lite mode**
  (no ONNX / CUDA / MIGraphX runtime dependency)
- Wrist UVC camera
- IMU + encoder readout via the TC-GU-01 serial protocol
- Motor control (follower side, FDCAN→灵足 transparently routed via MCU)
- Leader / follower gripper objects that aggregate the above

This repository is the foundation for two adapter repos that will follow:

- `taccap-gripper-ros2` — ROS2 (Humble + Jazzy) hardware interface package
- a fork of `lerobot-xense` (`feature/v5.1_dev`) with a `taccap_gripper`
  robot class

Both adapters consume this SDK; they do not reimplement device access.

## Status

Bootstrap scaffold (v0.0.1) — only build/scaffold validated. Real components
land incrementally on `feat/*` branches.

---

## Install

The SDK has two consumable surfaces — the C++ shared library
(`libtaccap_core.so` plus the vendored `libxensesdk.so`) and the Python
extension (`xense.taccap`). Both are produced by the **same** top-level
CMake project; you choose which surface to build.

### 1. Prerequisites

| | Required |
|---|---|
| OS | Linux (Ubuntu 22.04+ tested). The capture path is V4L2 + UVC XU; macOS / Windows are not supported. |
| Toolchain | gcc/g++ ≥ 13, CMake ≥ 3.20, Ninja, pkg-config |
| Python (for bindings) | CPython 3.12 |
| Recommended | `mamba` / `conda` — `environment.yml` pins the entire toolchain & C++ deps to a known-good set |

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

| Option | Default | Effect |
|---|---|---|
| `TACCAP_BUILD_PYTHON`   | `ON`  | Build the `_taccap_native` pybind11 module |
| `TACCAP_BUILD_EXAMPLES` | `OFF` | Build `taccap_hello` and `leader_demo` smoke binaries |
| `TACCAP_BUILD_TESTS`    | `OFF` | Build the gtest suite under `cpp/tests/` |

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

```python
import xense.taccap as t

# Auto-discover one connected gripper (left or right) by USB topology
gripper = t.LeaderGripper.open()
gripper.start_streaming(imu_hz=100, encoder_hz=100)

gripper.tactile_left.start(lambda f:  print("L", f.frame_index, f.rectified.shape))
gripper.tactile_right.start(lambda f: print("R", f.frame_index, f.rectified.shape))

imu_sub = gripper.imu.on_data(lambda s: print(s))

# ... do work ...
gripper.stop_streaming()
```

See `python/examples/rerun_visualize.py` for a full multimodal viewer.

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
│   └── libxensesdk/           # Submodule, lite mode (no ML deps)
├── docs/                      # Architecture & API docs
├── environment.yml            # mamba env (Python 3.12, conda-forge only)
├── pyproject.toml             # scikit-build-core wheel config
└── CMakeLists.txt             # Top-level build orchestrator
```

## Documentation

- [Architecture overview](docs/ARCHITECTURE.md) — layered stack, module
  map, data-flow diagrams, threading model, USB-topology discovery, and
  the explicit boundary between this SDK and downstream consumers
  (dataset recording / ROS 2 / lerobot adapters).

## License

Apache-2.0. Copyright (c) 2026 XenseRobotics Co., Ltd.
