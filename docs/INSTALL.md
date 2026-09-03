# 安装与构建

<!-- 从 README.md 拆出，保持内容不变；README 只保留入门路径。 -->

## Install

The SDK has two consumable surfaces — the C++ shared library
(`libtaccap_core.so`) and the Python extension (`xense.taccap`). Both are
produced by the **same** top-level CMake project; you choose which surface
to build.

### 1. Prerequisites

|                       | Required                                                                                            |
| --------------------- | --------------------------------------------------------------------------------------------------- |
| OS                    | Linux (Ubuntu 22.04 / 24.04 tested). The capture path is V4L2 + UVC XU; macOS / Windows are not supported. |
| Toolchain             | gcc/g++ ≥ 13, CMake ≥ 3.20, Ninja, pkg-config                                                       |
| Python (for bindings) | CPython ≥ 3.10 (`requires-python` in `pyproject.toml`); `environment.yml` pins 3.12 as the recommended development interpreter |
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

Every command below assumes the SDK root as the working directory. If you
consume the SDK as a vendored submodule of a downstream repo instead (e.g.
`xense-taccap-lerobot` carries it at `third_party/taccap-gripper`), skip the
clone and `cd third_party/taccap-gripper` first — the steps are otherwise the
same. If all you do is collect data with such a repo you normally never need
this page: its `setup_env.sh` builds the SDK as part of that environment.

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
# Editable / development install (re-runs CMake on every `pip install -e .`).
# --no-build-isolation builds against the env's pinned pybind11 /
# scikit-build-core instead of pulling fresh copies into a temp venv:
pip install -e . --no-build-isolation

# Or a regular install (builds a wheel, installs it):
pip install .
```

What ends up where (editable build):

```
python/xense/taccap/
├── _taccap_native.cpython-312-x86_64-linux-gnu.so   # pybind11 module
└── libtaccap_core.so.<version>  (+ .so.0 symlink)   # SDK core
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
├── cpp/libtaccap_core.so(.0)(.<version>)
├── cpp/examples/leader_demo
└── cpp/tests/...                # gtest binaries; run via `ctest`
```

CMake options (top-level `CMakeLists.txt:19-21`):

| Option                  | Default | Effect                                          |
| ----------------------- | ------- | ----------------------------------------------- |
| `TACCAP_BUILD_PYTHON`   | `ON`    | Build the `_taccap_native` pybind11 module      |
| `TACCAP_BUILD_EXAMPLES` | `OFF`   | Build the `leader_demo` smoke binary            |
| `TACCAP_BUILD_TESTS`    | `OFF`   | Build the gtest suite under `cpp/tests/`        |

### 5c. Integrate into another CMake / ROS 2 project

The SDK currently **installs no headers and exports no CMake package config**,
so `find_package(taccap-gripper)` does not work. Consume it as a source
subdirectory instead:

```cmake
add_subdirectory(path/to/taccap-gripper taccap-gripper-build)
target_link_libraries(my_target PRIVATE taccap_core)
```

`taccap_core` propagates its public include directory and the OpenCV / spdlog
dependencies to `my_target`. Copying `libtaccap_core.so` on its own is not
enough for a C++ integration — you would also need the matching public headers
and dependencies — so that route is not recommended.

### 6. Verify

```bash
# Python — note `env -u PYTHONPATH`, see the note below
env -u PYTHONPATH python -c "import xense.taccap as t; print(t.hello()); print(t.__version__)"
# → taccap-gripper OK; version <version>
# → <version>          # both lines match python/xense/taccap/_version.py

# Python tests (hardware-free cases always run; IMU cases skip without a gripper)
env -u PYTHONPATH pytest python/tests

# C++ tests (only if TACCAP_BUILD_TESTS=ON)
ctest --test-dir build --output-on-failure
```

**Hardware self-check.** With a gripper plugged in, the scan should print one
line per connected unit:

```bash
python -c "from xense.taccap import scan_grippers
for g in scan_grippers():
    print(f'side={g.side.name} role={g.role.name} ch343={g.mcu_serial} fw_sn={g.firmware_sn!r}')"
```

A single line with only one gripper attached is normal — left and right do not
have to be present together. Under the current SN scheme `firmware_sn` is
non-empty and `role` is `Leader` / `Follower`. A legacy SN, a unit whose SN was
never burned, a failed SN read on a cold start, or old firmware can all show an
empty SN / `Unknown` — do not read that alone as "firmware < V1.6". An empty
scan usually means the serial permissions from step 4 have not taken effect
yet, or ModemManager has grabbed the `/dev/ttyACM*` node. The equivalent checks
for the wrist camera and the visuotactile sensors are in
[USAGE.md](USAGE.md) §0.

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

