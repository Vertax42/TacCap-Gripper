# taccap-gripper

C++17 / Python SDK for the **TacCap-Gripper** —— XenseRobotics' multimodal
tactile data-collection gripper. Provides a single namespace
(`xense::taccap::` / `xense.taccap`) for:

- Two visuotactile cameras (V4L2 + on-sensor calibration → rectify) — vendored
  from [libxensesdk](http://192.168.1.61/Vertax42/libxensesdk) in lite mode
- Wrist UVC camera
- IMU + encoder readout via TC-GU-01 serial protocol
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

## Quick start (development)

```bash
mamba env create -f environment.yml
mamba activate taccap

# C++ build (CMake/Ninja)
cmake -B build -G Ninja -DTACCAP_BUILD_EXAMPLES=ON
cmake --build build
./build/cpp/examples/taccap_hello

# Python (scikit-build-core)
pip install -e .
python -c "import xense.taccap; print(xense.taccap.hello())"
```

The build pulls `third_party/libxensesdk` as a git submodule and
configures it with `XENSE_LITE_BUILD=ON`, so no ONNX / CUDA / MIGraphX
runtime is required.

## Layout

```
taccap-gripper/
├── cpp/
│   ├── include/taccap/        # Public C++ headers (namespace alias + new types)
│   └── examples/              # C++ example programs
├── python/
│   ├── bindings/              # pybind11 module sources
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
