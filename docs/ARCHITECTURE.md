# TacCap-Gripper SDK — Architecture

> **Note (0.1.4):** the visuotactile (OG) capture/rectify path was removed from
> this SDK. The `libxensesdk` submodule, the `vision.hpp` alias header and the
> `TactileSensor` / `TactileFrame` classes are gone; visuotactile imaging now
> lives at the Python level via the `xensesdk` wheel. Diagrams and sections
> below that mention `vision.hpp`, `TactileSensor`, `Rectifier`, libxense
> `Sensor`/`Frame`, or the `third_party/libxensesdk` submodule are **historical**
> — the current C++ surface is gripper protocol + the wrist `Camera` (plain
> OpenCV) only.

This document captures the state of the SDK as of the close of step 4
(zero-config bilateral discovery + ergonomic component classes).

The SDK in this repository is the **C++ / Python device-access layer
only**. Higher-level products that build on top of it — dataset
recording tools, ROS 2 hardware-interface packages, lerobot Robot
adapters, the follower gripper with motor control — live in their own
repositories and are out of scope here.

**Protocol tracked:** wire framing **V1.8** (the body between HEAD and TAIL
is byte-stuffed; CRC over the unstuffed HEAD..PAYLOAD) + command set **V1.7**
(motor / CAN-id / gripper-config, plus motor_status_t / motor_impedance_ctrl_t
growth). Discovery is MCU-only; cameras are owned by an external camera
service. Side/role come from the firmware SN (`parse_serial`) with a
GetDevType fallback. The follower-only V1.7 commands are implemented to the
protocol but not yet validated on follower hardware.

---

## 1. Layered stack

```
┌────────────────────────────────────────────────────────────────────────┐
│                       USER CODE (Python / C++)                         │
│   - data-collection scripts, Jupyter notebooks, downstream products    │
└────────────────────────────┬───────────────────────────────────────────┘
                             │  xense.taccap (Python)  ↔  xense::taccap (C++)
┌────────────────────────────▼───────────────────────────────────────────┐
│  L4  AGGREGATE                                                         │
│  ─────────────                                                         │
│   LeaderGripper          aggregate object: owns Transport + IMU +      │
│                          Encoder; lifecycle (start/stop_streaming).    │
│                          Discovery open() auto-finds by MCU serial.    │
│                          Cameras/tactile are opt-in (open_cameras).    │
│                                                                        │
│   FollowerGripper        (future: same layout + Motor; not in repo yet)│
└────────────────────────────┬───────────────────────────────────────────┘
                             │
┌────────────────────────────▼───────────────────────────────────────────┐
│  L3  COMPONENTS                                                        │
│  ─────────────                                                         │
│   IMU            Encoder           Camera            TactileSensor     │
│   ──────────     ──────────────    ──────────────    ──────────────    │
│   read_once()    read_once()       read() (sync)     start(callback)   │
│   on_data(cb)    on_data(cb)       start(callback)   stop()            │
│                                    stop()            (rectified +      │
│   ImuSample      EncoderSample     CameraFrame        raw cv::Mat)     │
│   - mcu_ts_us    - mcu_ts_us       - host_time       TactileFrame      │
│   - accel_mps2   - position_rad    - frame_index                       │
│   - gyro_radps   - velocity_rad_s  - image (BGR8)                      │
│   - mag_uT       - status                                              │
│   - temp_c       - seq                                                 │
│   - seq                                                                │
└──────────┬─────────────────┬──────────────┬──────────────┬─────────────┘
           │                 │              │              │
           ▼                 ▼              ▼              ▼
┌─────────────────────┐  ┌─────────────────────┐  ┌─────────────────────┐
│  L2a  ASYNC TRANSPORT│  │  L2b  V4L2 CAPTURE   │  │  L2c  LIBXENSE LITE │
│  ───────────────────│  │  ───────────────────│  │  ───────────────────│
│   Transport         │  │   cv::VideoCapture  │  │   xense::Sensor     │
│   - background      │  │   (wrist camera     │  │   xense::Rectifier  │
│     reader thread   │  │    only)            │  │   xense::Context    │
│   - ACK matching    │  │                     │  │   (visuotactile path│
│     (seq → promise)│  │                     │  │    incl. on-sensor  │
│   - subscribe(cmd)  │  │                     │  │    flash calibration│
│   - send_cmd_no_ack │  │                     │  │    via V4L2 XU)     │
│   - host-side retry │  │                     │  │                     │
└──────────┬──────────┘  └─────────────────────┘  └──────────┬──────────┘
           │                                                  │
           ▼                                                  │
┌─────────────────────┐                                       │
│  L1   PROTOCOL +     │                                       │
│       BUS WIRE       │                                       │
│  ───────────────────│                                       │
│   pack_frame        │                                       │
│   try_parse_frame   │                                       │
│   FrameParser       │                                       │
│   crc16_modbus      │                                       │
│   stuff/unstuff     │                                       │
│   SerialBus(termios)│                                       │
└──────────┬──────────┘                                       │
           │                                                  │
           ▼                                                  ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                              KERNEL / DRIVERS                          │
│                                                                        │
│      /dev/ttyACM*  (CH343 USART3 @ 3 Mbps)        /dev/video*  (UVC)   │
└─────────────────────────────────────────────────────────────────────────┘
                                ▲
                                │ over USB
                                ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                  TC-GU-01 GRIPPER HARDWARE                             │
│   STM32H562 + ThreadX  ──── USART3 (3 Mbps) ────  IMU / Encoder /      │
│                                                   Eskin (G2+) / Motor │
│                        ──── FDCAN1 (1 Mbps) ────  灵足 motor (follower) │
│                                                                        │
│   2× OG-series planar visuotactile sensors  ──── UVC                   │
│   1× XC-series wrist UVC camera              ──── UVC                  │
└─────────────────────────────────────────────────────────────────────────┘
```

---

## 2. Module map

```
taccap-gripper/
├── cpp/
│   ├── include/taccap/
│   │   ├── version.hpp.in                        L0  generated by CMake
│   │   ├── error.hpp                             Error / ProtocolError /
│   │   │                                          CrcError / IoError /
│   │   │                                          TimeoutError
│   │   ├── vision.hpp                            using-decls re-exporting
│   │   │                                          libxense lite into the
│   │   │                                          xense::taccap namespace
│   │   ├── protocol/
│   │   │   ├── commands.hpp / .cpp               L1  Address, FrameType,
│   │   │   │                                          Cmd, ErrorCode enums
│   │   │   ├── payloads.hpp                      L1  packed POD structs
│   │   │   │                                          (ImuData, EncoderData,
│   │   │   │                                          MotorPosCtrl, ...)
│   │   │   └── codec.hpp / .cpp                  L1  typed encode/decode
│   │   ├── bus/
│   │   │   ├── frame.hpp / .cpp                  L1  CRC16, pack/parse,
│   │   │   │                                          FrameParser, byte
│   │   │   │                                          stuffing
│   │   │   ├── serial_bus.hpp / .cpp             L1  termios at 3 Mbps
│   │   │   └── transport.hpp / .cpp              L2a async transport:
│   │   │                                              reader thread, ACK
│   │   │                                              matching, subscribe
│   │   ├── components/
│   │   │   ├── imu.hpp / .cpp                    L3  ImuSample + IMU
│   │   │   ├── encoder.hpp / .cpp                L3  EncoderSample +
│   │   │   │                                          Encoder
│   │   │   ├── camera.hpp / .cpp                 L3  cv::VideoCapture
│   │   │   │                                          wrapper for wrist
│   │   │   └── tactile_sensor.hpp / .cpp         L3  libxense Sensor +
│   │   │                                              Rectifier facade
│   │   ├── discovery.hpp                         L4 (helper) zero-config
│   │   └── leader_gripper.hpp                    L4  aggregate object
│   ├── src/                                      mirrors include/
│   ├── examples/
│   │   ├── taccap_hello.cpp                      smoke (no hardware)
│   │   └── leader_demo.cpp                       5-second multistream on
│   │                                              real hardware
│   └── tests/
│       ├── test_crc.cpp                          known CRC vectors
│       ├── test_frame.cpp                        pack/parse + FrameParser
│       ├── test_codec.cpp                        per-payload roundtrip
│       ├── test_python_compat.cpp                C++ vs GUI Python parity
│       ├── test_stuff.cpp                        byte-stuffing roundtrip
│       ├── test_transport.cpp                    PTY-based fake firmware
│       └── test_components.cpp                   unit conversions
│
├── python/
│   ├── bindings/
│   │   ├── module.cpp                            pybind11 entry point
│   │   │                                          (enums, Frame, Transport,
│   │   │                                          SerialBus, codec helpers)
│   │   └── components.cpp                        (ImuSample/EncoderSample
│   │                                              with numpy fields,
│   │                                              IMU/Encoder/Camera/
│   │                                              TactileSensor,
│   │                                              LeaderGripper, discovery)
│   └── xense/taccap/                             PEP 420 namespace package
│       ├── __init__.py                           re-exports the C-extension
│       └── _version.py
│
├── third_party/
│   └── libxensesdk/                              git submodule pinned at
│                                                  feat/lite-build@7d4687e
│                                                  (XENSE_LITE_BUILD=ON,
│                                                  no ML deps)
│
├── docs/
│   └── ARCHITECTURE.md                           ← you are here
│
├── environment.yml                               mamba env (Python 3.12,
│                                                  conda-forge only,
│                                                  opencv-python==4.12.0.88
│                                                  pinned via pip)
├── pyproject.toml                                scikit-build-core wheel
├── CMakeLists.txt                                top-level orchestrator
└── README.md
```

---

## 3. Data flow

### 3.1 Synchronous command (e.g. `imu.read_once()`)

```
  Python / C++ user
        │ send_cmd(GetImu)
        ▼
  Transport::send_cmd()
        │ pack_frame(...)           [L1]
        │ SerialBus::write(...)     [L1]
        │ create promise<AckResponse>, register under seq
        │ future.wait_for(timeout)
        │
        │   ┌──────────────────────── concurrent ────────────────────────┐
        │   │  reader_thread:                                            │
        │   │    SerialBus::read() → bytes                               │
        │   │    FrameParser::feed() → emits Frame                       │
        │   │    dispatch_(Frame):                                       │
        │   │      type==ACK → handle_ack_:                              │
        │   │        match seq → promise.set_value(AckResponse)          │
        │   └────────────────────────────────────────────────────────────┘
        │
        ▼
  AckResponse{ seq, cmd, error_code, data, is_nack }
        │  (decode wire format: cmd==0 ⇔ NACK)
        ▼
  IMU::decode(ack.data) → ImuSample (unit-converted)
```

### 3.2 Streaming DATA (e.g. `imu.on_data(cb)`)

```
  Python / C++ user
        │ imu.on_data(callback)
        ▼
  IMU::on_data → Transport::subscribe(Cmd::GetImu, cb)

  Earlier: leader.start_streaming()
        │ pack StreamConfig{ source_mask, mode, rates, iface }
        │ Transport::send_cmd(StartStream, cfg) → ACK
        ▼
  Firmware now pushes DATA frames at the configured rate.

  reader_thread (background):
        SerialBus::read() → bytes
        FrameParser::feed() → emits Frame
        dispatch_(Frame):
          type==DATA → handle_data_(f):
            for each subscriber matching f.cmd:
              cb(f)  → IMU::decode → ImuSample → user callback
                      (Python: gil_scoped_acquire + try/discard_as_unraisable)
```

> The Camera (3.3) and Tactile (3.4) paths below are **opt-in** — they run
> only when the device was opened (a gripper constructed with
> `open_cameras=true`, or a standalone `Camera` / `TactileSensor`). They are
> not part of the default gripper lifecycle, which is MCU-only.

### 3.3 Camera (wrist, V4L2)

```
  user → camera.start(callback)
        ▼
  Camera::start → spawn capture_loop_ thread
        loop:
            cv::VideoCapture::read() → cv::Mat
            CameraFrame{ host_time, frame_index, image }
            cb(CameraFrame)
                Python: gil_scoped_acquire + numpy view via mat_to_numpy
```

### 3.4 Tactile (visuotactile, libxense lite + Rectifier)

```
  user → tactile.start(callback)
        ▼
  TactileSensor::start
        libxense Sensor::start([this](xense::Frame raw) {
            cv::Mat raw_mat   = clone(raw)
            cv::Mat rect_mat  = Rectifier::process(raw)
            cb(TactileFrame{ host_time, frame_index, raw_mat, rect_mat })
        })
                Python: numpy views for both raw and rectified
```

---

## 4. Discovery (zero-config, MCU-only)

Discovery is MCU-only. The wrist camera and OG visuotactile sensors are
owned by an external camera service now, so the scanner no longer
enumerates them — one CH343 MCU board = one gripper unit, and there is no
USB-hub grouping step any more.

The firmware SN follows the TacCap scheme and encodes both side and role:

```
  TCGU01 A24 Z 0001 m     line: Z=R&D / A=production   seq last digit: odd→Left
  product batch line seq patch  patch: m=Master(leader) / s=Slave(follower)
```

```
scan_grippers():
  1) walk /dev/serial/by-id/, keep usb-1a86_USB_Dual_Serial_<SN>-if02
  2) for each MCU, open a transient Transport and read the firmware SN
     (Cmd::GetSn), then parse_serial(): side := odd-last(seq) -> Left else
     Right; role := patch m|s -> Leader|Follower. Side falls back to the
     CH343 chip-SN parity if the SN read fails; role is Unknown for legacy/
     empty SNs.
  3) emit one GripperEndpoints{side, role, mcu_device, mcu_serial,
     firmware_sn} per MCU board.

API:
   scan_grippers()       -> list of GripperEndpoints
   find_one()            -> single gripper, throws if 0 or >1
   find_left()  / find_right()    -> by side  (odd/even seq digit)
   find_leader()/ find_follower() -> by role  (SN patch suffix m/s)
   parse_serial(sn)      -> ParsedSerial{product,batch,line,sequence,side,role,valid}
```

---

## 5. Public API surface

### C++

```cpp
#include <taccap/leader_gripper.hpp>

auto g = xense::taccap::LeaderGripper::open();   // unique_ptr; MCU-only, throws on no device

g->imu().on_data    ([](const xense::taccap::ImuSample& s)     { ... });
g->encoder().on_data([](const xense::taccap::EncoderSample& s) { ... });

g->start_streaming(/*imu_hz=*/100, /*encoder_hz=*/100);
std::this_thread::sleep_for(5s);
g->stop_streaming();

// Wrist camera + tactile are opt-in — an external camera service owns the
// V4L2 devices. Construct explicitly with open_cameras=true to drive them:
//   LeaderGripper::Config cfg; cfg.mcu_device = ...; cfg.wrist_video = ...;
//   cfg.tactile_left_serial = ...; cfg.open_cameras = true;
//   auto g = std::make_unique<LeaderGripper>(cfg);
//   g->tactile_left().start(...); g->wrist_camera().start(...);
```

### Python

```python
import time
from xense.taccap import LeaderGripper

with LeaderGripper.open() as g:          # MCU-only; cameras off by default
    g.imu.on_data           (lambda s: print(s.accel_mps2, s.temperature_c))
    g.encoder.on_data       (lambda s: print(s.position_rad))

    g.start_streaming(imu_hz=100, encoder_hz=100)
    time.sleep(5)
    g.stop_streaming()

# Opt-in cameras: LeaderGripper(mcu_device, wrist_video=..., open_cameras=True)
# then g.tactile_left.start(...) / g.wrist_camera.start(...); or use the
# standalone Camera / TactileSensor classes directly.
```

---

## 6. Threading model

| Thread                          | Owner                      | Lifetime                        |
|---------------------------------|----------------------------|---------------------------------|
| user thread (`send_cmd` blocks) | caller                     | per call                        |
| `Transport::reader_loop_`       | one per Transport          | open()→stop()                   |
| `Camera::capture_loop_`         | one per Camera::start()    | start()→stop()                  |
| libxense sensor capture threads | libxense Sensor (×2 OG)    | TactileSensor::start()→stop()  |

Callbacks fire on the **producer** thread (reader / capture). Python
callbacks reacquire the GIL via `py::gil_scoped_acquire`; exceptions
inside callbacks are reported via `discard_as_unraisable` so a buggy
callback never tears down the producer.

Lifetime contract:
- `LeaderGripper` is *not* copyable / movable — its members own
  threads / mutexes. Construct once via `open()` (returns
  `std::unique_ptr`).
- `~LeaderGripper` calls `stop_streaming()` and component destructors,
  joining all threads. Idempotent.

---

## 7. Dependencies

```
build-time:
   conda-forge:  cmake, ninja, gcc-14, libopencv 4.12, eigen, openssl,
                 zlib, nlohmann_json, gtest, pybind11, scikit-build-core,
                 numpy, pyserial
   pip:          opencv-python==4.12.0.88   (single source for cv2 in
                                               python land — pinned across
                                               XenseRobotics SDKs)

runtime (linked into libtaccap_core.so):
   libxensesdk.so   ← from third_party/libxensesdk submodule
                       (XENSE_LITE_BUILD=ON, NO onnxruntime / cuda /
                        migraphx / hip / rknn / openvino / directml /
                        coreml / directml. The lib weighs ~1.9 MB.)
   libopencv_core.so / imgproc / imgcodecs / video / videoio
   spdlog (fetched via FetchContent, header-only)
   pthread

NOT linked in (deliberately): any ML inference runtime, ROS, lerobot.
This is a hardware-access SDK, not a model server.
```

---

## 8. What this SDK is **not**

The following live in their own repositories on top of this SDK and
will be implemented later:

| Concern                  | Where it lives                          |
|--------------------------|-----------------------------------------|
| Dataset recording (hdf5 / mcap, time alignment, episode markers) | a separate tool / script repo         |
| ROS 2 node + hardware_interface package | `taccap_gripper_ros2` (separate repo) |
| lerobot Robot adapter     | fork of `lerobot-xense feature/v5.1_dev` with a `taccap_gripper` Robot class |
| Follower gripper + Motor  | `cpp/include/taccap/components/motor.hpp` + `follower_gripper.hpp` (this repo, future commits — guarded behind hardware availability) |
| Higher-level orchestration (teleop, episode controller, replay, visualisation) | downstream applications |

Keeping this SDK narrow lets each downstream consumer pick exactly the
hardware it needs (e.g. a ROS 2 node may want only IMU + Encoder DATA
streams, no cameras) and assemble its own data-flow on top.

---

## 9. Verified end-to-end (real hardware)

5-second multistream capture on the lab leader (`OG000477` /
`OG000478` / `XC000008`, MCU `5C2C247728`). This historical run was taken
with the cameras opened (`open_cameras=true`); the default gripper
lifecycle is now MCU-only (IMU + encoder):

```
IMU         : 506 frames | 101.2 fps   (firmware caps at ~100 Hz)
Encoder     : 506 frames | 101.2 fps
Tactile L   : 156 frames |  31.2 fps   (OG000477)
Tactile R   : 153 frames |  30.6 fps   (OG000478)
Wrist cam   : 149 frames |  29.8 fps   (XC000008)

discovery  : side=Right (MCU SN '5C2C247728' last digit 8 → even)
test suite : 48 / 48 gtest pass (PTY-based transport tests included)
```
