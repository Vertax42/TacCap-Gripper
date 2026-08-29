# 示例脚本

<!-- 从 README.md 拆出，保持内容不变；README 只保留入门路径。 -->

## Examples

All scripts live under `python/examples/`. Enable C++ examples with
`-DTACCAP_BUILD_EXAMPLES=ON` (they're off by default).

## 选谁:统一用 `left` / `right`

所有需要指定设备的示例都用**同一个位置参数**:`left` / `right`,或直接给序列号。
没有 `--sn` / `--side` / `--device` 这类各写各的开关。

```bash
python python/examples/calibrate.py right
python python/examples/fisheye_cal.py show right
python python/examples/leader_normalized_position.py right
python python/examples/wrist_camera.py right
python python/examples/ota_update.py tc-gu-01-master.bin right
python python/examples/ota_update.py --get-status right
```

只插了一台时可以省略(`calibrate.py` 除外 —— 它会改硬件状态,所以要求显式指定)。
侧别一律来自**固件烧录的 SN**(`Cmd::GetSn`),不是 CH343 芯片 SN;腕相机的序号
与它所在夹爪一致,所以 `left` 在哪个示例里都指同一半设备。解析逻辑只有一份,在
`_calib_flow.resolve_target()` 里。

| Script                           | What it does                                                                                                                                                                                                                                                                                                                                                                                |
| -------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `rerun_dual_with_tracker.py`     | Dual-gripper IMU/encoder + Pico4 motion-tracker 6-DoF poses in one viewer. Requires [`xensevr_pc_service_sdk`](https://github.com/Vertax42/Xense-Pico-Teleop-Interface) and the XenseVR PC Service running. Use `--left-tracker-sn` / `--right-tracker-sn` to map tracker SNs to sides. (Cameras are owned by the external camera service and not shown here.)                                  |
| `calibrate.py`                   | Per-gripper encoder calibration CLI, selected by `left` / `right` (or an explicit SN) — latches the zero **and stores the measured travel span** (`Cmd::EncoderMaxCal`), which is what unlocks normalized position. Shows raw + cooked side-by-side, then a live `raw \| cooked \| position 0..1` readout. Checks firmware support before writing anything. See [Calibration](#calibration).                                                          |
| `motor_mit_control.py`           | Primitive demo of the raw MIT submission API (`submit_impedance`) with the out-of-band health channel (`control_stats` / `read_status`).                                                                                                                                                                                                                                                    |
| `gripper_console.py`             | Interactive MIT force-position console using the SDK device/transport APIs. `j/k` provide incremental point jogging (short or long press); `o/c/h` open, close, or hold; `e/d/f` enable, disable, or clear a fault. Shows live status, command counters, force parameters, and envelope settings. See [interactive console implementation](GRIPPER_CONSOLE.md). **Real motion + grip force.** |
| `gripper_force_grasp_test.py`    | Gentle force grasp — the *force* half of force-position control, via host-side contact detection. Closes in small steps with a soft impedance and detects contact from **position stall**, not a torque threshold (the gripper's own restoring torque grows as it closes, so a torque threshold false-triggers). **Real motion + grip force — use a soft object or none.** |
| `impedance_control.py`           | `ControlLoop` 的用法:`set_target(0..1)` + `observation()` 走一遍开合序列。同时是**运动安全包络**的配置入口(`--show-envelope` / `--set-envelope --peak --cont --temp-wall`)。**真实运动。** |
| `force_position_control.py`      | `ForcePositionController` 的用法:同样两个调用,但被挡住后切 `kp=kd=0` 纯 `tau_ff` 保持。输出里 `cmd` 与实测力矩的差就是固件的热降额。**真实运动 + 夹持力。** |
| `fisheye_cal.py`                 | Read/write the flash-persisted calibration records (V2.0/V2.1): `show`, `set-fisheye` (flags or an OpenCV `.npz` holding `K`/`D`), `set-encoder-max`, and `measure-encoder-max` — the guided close-zero → open-sample → store flow that unlocks normalized leader position.                                                                                                                     |
| `wrist_camera.py`                | Stand-alone wrist-camera viewer, selected by `left` / `right` (or an XC serial) like every other example. **XC wrist cameras only** — a GSPS visuotactile serial or a raw `/dev/videoN` path is refused, since those sensors belong to `xensesdk`. Fisheye undistortion on a switch, **off by default like the SDK itself**: `--undistort` / `--compare` (raw \| rectified side by side), `--balance`, and `u` / `[` `]` to cycle live. Intrinsics come from the same-side gripper via `resolve_fisheye()`, from a `.npz`, or from the SDK reference values with `--no-mcu`. Headless with `--no-display` (+ `--save-dir` for one PNG/s). |
| `leader_normalized_position.py`  | Streams a leader gripper's opening as `0..1` via `normalize_position=True`, with a live bar. Needs the encoder-max record (or `--encoder-max-rad` to bypass the firmware read).                                                                                                                                                                                                              |
| `ota_update.py`                  | Firmware OTA flashing CLI with progress + post-flash status probe. **Risky — wrong artefact bricks the MCU.**                                                                                                                                                                                                                                                                               |
| `v4l2_probe.py`, `v4l2_sweep.py` | Manual V4L2 bringup probes for the wrist / OG cameras (discovery is MCU-only and no longer enumerates them). Also handy when a firmware SN isn't burned yet.                                                                                                                                                                                                                                  |
| `leader_demo` (C++)              | Reports streaming rates for a single leader gripper over 5 seconds.                                                                                                                                                                                                                                                                                                                         |
