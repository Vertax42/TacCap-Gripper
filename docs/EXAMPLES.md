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
| `calibrate.py`                   | Per-gripper encoder calibration CLI, selected by `left` / `right` (or an explicit SN) — latches the zero **and stores the measured travel span** (`Cmd::EncoderMaxCal`), which is what unlocks normalized position. Shows raw + cooked side-by-side, then a live `raw \| cooked \| position 0..1` readout. Checks firmware support before writing anything. See [Calibration](#calibration).                                                          |
| `impedance_control.py`           | `ControlLoop` 的用法:`set_target(0..1)` + `observation()` 走一遍开合序列。同时是**运动安全包络**的配置入口(`--show-envelope` / `--set-envelope --peak --cont --temp-wall`)。**真实运动。** |
| `force_position_control.py`      | `ForcePositionController` 的用法:同样两个调用,但被挡住后切 `kp=kd=0` 纯 `tau_ff` 保持。输出里 `cmd` 与实测力矩的差就是固件的热降额。**真实运动 + 夹持力。** |
| `gripper_console.py`             | 单夹爪键盘控制台,`--mode impedance` / `--mode force-position` 两种控制器共用一个 UI,并内置**运动安全包络**的配置入口。表头常驻 `ENFORCED` / `*** INACTIVE ***`。**真实运动 + 夹持力。** 见 [力位混合控制器](#力位混合控制器)。 |
| `read_intrinsics.py`             | **只读**腕相机内参,输出 JSON。走 `resolve_fisheye()` 而不是 `read_fisheye()` —— 没标定过的机器对读请求回的是**全零记录而不是 NACK**,直接拿去矫正会把每一帧变全黑。输出的 `source` 字段标明内参来自设备标定(`device`)还是 SDK 参考值(`reference`),`--require-device` 让后者直接失败退出。JSON 走 stdout、诊断走日志,可以直接 `> cal.json` 或管道给 `jq`。记录里**不存图像尺寸**,所以 `assumed_image_size` 把 640x480 这个假设显式写出来。**只读,不写 flash、不动电机。** |
| `fisheye_cal.py`                 | Read/write the flash-persisted calibration records (V2.0/V2.1): `show`, `set-fisheye` (flags or an OpenCV `.npz` holding `K`/`D`), `set-encoder-max`, and `measure-encoder-max` — the guided close-zero → open-sample → store flow that unlocks normalized leader position.                                                                                                                     |
| `wrist_camera.py`                | Stand-alone wrist-camera viewer, selected by `left` / `right` (or an XC serial) like every other example. **XC wrist cameras only** — a GSPS visuotactile serial or a raw `/dev/videoN` path is refused, since those sensors belong to `xensesdk`. Fisheye undistortion on a switch, **off by default like the SDK itself**: `--undistort` / `--compare` (raw \| rectified side by side), `--balance`, and `u` / `[` `]` to cycle live. Intrinsics come from the same-side gripper via `resolve_fisheye()`, from a `.npz`, or from the SDK reference values with `--no-mcu`. Headless with `--no-display` (+ `--save-dir` for one PNG/s). |
| `leader_normalized_position.py`  | Streams a leader gripper's opening as `0..1` via `normalize_position=True`, with a live bar. Needs the encoder-max record (or `--encoder-max-rad` to bypass the firmware read).                                                                                                                                                                                                              |
| `ota_update.py`                  | Firmware OTA flashing CLI with progress + post-flash status probe. **Risky — wrong artefact bricks the MCU.**                                                                                                                                                                                                                                                                               |
| `leader_demo` (C++)              | Reports streaming rates for a single leader gripper over 5 seconds.                                                                                                                                                                                                                                                                                                                         |


## 力位混合控制器

`ForcePositionController` 和纯阻抗的区别在被挡住之后:

    kp=0 的速度阻尼闭合  ->  接触判定  ->  kp=kd=0 的纯 tau_ff 保持

**要可设定的夹持力就必须用它。** 阻抗模式下夹持力不是设定值:堵转保持把命令目标
钳在判定成立那一刻夹爪所在的位置(`control_loop.cpp` `stall_clamp_ = here`),之后
位置误差 ~ 0,`kp x 误差` ~ 0,残余力矩只是物体回弹留下的那点误差 —— 既不可设定
也不可复现(实测同一物体两次落在 0.30 / 0.40 Nm)。力位混合进入 `HOLDING_FORCE`
后 `kp=kd=0`,帧里只剩前馈力矩,`grasp_torque_nm` 就是那个力矩本身。

### 0. 先写运动安全包络(每台设备一次,掉电保持)

包络在**固件侧**,是 MIT 路径上唯一谁都绕不过的一层,出厂默认不启用
(`GripperConfig.reserved` 全 0 → `flags=0`)。主机侧的控制器替代不了它:链路
100 Hz 相位锁,主机反应下限几十毫秒,8 rad/s 下就是 0.24 rad。

```bash
# 写入并启用后退出(set 在 show 之前执行),默认 peak=2.0 cont=1.6、温度走固件 90/100
python python/examples/gripper_console.py --set-envelope --show-envelope

# 写完直接进控制台
python python/examples/gripper_console.py --set-envelope --mode force-position

# 只看不写
python python/examples/gripper_console.py --show-envelope

# 等价入口,参数名一致
python python/examples/impedance_control.py --set-envelope --peak 2.0 --cont 1.6
```

读回 `flags=0x2003` 即生效(高 4 位 layout 2 | `VALID` | `ENFORCE`)。

| 包络参数 | 默认 | 含义 |
|---|---|---|
| `--peak` | 2.0 Nm | 运动瞬态上限,**同时决定接近速度**(约 `peak/kd`) |
| `--cont` | 1.6 Nm | 可持续上限,I²t 降额的下限 —— **长期保持的实际天花板** |
| `--temp-derate-start` | 0 → 固件 90 °C | 温度降额起点 |
| `--temp-wall` | 0 → 固件 100 °C | 温度墙,之上只留 0.30 Nm |

传 0 和传 90/100 等价:0 表示"用固件默认",而固件默认就是 90/100。注意两处
显示不一致但都对 —— `[envelope]` 那行是**原始记录**,存的 0 就打印 `temp=0/0C`;
控制台表头那行已经把 0 解释成固件默认,所以打印 `temp=90/100C`。

包络**不是** `start()` 的前提 —— `start()` 只校验设备持久化的 0x700B 启动上限
(必须 <= `motion_torque_limit_nm`)。包络没开照样启动,只是长时间保持没有 I²t
和温度墙兜底。

### 1. 启动

```bash
python python/examples/gripper_console.py --mode force-position
python python/examples/gripper_console.py --mode force-position --grasp-torque 1.2
python python/examples/gripper_console.py --mode force-position \
       --grasp-torque 1.2 --close-speed 0.5 --contact-torque 0.080

# 非交互版本:同一个控制器,跑一遍固定 target 序列
python python/examples/force_position_control.py --grasp-torque 1.2
```

### 2. 命令行参数

| 开关 | 映射到 | 默认 | 说明 |
|---|---|---|---|
| `--grasp-torque` | `grasp_torque_nm` | 0.35 | **接触后的保持力矩,就是夹持力设定值。**上限 `hold_torque_limit_nm` |
| `--close-speed` | `close_speed_radps` | 0.5 | 闭合速度 rad/s |
| `--contact-torque` | `contact_torque_nm` | 0.080 | 接触力矩**下限**,不是阈值。见下 |
| `--kp` | `position_kp` | 20.0 | 位置保持刚度。只收窄误差窗口(预算/kp),不抬高输出 |
| `--kd` | `position_kd` | 1.0 | 位置保持阻尼 |
| `--step` | — | 0.05 | `j`/`k` 步进量,归一化 0..1 |

`--grasp-torque` 的硬上限是 **1.8 Nm**(`hold_torque_limit_nm`,即电机额定力矩),
超了 `validate_config()` 直接抛 `invalid_argument`。另外**持续保持超过 `--cont`
会被包络的 I²t 降额拉回来**,所以默认包络下实际可用区间到 1.6 Nm。

### 3. `ForcePositionConfig` 完整字段(API 用户)

命令行只暴露了常用的几个,直接用 API 时全部可改:

| 字段 | 默认 | 说明 |
|---|---|---|
| `grasp_torque_nm` | 0.35 | 接触后的纯前馈保持力矩 |
| `hold_torque_limit_nm` | 1.8 | 无限期保持上限 = 电机**额定**力矩,硬校验 (0, 1.8] |
| `motion_torque_limit_nm` | 6.0 | 运动瞬态上限 = 电机**峰值**力矩,硬校验 (0, 6.0] |
| `close_position` | 0.0 | 闭合目标,归一化 |
| `close_speed_radps` | 0.5 | 闭合速度 |
| `brake_distance_rad` | 0.10 | 距目标多远时从速度切到钳位 PD |
| `contact_torque_nm` | 0.080 | 接触力矩下限(固件 `TASK_CANMOTOR_STALL_TORQUE_FLOOR_NM`) |
| `contact_vel_radps` | 0.035 | 该值以下即算"停住",与行程史无关 |
| `contact_vel_ratio` | 0.25 | 已移动过之后,沿运动方向速度低于命令速度的这个比例也算停住 |
| `contact_moved_rad` | 0.010 | 比例判据的解锁行程 |
| `contact_samples` | 3 | 连续确认帧数;100 Hz 下对应固件 30 ms `stall_hold_ms` |
| `position_kp` / `position_kd` | 20.0 / 1.0 | 位置保持增益 |
| `startup_guard_ms` | 250 | 闭合起步阶段忽略加速力矩 |
| `status_timeout_ms` | 350 | 状态流超时 → 零命令 + `FAULT` |
| `motor_stream_hz` | 100 | 状态流速率 |

### 4. 接触判定:三条同时成立

```
contact = |torque| >= contact_torque_nm  且  运动停止  且  连续 contact_samples 帧
```

**做分离的是速度门,不是力矩数字。** 1.1.5 上空载闭合全行程实测(1578 采样):
自由行程 `|vel| >= 0.183 rad/s`(五倍于 0.035 的门)而 `|torque| <= 0.142 Nm`;
机械止点 `|vel| ~ 0.012`、`|torque| ~ 0.21`。零个自由行程采样同时通过两项。

所以**不要把 `--contact-torque` 往上调**。它只负责排除"停住但没受力",调高了会
变成够不着 —— kd 形式的 MIT 帧在堵转时反馈力矩只有命令的约 0.59,门放在命令值
附近就永远判不到接触,夹爪会一直推下去,而那正是这个控制器要防的堵转。

### 5. 状态机

`IDLE` → `HOLDING_POSITION` / `CLOSING` → `HOLDING_FORCE` / `OPENING` / `FAULT`。
夹住的判据是走到 **`HOLDING_FORCE`**。控制台底部状态行:

```
state=HOLDING_FORCE   contact=3  cmd=1.200Nm  grasp=1.200Nm  limit hold=1.80/motion=6.00/dev=6.00Nm
```

`cmd` / `grasp` 是**设定值**,实测力矩看 `Torq(Nm)` 那一列 —— 两者之差就是固件的
I²t 与温度墙降额。`dev` 是设备持久化的 0x700B,`start()` 会校验它。

### 6. 按键(与阻抗模式的差别)

| 键 | 力位混合下的行为 |
|---|---|
| `j` / `k` / `c` | `set_target()` —— 低于当前开度走接触感知路径,高于则有界张开 |
| `o` | `release()` —— 有界速度阻尼张开,不是位置阶跃 |
| `h` | `hold_position()` —— 取消运动/夹持,停在当前位置 |
| `f` | `motor.clear_fault()` **然后** `reset()` 退出 `FAULT` 态(顺序是硬性的) |
| `e` / `d` | 使能 / 失能 |
| `q` / ESC | 退出。退出路径必定先 `stop()` 下发零力矩,再 `disable()` |
