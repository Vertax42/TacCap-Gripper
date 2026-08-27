# 使用文档 —— 三路数据怎么开起来

<!-- 面向"第一次要把设备跑起来"的人:触觉、视觉、夹爪读数与控制,各一节,
     每节都是从插上线到拿到数据的完整路径。API 细节见 README 与各专题文档。 -->

一台 TacCap-Gripper 上有三路互相独立的数据,**分别由不同的软件栈负责**,
这是本文档最重要的一件事:

| 数据 | 硬件 | 由谁负责 | 设备节点 |
| --- | --- | --- | --- |
| **夹爪** 读数与控制 | STM32 MCU(IMU / 编码器 / 电机) | **本 SDK** `xense.taccap` | `/dev/serial/by-id/...-if02` |
| **视觉** 腕部相机 | XC 系列 UVC 相机 | **本 SDK** 的 `Camera`(可选) | `/dev/video*` |
| **触觉** 视触觉(OG)传感器 | GSPS01 视触觉模组 | **`xensesdk` wheel,不在本 SDK 内** | `/dev/video*` |

三者没有共享的会话:开一路不需要开另一路,某一路坏了也不会拖垮其它两路。
夹爪走串口,两个相机走 USB 视频,互不抢占同一个句柄 —— 但它们**共享 USB 带宽**,
见[三路一起跑](#三路一起跑)。

> **示例脚本一律用位置参数 `left` / `right` 选设备**(或直接给序列号),没有
> `--sn` / `--side` / `--device` 之类的开关 —— 见
> [EXAMPLES.md](EXAMPLES.md#选谁统一用-left--right)。

前置的安装、设备权限、`PYTHONPATH` / `LD_LIBRARY_PATH` 陷阱见
[docs/INSTALL.md](INSTALL.md)。下文假定 `xense-taccap` 环境已激活、
`import xense.taccap` 能成功。

---

## 0. 先确认设备在

三条线各有各的枚举方式,**先分别确认,再写业务代码** —— 大多数"读不到数据"
最后都是设备没被认出来。

```bash
# 夹爪(MCU):按固件烧录的 SN 发现,不依赖 udev 规则
python -c "from xense.taccap import scan_grippers, Side
for g in scan_grippers():
    s='L' if g.side==Side.Left else 'R'
    print(f'  [{s}] {g.role} ch343={g.mcu_serial} fw_sn={g.firmware_sn!r}')"

# 视觉(腕相机)+ 触觉(OG):都在 /dev/v4l/by-id 下,按序列号区分
python python/examples/wrist_camera.py --list

# 触觉(OG):由 xensesdk 自己扫,返回 {序列号: cam_id}
python -c "from xensesdk import Sensor; print(Sensor.scanSerialNumber())"
```

健康的输出:夹爪那条打出 `[L]` / `[R]` 且 `fw_sn` 非空;`--list` 那条把腕相机
(`XC…`)和视触觉(`GSPS01…`)分开列出;OG 那条打出形如 `{'OG000352': 10}` 的
字典。`fw_sn` 为空说明固件还没烧 SN(或固件早于 V1.6),此时侧别会是
`Side.Unknown`。

> **三类设备靠序列号区分,不靠设备号。** `/dev/videoN` 的编号随插拔顺序变,而且
> 腕相机和视触觉挨着枚举 —— 认错了就会把触觉传感器当相机打开。序列号语法和
> 夹爪固件 SN 是同一套(**序号末位单左双右**,`m` = leader / `s` = follower):
>
> | 设备 | 语法 | 例 |
> | --- | --- | --- |
> | 夹爪 | `TCGU01<批次><产线><序号><m\|s>` | `TCGU01A28Z0116m` |
> | 腕相机 | `XC<批次><产线><序号><m\|s>` | `XCA28Z0116m` |
> | 视触觉 | `GSPS01<批次><产线><序号>` | `GSPS01A31Z0049` |
>
> 腕相机的序号和它所在夹爪的序号一致,所以 `left` / `right` 一个选择器就能同时
> 定位夹爪和它的腕相机。

> **腕相机不在发现结果里,这是设计使然。** 发现层只认 MCU:UVC 设备通常由外部
> 相机服务持有,SDK 不去抢。所以夹爪对象默认 **不打开** 腕相机
> (`open_cameras=False`),要用就自己给设备路径。

---

## 1. 夹爪:读数与控制

### 1.1 打开

```python
import xense.taccap as t

g = t.LeaderGripper.open()        # 恰好插了一台时;插了多台会抛 IoError
```

多台的场合别用 `open()`,一次扫描拿全再按侧别/角色挑,避免重复探测:

```python
from xense.taccap import LeaderGripper, FollowerGripper, scan_grippers, Side, Role

eps   = scan_grippers()
left  = next(e for e in eps if e.side == Side.Left)
lead  = next(e for e in eps if e.role == Role.Leader)
g     = LeaderGripper(mcu_device=left.mcu_device)
```

leader 与 follower 是**两种硬件**,由 SN 尾缀决定(`m` = leader,`s` = follower),
类只是决定你拿到哪套命令面 —— 用错类不会自动纠正,只会在发命令时 NACK。

### 1.2 读数:一次性 vs 流式

一次性读(阻塞,等 ACK),适合标定、自检这类低频场合:

```python
s = g.encoder.read_once()
print(s.position_rad, s.raw_position_rad)   # 熟化值(钳到 >= 0) vs 原始值
print(g.imu.read_once())
```

流式(固件按固定频率推,回调在后台线程里跑),这是采数据的正路:

```python
sub_e = g.encoder.on_data(lambda s: print("enc", s.position_rad))
sub_i = g.imu.on_data(lambda s: print("imu", s.accel_mps2, s.gyro_radps))

g.start_streaming(imu_hz=100, encoder_hz=100)   # leader:两路都可单独关(0)
...
g.stop_streaming()
```

follower 只有一路可流:**电机状态**。IMU/编码器在这个角色上被固件编译掉了,
所以签名不同,`motor_hz=0` 会直接抛 `IoError(EINVAL)`,而不是开一条空流:

```python
f = t.FollowerGripper.open()
f.motor.on_status(lambda s: print(s.actual_pos, s.actual_torque))
f.start_streaming(motor_hz=100)     # 固件把 1 kHz 整除,只有 1000 的因数能精确命中
```

> 频率不是随便填的:固件用 1 kHz 整除,非因数的频率会被它悄悄调整;
> SDK 检测到这种情况会打 warning,但不会 NACK。

### 1.3 归一化开口度(0 = 闭合,1 = 张开)

原始编码器是弧度,策略/数据集通常要 `[0, 1]`。打开 `normalize_position=True`
后,一次性读和每一个流式样本都会带上 `.position`:

```python
g = t.LeaderGripper(mcu_device=dev, normalize_position=True)
s = g.encoder.read_once()
s.position_rad     # 0.65  —— 永远是弧度,含义不变
s.position         # 0.50  —— 归一化;没开这个标志时是 nan
g.position()       # 0.50  —— 一次性读+换算
```

这依赖固件里存的 **行程上限**(`Cmd::EncoderMaxCal`,固件 ≥ V2.1)。没标定过时
构造会抛 `ProtocolError`(固件答 `CalNotSet`,而不是给个假的 0)。标定顺序是
**先置零,再存行程**:

```bash
python python/examples/calibrate.py left            # 交互式:置零 + 存行程
python python/examples/fisheye_cal.py measure-encoder-max   # 只做行程那一步
```

细节见 [docs/CALIBRATION.md](CALIBRATION.md)。

### 1.4 控制(仅 follower)

follower 驱动一个 FDCAN 电机,控制原语是 **MIT 阻抗帧**(力位混合):
`kp`/`kd` 跟踪目标位置,前馈力矩叠加力的分量。动之前必须 enable:

```python
f = t.FollowerGripper.open()
f.motor.clear_fault()
f.motor.enable()
```

**推荐路径 —— `ControlLoop`**:C++ 后台线程按电机状态流的相位提交目标,
你的策略只碰两个非阻塞调用:

```python
loop = t.ControlLoop(f, hz=100, kp=20, kd=1)    # 默认 SubmitPhase.STREAM_LOCKED
loop.start()                                    # 以当前位置作为初始目标,不会跳
try:
    while running:
        obs = loop.observation()   # .position [0,1] / .velocity / .torque / .age_ms
        loop.set_target(policy(obs))
finally:
    loop.stop()
    f.motor.disable()
```

**硬物夹持/限力保持 —— `ForcePositionController`**:普通位置阻抗在物体挡住夹爪后
会继续积累 `kp × 位置误差`,不适合把目标长期放在完全闭合点。力位混合控制器改用:

1. `kp=0` 的速度阻尼闭合,目标位置误差不参与闭合力矩;
2. 接触判定,直接照搬固件开机自标定的做法(见下);
3. 接触位置锁存后切到 `kp=kd=0` 的纯 `tau_ff` 力矩保持。

**接触判定为什么是这样**:固件的开机自标定要解决的是同一个问题(闭合到堵转来找零
点),它的做法是**力矩下限 _并且_ 运动停止**,两个条件缺一不可
(`third_party/firmware/tc-gu-01` `App/tasks/task_canmotor.c`
`task_canmotor_is_stalled()`):

```
contact = 停止 AND |力矩| >= contact_torque_nm
停止    = |vel| <= contact_vel_radps,或(已经走过一段)沿运动方向的速度
          已跌到命令速度的 contact_vel_ratio 以下
确认    = 连续 contact_samples 帧(固件对应 stall_hold_ms,默认 30 ms)
```

| 固件常量 | 值 | SDK 字段 |
|---|---|---|
| `TASK_CANMOTOR_STALL_TORQUE_FLOOR_NM` | 0.080 Nm | `contact_torque_nm` |
| `TASK_CANMOTOR_STALL_VEL_RAD_S` | 0.035 rad/s | `contact_vel_radps` |
| `TASK_CANMOTOR_STALL_VEL_RATIO` | 0.25 | `contact_vel_ratio` |
| `TASK_CANMOTOR_STALL_MOVED_RAD` | 0.010 rad | `contact_moved_rad` |
| `stall_hold_ms` | 30 ms | `contact_samples`(100 Hz 下 3 帧) |

**做分离的是速度门,不是力矩数字。** 在 1.1.5 从爪上实测整段空载闭合(1578 帧):
自由行程 `|vel|` 始终 ≥ 0.183 rad/s,是 0.035 门限的五倍,同时 `|力矩|` ≤ 0.142 Nm;
机械止点处 `|vel|` ≈ 0.012 rad/s、`|力矩|` ≈ 0.21 Nm。**没有任何一帧同时满足两个
条件**。力矩项只需要否掉"停着但没受力",所以默认值直接用固件那个下限 0.080 Nm。

**不要把阈值定成命令力矩上限的比例。** 固件能用 ratio = 1.00,是因为它下发的是
**速度命令带 `max_torque`**,执行器自己的环会一路顶到那个上限;而 `kd` 形式的 MIT
帧不会。同一次实测:堵住时命令要 0.359 Nm,反馈只饱和到 0.213 Nm(≈0.59)。把阈值
放在命令上限处就是**永远够不到** —— 夹爪会一直推下去而从不锁存,正是这个控制器
本该消灭的堵转。

`brake_distance_rad` 那段减速也受 `grasp_torque_nm` 约束(而不是 6 Nm 运动上限):
它属于调用方的夹持动作,而且如果按运动上限走,最后 0.10 rad 会退化成误差钳位到
6 Nm 的 PD 推压 —— 堵转原样保留,并且低命令力矩还会把反馈压在接触下限以下,导致
什么都锁存不了。

控制器把力矩限制拆成两个职责明确的上限:

这两个上限就是**电机自身的两个额定值**,不是随手取的安全裕度:

- `motion_torque_limit_nm`:闭合/张开/位置保持过程中的速度阻尼和 PD **瞬时**力矩
  上限,最大 **6.0 Nm** —— 电机的**峰值力矩**;反馈力矩超过它会进入零力矩故障状态。
  6.0 Nm 同时也是固件对 `0x700B` 启动上限的默认值和最大值
  (`storage.c` `STORAGE_MOTOR_LIMIT_TORQUE_{DEFAULT,MAX}_NM`),所以默认配置和
  出厂设备是一致的。
- `hold_torque_limit_nm`:检测到接触后 `kp=kd=0` 的纯 `tau_ff` **长期**保持力矩上限,
  最大 **1.8 Nm** —— 电机的**额定(标称)力矩**;`grasp_torque_nm` 和运行时传入的
  目标力矩都不能超过它。

`start()` 会读取 V2.2 的持久化 `0x700B limit_torque`,并要求设备值不高于配置的
运动上限。这里持久化的是**夹爪 MCU Flash 中的启动配置**,不是电机自身 Flash。
第一次配置设备上限后必须物理断电重启,让 MCU 在开机时把该值写入电机运行参数
0x700B:

```python
f = t.FollowerGripper.open()
f.motor.set_startup_limit_torque(6.0)   # 写 MCU Flash,只需配置一次
print(f.motor.get_startup_limit_torque())
# 此处退出并拔插夹爪;不要在同一次上电中直接继续运动
```

重启后使用控制器:

```python
cfg = t.ForcePositionConfig()
cfg.grasp_torque_nm = 0.35
cfg.hold_torque_limit_nm = 1.8
cfg.motion_torque_limit_nm = 6.0
cfg.close_speed_radps = 0.5

f = t.FollowerGripper.open()
f.motor.clear_fault()
grasp = t.ForcePositionController(f, cfg)
grasp.start()                 # 先验证设备上限,尚不主动闭合
f.motor.enable()
try:
    grasp.set_target(0.0)     # 闭合 -> 接触 -> 0.35 Nm 纯力矩保持
    grasp.set_target(0.35, 0.45)  # 运行时改为 35% 开度、0.45 Nm
    while running:
        print(grasp.snapshot())
    grasp.release()           # 有界速度张开
finally:
    grasp.stop()              # 先下发零力矩
    f.motor.disable()
```

示例:

```bash
# 两个控制器的用法
python python/examples/impedance_control.py --side right
python python/examples/force_position_control.py --side right --grasp-torque 0.35

# 运动安全包络(固件侧的力矩与热保护,默认不启用,每台设备配一次)
python python/examples/impedance_control.py --show-envelope
python python/examples/impedance_control.py --set-envelope --peak 2.0 --cont 1.6
```

注意:6 Nm 是电机峰值、只允许运动阶段的**瞬时**力矩到这个量级,并不把夹爪机构的
安全额定值提高到 6 Nm。如果机构本身不能承受高于 1.8 Nm 的瞬时负载,应把
`motion_torque_limit_nm` 和设备 `0x700B` 一并设低;软件只能保证进入
`HoldingForce` 后的命令力矩不超过 1.8 Nm。

该控制器需要 follower 固件 ≥ 1.1.2(支持 V2.2 启动力矩上限读取),并且从爪
开合行程已经标定。运行期间它独占电机控制与状态流,不要并发使用 `ControlLoop`
或直接发送其他运动命令。

**相位为什么重要**:主机帧只要在 MCU 发送期间落地,就会让它丢掉正在发的那一帧,
整帧作废。所以碰撞取决于**落在什么时刻**,不是发了多少 —— `STREAM_LOCKED`
是每收到一帧状态提交一次,落在 MCU 已知空闲的窗口里,实测 6000 提交 : 6000 帧 :
0 丢失;同一条件下自由跑 100 Hz 每轮丢 156–308 帧。别把 500 Hz 当预算花。

低层写法(知道自己在做什么时用):

```python
f.set_position(0.5, kp_nm_per_rad=8, kd_nm_s_per_rad=1)   # 归一化,无 ACK,面向实时循环
f.motor.set_impedance(target_pos_rad=-0.5, kp_nm_per_rad=8,
                      kd_nm_s_per_rad=1, feedforward_torque_nm=0.0)   # 原始弧度,阻塞等 ACK
st = f.motor.read_status()
```

注意 `f.set_position()`(归一化 `[0,1]`)和 `f.motor.set_position()`(原始弧度)
是两个不同的东西。**反馈频率**:电机 `actual_*` 遥测只有 ~50–100 Hz,
读观测请走**流**,别用 `read_status()` 轮询 —— 超过 ~100 Hz 会拖住固件自己的刷新。

> **抓取力**不要指望位置模式的 `max_torque`,它不是紧的力上限,软物会被压坏。
> 做法见 `python/examples/gripper_force_grasp_test.py`:小步闭合 + 位置停滞检测,
> 而不是力矩阈值(夹爪本身有随开口变化的回复力矩,力矩阈值会误触发)。

可跑的示例:`impedance_control.py`、`force_position_control.py`(见上)、
`motor_mit_control.py`(裸 `submit_impedance`)、`gripper_force_grasp_test.py`
(柔性抓取)。

---

## 2. 视觉:腕部相机

腕相机是普通 UVC 设备,本 SDK 用 `Camera` 类(底层 `cv::VideoCapture`)读它。
**默认没人替你打开它** —— 两种开法:

### 2.1 独立开(推荐,和夹爪解耦)

```python
from xense.taccap import Camera, ColorMode

cam = Camera(device="/dev/video2", width=640, height=480, fps=30.0,
             use_mjpg=True, color_mode=ColorMode.BGR)

frame = cam.read(timeout_ms=500)      # 同步一次性读;失败返回 None
print(frame.frame_index, frame.image.shape)

cam.start(lambda f: handle(f.image))  # 或者异步:后台线程回调
...
cam.stop()
```

设备路径优先用 `/dev/v4l/by-id/...-video-index0` 这种稳定路径,`/dev/videoN`
的编号会随插拔顺序变。用 `python python/examples/wrist_camera.py --list` 列。

### 2.2 挂在夹爪对象上

```python
g = t.LeaderGripper(mcu_device, wrist_video="/dev/video2", open_cameras=True,
                    undistort_wrist=True)     # 帧出来就是矫正过的
g.wrist_camera.start(lambda f: print("wrist", f.frame_index))
```

> **通道顺序有个刻意的不一致**:裸 `Camera` 默认 **BGR**(OpenCV 原生,`imshow`/
> `imwrite` 直接对);夹爪的 `wrist_camera` 默认 **RGB**(喂视觉/学习管线,
> LeRobot 数据集存 RGB)。两边都能用 `color_mode` / `wrist_color_mode` 改回去。
> 搞反了不会报错,只会录进去一份通道颠倒的数据。

### 2.3 鱼眼去畸变

腕部是鱼眼镜头,内参存在 MCU flash 里(`Cmd::CameraFisheyeCal`,固件 ≥ V2.0)。
`FisheyeUndistorter` 把内参编译成一次性的 remap 表,之后每帧只做重采样:

```python
from xense.taccap import FisheyeUndistorter

cal, is_reference, reason = g.calibration.resolve_fisheye()
if is_reference:
    log.warning(f"用的是 SDK 参考内参,不是这一台的:{reason}")

undist = FisheyeUndistorter(cal, width=640, height=480, balance=0.0)
cam.set_undistorter(undist)      # 装进采集路径:read() 和回调拿到的都是矫正帧
# 或者手动:rect = undist.apply(img)
```

三个必须知道的约束:

- **读内参用 `resolve_fisheye()`,不要用 `read_fisheye()`。** 没标定过的机器
  不是 NACK,而是返回一条**全零**记录,它能过 `if cal is None` 检查,但
  `fx = fy = 0` 会把每个像素映射到画面外 —— 得到一张纯黑的"矫正帧",全程无报错。
  `resolve_fisheye()` 已经把这个策略(读 → 判可用 → 回退参考值)做在一处了。
- **只支持 640×480**,即标定分辨率。固件记录里不带图像尺寸,所以缩放内参等于
  猜,构造函数宁可抛错。要别的分辨率就得先在固件里存一份对应的标定。
- **参考内参是近似的**。每台的镜头装配位置有差异(尤其主点),回退到
  `FISHEYE_FALLBACK_CAL` 只是比完全不矫正强,不能拿它去按像素做测量。
  自己标定后用 `fisheye_cal.py set-fisheye --from-npz cam.npz` 写进 flash。

`balance` 是取景口味:0 = 保持标定焦距(自然视角,和 PC 工具默认一致),
1 = 焦距压到 0.70x 换最大视场,代价是四周更多黑边。

> 画面看起来偏心或轻微倾斜,**不一定是标定错了** —— 传感器未必正好落在镜头
> 光轴上,矫正是围绕主点做的,不是围绕画面中心。

### 2.4 一条命令看效果

```bash
python python/examples/wrist_camera.py right                 # 原始鱼眼(默认)
python python/examples/wrist_camera.py right --undistort     # 矫正
python python/examples/wrist_camera.py right --compare       # 左右对照
python python/examples/wrist_camera.py XCA28Z0116m           # 也可以直接给序列号
python python/examples/wrist_camera.py right --no-mcu --no-display \
    --duration 10 --save-dir /tmp/shots                      # 无头,存图
```

选择器就是 `left` / `right`,和其它示例一致 —— 内参会自动去**同侧**的夹爪上读。
**去畸变默认关**,和 SDK 本身一致(裸 `Camera` 不带 undistorter,夹爪的
`undistort_wrist` 默认 `False`);窗口模式下即使起在 raw 也会先把内参读好,
因为 `u` 随时可能切到矫正。无头 + raw 是唯一不会去碰夹爪的组合。
窗口里 `u` 键在 原始 / 矫正 / 对照 之间循环切,`[` `]` 调 balance,`s` 存图。
没插夹爪时加 `--no-mcu` 走参考内参;`--from-npz` 用离线标定文件。完整参数 `--help`。

> **这个脚本打不开视触觉传感器,是故意的。** 传 GSPS 序列号或 `/dev/videoN` 路径
> 都会被拒绝并说明原因 —— OG 的采集和矫正在 `xensesdk` 里,不走本 SDK(见下一节)。

---

## 3. 触觉:视触觉(OG)传感器

**这部分不在本 SDK 里。** `xense.taccap` 只覆盖夹爪协议和腕相机;OG 的采集、
矫正、力/深度推理都在 `xensesdk` wheel 里(本机版本 2.1.1)。下面给的是把它
跑起来的最短路径,接口以你装的那版 wheel 为准。

```python
from xensesdk import Sensor

print(Sensor.scanSerialNumber())     # {'OG000352': 10, 'OG000344': 8} —— 序列号: cam_id

sensor = Sensor.create("OG000352")   # 也接受 cam_id;第一次会建配置缓存,较慢
try:
    # 一次要多个输出就传多个,返回值按顺序一一对应
    rectify, depth, force = sensor.selectSensorInfo(
        Sensor.OutputType.Rectify,      # 矫正后的图像
        Sensor.OutputType.Depth,        # 深度图,单位 mm
        Sensor.OutputType.Force,        # 力分布
    )
finally:
    sensor.release()
```

常用的 `OutputType`(完整列表 `dir(Sensor.OutputType)`):

| 输出 | 含义 |
| --- | --- |
| `Rectify` | 矫正后的图像 |
| `AugDifference` / `Difference` | 与参考帧的差分图(接触可视化) |
| `Depth` | 深度图,mm |
| `Force` / `ForceNorm` / `ForceResultant` | 力分布 / 法向力 / 合力(6 维) |
| `Marker2D` / `Mesh3D` / `Mesh3DFlow` | 标记点与三维网格及其流场 |

实践上要注意的几点:

- **只要图像、不要推理时加 `disable_infer=True`**,可以省掉推理引擎的加载;
  `Rectify` 和 `Difference` 本来就不需要推理。
- **首次 `create()` 慢**是在建每序列号的配置缓存(`~/.xensesdk/config`);
  多传感器场景建议先预热缓存再并发开,否则会撞在一起。
- **OG 和腕相机都是 `/dev/video*`**,但腕相机**不在** `Sensor.scanSerialNumber()`
  的结果里 —— 它不是 OG 设备,别指望用触觉 SDK 去枚举它。反过来也一样:
  `wrist_camera.py` 会拒绝 GSPS 序列号。两边都按序列号语法把对方挡在门外。
- 力/网格类输出形状是固定的(力分布 `(35, 20, 3)`、合力 `(6,)`),
  图像类的形状随 `rectify_size` 变。

要看完整的产品级用法(异步读、配置覆盖、多传感器编排),参考
`lerobot` 侧的 `lerobot/cameras/xense/camera_xense.py`。

---

## 4. 三路一起跑

各自独立,但同时开有几个已知的坑:

- **USB 带宽是共享的。** 两个 OG + 一个腕相机同时 MJPG 满帧,已经吃掉相当一部分
  总线;夹爪走的是串口,不受影响,但相机之间会互相挤。真要满配采集,
  先测一遍实际帧率,别假设标称值。
- **谁持有 UVC 设备。** 生产环境里外部相机服务持有 `/dev/video*`,这时
  **不要**再用 `open_cameras=True` 或裸 `Camera` 去开同一个节点 —— 会失败或抢到
  半路。SDK 默认不开相机,就是为了这个。
- **控制回路要用 `ControlLoop`(`STREAM_LOCKED`)。** 上面的相位说明在满负载下
  尤其重要:实测就是在"所有相机都在流 + 电机在往复"的条件下做的。
- **日志。** 全 SDK 一个单例 logger(`xense.taccap.log`),控制台默认 INFO,
  文件 sink 恒为 DEBUG,落在 `~/.taccaplogs/session_*.log`(可用 `$TACCAP_LOG_DIR`
  改),最多留 10 份。出问题先翻这个文件,里面有控制台被过滤掉的那些行。
- **固件刷完必须断电重插。** bank-swap 之后是软复位,设备看起来完全正常
  (版本对、流在跑、计数干净),但会静默丢状态帧。见 [docs/FIRMWARE.md](FIRMWARE.md)。

---

## 出问题时

| 现象 | 多半是 |
| --- | --- |
| `scan_grippers()` 返回空 | 串口权限(见 [INSTALL.md](INSTALL.md))或线没插好 |
| `fw_sn` 为空 / `Side.Unknown` | 固件没烧 SN,或固件早于 V1.6 |
| 矫正后画面全黑 | 用了 `read_fisheye()` 的全零记录 —— 改用 `resolve_fisheye()` |
| 矫正后画面偏心 | 未必是错的,主点本来就不一定在画面中心 |
| 录下来的颜色是反的 | 裸 `Camera` 是 BGR、`wrist_camera` 是 RGB,搞混了 |
| 电机状态帧成片丢失 | 提交相位(用 `ControlLoop`),或刷完固件没断电重插 |
| 改了 C++/bindings 但 Python 里没生效 | 消费端的环境没重装,见 [INSTALL.md](INSTALL.md) |
| `import xense.taccap` 报 `libopencv_core.so` | 缺 `LD_LIBRARY_PATH=$CONDA_PREFIX/lib` |

更细的专题:[INSTALL.md](INSTALL.md) · [CALIBRATION.md](CALIBRATION.md) ·
[FIRMWARE.md](FIRMWARE.md) · [EXAMPLES.md](EXAMPLES.md) ·
[ARCHITECTURE.md](ARCHITECTURE.md)
