# `gripper_console.py` 实现与修改说明

## 对照基线

本文档以远端 `origin/main` 的 `3d44440` 为对照基线。该版本已经提供：

- `xense.taccap.ForcePositionController` SDK 控制器；
- `python/examples/force_position_control.py` 非交互式力位控制示例；
- `python/examples/motor_mit_control.py` MIT 原语示例。

基线中没有终端交互式 `gripper_console.py`。本次提交新增该示例，保持客户现有
命令行习惯和按键操作方式。

## 实现说明

### SDK 使用边界

脚本使用 `xense.taccap` SDK 完成设备访问和 MIT 指令传输，不自行实现串口协议、
帧编码或电机状态解码。使用的 SDK 接口包括：

```python
FollowerGripper
find_follower / find_left / find_right
motor.get_protocol()
motor.read_status()
motor.on_status()
g.start_streaming() / g.stop_streaming()
motor.submit_impedance()
g.get_envelope() / g.set_envelope()
```

脚本中的 `MitForcePositionController` 是交互示例自己的控制状态机，不是 SDK
内置 `ForcePositionController` 的另一个别名。这样可以在不改变 SDK 公共 API 的
情况下提供点动、键盘超时和终端状态显示。

### 控制状态机

控制器维护以下状态：

- `pos_hold`：以当前开度发送零前馈位置保持；
- `closing`：MIT 速度阻尼闭合，接近目标后切换到位置制动；
- `holding`：接触确认后以 `kp=kd=0` 的纯前馈力矩保持；
- `releasing`：MIT 速度阻尼张开，结束后回到位置保持；
- `fault`：状态流超时、严重电机故障或指令发送连续失败时停止输出。

闭合接触判定包含启动屏蔽、力矩阈值和连续状态确认，避免启动加速阶段的阻尼
力矩被误判为接触。接触后保持阶段只发送 `tau_ff`，位置误差不会继续增加保持
力矩。

### 指令发送与按键响应

终端输入由独立线程持续读取并放入队列，控制线程不再阻塞等待 stdin。每个 `j/k`
字符会改变一个角度增量；终端自动重复字符可实现长按连续运动。收到按键后立即
发送当前阶段指令，活动阶段按固定周期重发，写入异常立即重试一次。

`submit_impedance()` 是 SDK 提供的无 ACK 快速发送接口，因此周期重发用于覆盖
偶发的串行帧丢失；它不把一次成功写入误认为设备已经执行，也不替代状态流超时
保护。

### 力矩、速度和包络参数

```text
--grasp-torque       接触后的目标保持力矩
--hold-torque-offset 保持阶段固定偏置补偿，默认 0.05 Nm
--close-speed        闭合速度，rad/s
--release-speed      张开速度，rad/s
--step-rad           j/k 单次角度增量，rad
--contact-torque     接触判定下限
--force-limit        主机侧力矩保护上限
--peak / --cont      固件运动包络的峰值/持续力矩
```

保持偏置只作用于接触后的 `holding` 状态。例如目标 `1.20 Nm`、默认偏置
`0.05 Nm` 时，下发保持前馈力矩为 `1.15 Nm`。闭合速度阶段的阻尼力矩不使用该
偏置，避免改变闭合速度和接触判定。

## 修改说明

相对于远端最新版本，本次提交包含：

1. 新增 `python/examples/gripper_console.py`，提供 MIT-only 终端交互控制。
2. 新增 `j/k` 点动、长按跟随、`o/c/h/e/d/f/q` 操作和实时状态显示。
3. 增加独立键盘读取线程、按键即时下发、活动指令周期重发和一次失败重试。
4. 增加 MIT 协议检查、状态流新鲜度检查、严重故障保护和安全退出零力矩流程。
5. 增加闭合速度、张开速度、点动步长、接触阈值、力矩限制等命令行参数。
6. 增加最终保持力矩的可调固定偏置补偿，默认修正约 `0.05 Nm` 的整体偏高。
7. 增加 `--set-envelope` 及包络参数配置入口，并在界面显示当前包络和控制参数。
8. 更新 `docs/EXAMPLES.md`，使新脚本可从示例索引发现。

## 与 SDK 内置控制器的关系

SDK 的 `ForcePositionController` 仍然是应用程序直接调用控制功能的推荐接口；它
拥有自己的状态流和控制线程，适合非交互式程序。本示例为了兼容客户的终端按键
习惯，暂时在脚本层实现交互状态机，并复用 SDK 的设备、状态和 MIT 发送接口。

后续如果需要统一行为，可将本示例中已验证的保持偏置、独立张开速度和点动接口
下沉到 SDK，再让本脚本只保留键盘和显示层。

## 使用示例

```bash
python python/examples/gripper_console.py \\
  --mode force-position \\
  --grasp-torque 1.2 \\
  --close-speed 0.30 \\
  --release-speed 0.80
```

首次使用建议先检查包络：

```bash
python python/examples/gripper_console.py --show-envelope
```

这是实际电机运动和夹持力控制示例，必须确认夹爪周围无危险障碍物，并在合适的
供电、固件和包络配置下运行。

## 验证记录

- 使用项目 Python 环境通过 `py_compile` 语法检查；
- 使用离线伪电机验证 `1.20 - 0.05 = 1.15 Nm` 的保持力矩下发值；
- 未在本次提交中伪造硬件测试结果，实际夹爪仍需在目标设备上验证速度、接触和
  保持力矩。
