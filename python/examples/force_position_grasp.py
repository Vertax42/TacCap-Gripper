#!/usr/bin/env python3
# Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
"""力位混合抓取(非交互)。交互版见 force_position_console.py。

和纯阻抗的区别
--------------
阻抗用 kp x 位置误差生成力矩,被挡住时误差不消失,力矩就一直在。力位混合改成:

    kp=0 的速度阻尼闭合  ->  接触判定  ->  kp=kd=0 的纯 tau_ff 保持

进入 HoldingForce 后 kp 和 kd 都是 0,位置误差在结构上无法再对输出力矩做贡献。

接触判定照搬固件开机自标定那套(task_canmotor_is_stalled):**力矩下限 且
运动停止 且 连续确认**,不是裸力矩阈值。做分离的是速度门不是力矩数字 ——
实测整段空载闭合,自由行程 |vel| 始终 >= 0.183 rad/s(0.035 门限的五倍)而
|力矩| <= 0.142 Nm,机械止点 |vel| ~ 0.012、|力矩| ~ 0.21,没有任何一帧同时
满足两个条件。

握持力会随温度衰减
------------------
固件的 I2t 与温度墙对纯力矩保持同样生效。实测(墙 90/100,cont=1.6):
91 °C 起降额,94 °C / 1.370 Nm 平衡。上层 grasp_torque_nm 仍是设定值,
固件在下面把实际输出降下来 —— 所以**不要假设握持力是常数**。

用法
    python python/examples/force_position_grasp.py --side right
    python python/examples/force_position_grasp.py --grasp-torque 0.5 --hold 30

安全:真实运动 + 夹持力。请放软的或可牺牲的物体。
"""
from __future__ import annotations

import argparse
import time

from xense.taccap import (
    FollowerGripper, ForcePositionConfig, ForcePositionController,
    GRIPPER_ENVELOPE_ENFORCE, find_follower, find_left, find_right, log,
)


def open_gripper(side: str) -> FollowerGripper:
    eps = {"left": find_left, "right": find_right}.get(side, find_follower)()
    print(f"[discovery] {eps.firmware_sn}  {eps.mcu_device}")
    return FollowerGripper(mcu_device=eps.mcu_device)


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--side", default="follower", choices=("left", "right", "follower"))
    ap.add_argument("--grasp-torque", type=float, default=0.35,
                    help="接触后的纯前馈保持力矩 Nm(同时是接触判定的力矩上界)")
    ap.add_argument("--close-speed", type=float, default=0.5, help="闭合速度 rad/s")
    ap.add_argument("--contact-torque", type=float, default=0.080,
                    help="接触力矩下限 Nm,固件同名常数的默认值")
    ap.add_argument("--target", type=float, default=0.0, help="闭合目标开度 0..1")
    ap.add_argument("--hold", type=float, default=15.0, help="保持时长 s")
    ap.add_argument("--abort-temp", type=float, default=85.0, help="看门狗温度 °C")
    args = ap.parse_args()

    log.set_level("warn")
    g = open_gripper(args.side)
    print(f"[fw] {g.firmware_version}")
    env = g.get_envelope()
    print(f"[envelope] {env}")
    if not (env.flags & GRIPPER_ENVELOPE_ENFORCE):
        print("[warn] 包络未启用 —— 固件侧的 I2t 与温度墙不会生效,"
              "长时间夹持没有热保护。见 impedance_grasp.py --set-envelope")

    cfg = ForcePositionConfig()
    cfg.grasp_torque_nm = args.grasp_torque
    cfg.close_speed_radps = args.close_speed
    cfg.contact_torque_nm = args.contact_torque
    cfg.close_position = args.target

    c = ForcePositionController(g, cfg)
    t0_temp = g.motor.read_status().motor_temp_c
    try:
        g.motor.clear_fault()
        c.start()               # start() 会校验设备持久化的 0x700B 启动上限
        g.motor.enable()

        c.set_target(1.0)
        time.sleep(2.5)
        print(f"[open]  pos={c.snapshot().observation.position:.4f}")

        print(f"[grasp] target={args.target:.2f} torque={args.grasp_torque:.2f} Nm")
        c.set_target(args.target)
        t0 = time.perf_counter()
        latched = None
        while time.perf_counter() - t0 < 8.0:
            s = c.snapshot()
            st = str(s.state).split(".")[-1]
            if st in ("HOLDING_FORCE", "HOLDING_POSITION", "FAULT"):
                latched = (st, s.observation.position, time.perf_counter() - t0)
                break
            time.sleep(0.01)
        if latched:
            print(f"  -> {latched[0]} @ pos={latched[1]:.4f}  用时 {latched[2]:.2f}s")
        else:
            print("  -> 8s 内未收敛,仍在运动")

        print(f"[hold]  保持 {args.hold:.0f}s,观察热降额")
        t0 = time.perf_counter()
        while time.perf_counter() - t0 < args.hold:
            time.sleep(3.0)
            s = c.snapshot()
            st = g.motor.read_status()
            el = time.perf_counter() - t0
            print(f"  t={el:4.1f}s state={str(s.state).split('.')[-1]:15s} "
                  f"cmd={s.commanded_torque_nm:.3f} 实测={s.observation.torque:+.3f} Nm "
                  f"temp={st.motor_temp_c:.0f}°C")
            if st.motor_temp_c > args.abort_temp:
                print(f"  !! 看门狗:温度 {st.motor_temp_c:.0f}°C")
                break

        s = c.snapshot()
        st = g.motor.read_status()
        print(f"\n[result] state={s.state} pos={s.observation.position:.4f} "
              f"cmd={s.commanded_torque_nm:.3f} 实测={s.observation.torque:+.3f} Nm")
        print(f"         温度 {t0_temp:.0f} -> {st.motor_temp_c:.0f}°C"
              + ("  (cmd 与实测的差就是固件的热降额)"
                 if abs(s.commanded_torque_nm - abs(s.observation.torque)) > 0.1 else ""))
        c.set_target(1.0)
        time.sleep(2.0)
    finally:
        try:
            c.stop()             # stop() 先下发零力矩
        except Exception as exc:
            print("stop:", exc)
        try:
            g.motor.disable()
        except Exception as exc:
            print("disable:", exc)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
