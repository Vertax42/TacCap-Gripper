#!/usr/bin/env python3
# Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
"""阻抗控制抓取(非交互),并演示运动安全包络的配置与效果。

阻抗控制走 ControlLoop:后台线程按状态流相位提交 MIT 帧,策略侧只碰
set_target(0..1) 和 observation()。

为什么需要包络
--------------
被物体挡住时,kp x 位置误差没有上界。实测(EL05 / 24V / kp=20,硬物体挡在
约 0.6 rad 处):命令索要约 12 Nm,被电机 0x700B 削到 6 Nm,夹爪冲到
8.68 rad/s 撞上去,力矩 20ms 内从 0.03 爬到 2.41 Nm 且仍在上升 —— 24V 下这个
电流需求会把母线拉垮,电机欠压保护动作、松手,严重时整机掉电重启。

包络在固件里每个控制周期钳位,而且钳的是所有 MIT 命令的必经点,连绕过本 SDK
直接调 motor.submit_impedance() 也挡得住。开启后同样条件下力矩恒定在 peak 以内。

用法
    python python/examples/impedance_grasp.py --side right
    python python/examples/impedance_grasp.py --set-envelope --peak 2.0 --cont 1.6
    python python/examples/impedance_grasp.py --show-envelope

安全:真实运动。请放软的或可牺牲的物体。脚本带力矩/温度看门狗,退出时必定
下发零力矩并 disable。
"""
from __future__ import annotations

import argparse
import time

from xense.taccap import (
    ControlLoop, FollowerGripper, GripperEnvelope, StallAction,
    GRIPPER_ENVELOPE_VALID, GRIPPER_ENVELOPE_ENFORCE,
    find_follower, find_left, find_right, log,
)


def open_gripper(side: str) -> FollowerGripper:
    eps = {"left": find_left, "right": find_right}.get(side, find_follower)()
    print(f"[discovery] {eps.firmware_sn}  {eps.mcu_device}")
    return FollowerGripper(mcu_device=eps.mcu_device)


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--side", default="follower", choices=("left", "right", "follower"))
    ap.add_argument("--kp", type=float, default=8.0, help="阻抗刚度 Nm/rad")
    ap.add_argument("--kd", type=float, default=1.0, help="阻抗阻尼 Nm·s/rad")
    ap.add_argument("--target", type=float, default=0.0, help="抓取目标开度 0..1")
    ap.add_argument("--hold", type=float, default=15.0, help="保持时长 s")
    ap.add_argument("--abort-torque", type=float, default=2.5, help="看门狗力矩 Nm")
    ap.add_argument("--abort-temp", type=float, default=85.0, help="看门狗温度 °C")
    # ---- 运动安全包络 ----
    ap.add_argument("--show-envelope", action="store_true", help="打印包络后退出")
    ap.add_argument("--set-envelope", action="store_true", help="写入包络后继续")
    ap.add_argument("--peak", type=float, default=2.0, help="运动瞬态力矩上限 Nm")
    ap.add_argument("--cont", type=float, default=1.6, help="可持续力矩上限 Nm")
    ap.add_argument("--temp-derate-start", type=int, default=0, help="降额起点 °C,0=固件默认 90")
    ap.add_argument("--temp-wall", type=int, default=0, help="温度墙 °C,0=固件默认 100")
    args = ap.parse_args()

    log.set_level("warn")
    g = open_gripper(args.side)
    print(f"[fw] {g.firmware_version}")

    if args.set_envelope:
        e = g.get_envelope()
        e.cont_torque_nm, e.peak_torque_nm = args.cont, args.peak
        e.temp_derate_start_c, e.temp_wall_c = args.temp_derate_start, args.temp_wall
        e.flags = GRIPPER_ENVELOPE_VALID | GRIPPER_ENVELOPE_ENFORCE
        g.set_envelope(e)
    env = g.get_envelope()
    print(f"[envelope] {env}")
    if not (env.flags & GRIPPER_ENVELOPE_ENFORCE):
        print("[warn] 包络未启用 —— 被挡住时 kp*误差 没有上界。"
              "用 --set-envelope 写入,或 set_envelope() 显式开启。")
    if args.show_envelope:
        return 0

    loop = ControlLoop(g, kp=args.kp, kd=args.kd,
                       stall_action=StallAction.HOLD_POSITION)
    t0_temp = g.motor.read_status().motor_temp_c
    try:
        g.motor.clear_fault()
        loop.start()
        g.motor.enable()

        loop.set_target(1.0)
        time.sleep(2.5)
        print(f"[open]  pos={loop.observation().position:.4f}")

        print(f"[grasp] target={args.target:.2f}  保持 {args.hold:.0f}s")
        loop.set_target(args.target)
        t0 = time.perf_counter()
        peak_v = peak_q = 0.0
        while time.perf_counter() - t0 < args.hold:
            o = loop.observation()
            peak_v = max(peak_v, abs(o.velocity))
            peak_q = max(peak_q, abs(o.torque))
            el = time.perf_counter() - t0
            if abs(o.torque) > args.abort_torque:
                print(f"  !! 看门狗:力矩 {o.torque:+.3f} Nm @ t={el:.1f}s")
                break
            if int(el) and int(el) % 3 == 0:
                # read_status() 会和控制帧的 ACK 对撞,所以温度低频读
                st = g.motor.read_status()
                print(f"  t={el:4.1f}s pos={o.position:.4f} tq={o.torque:+.3f} "
                      f"temp={st.motor_temp_c:.0f}°C stalled={loop.stalled}")
                if st.motor_temp_c > args.abort_temp:
                    print(f"  !! 看门狗:温度 {st.motor_temp_c:.0f}°C")
                    break
                time.sleep(1.0)
            time.sleep(0.02)

        o = loop.observation()
        st = g.motor.read_status()
        print(f"\n[result] pos={o.position:.4f} tq={o.torque:+.3f} Nm  "
              f"峰值 vel={peak_v:.2f} rad/s tq={peak_q:.3f} Nm")
        print(f"         温度 {t0_temp:.0f} -> {st.motor_temp_c:.0f}°C  "
              f"堵转保护触发 {loop.stall_trips} 次")
        loop.set_target(1.0)
        time.sleep(2.0)
    finally:
        try:
            loop.stop()          # stop() 先下发零力矩
        except Exception as exc:
            print("stop:", exc)
        try:
            g.motor.disable()
        except Exception as exc:
            print("disable:", exc)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
