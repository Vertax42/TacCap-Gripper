#!/usr/bin/env python3
# Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
"""
单个 follower 夹爪的安全键盘控制台。

旧版直接发送位置阻抗命令。硬物越大，越早挡住夹爪，目标与实际位置的永久误差
越大，kp * error 会继续增长到数 Nm。本版改用 ForcePositionController：有界速度
闭合，检测接触后切换到 kp=kd=0 的纯力矩保持，夹持力由 --grasp-torque 指定。

用法:
    python python/examples/gripper_console.py --kp 20 --kd 1

按键:
    j / k   — 点动 − / +（每次从当前位置移动一个 --step，不累积旧目标）
    o / c   — 到标定的全开 / 全闭端
    e / d   — 以当前位置重新使能 / 保持当前位置后失能
    f       — 清故障并以当前位置重置控制器
    q / ESC — 零力矩、失能并退出

运行前必须启用 follower 1.1.6 的固件安全包络：
    python python/examples/impedance_control.py --set-envelope --show-envelope --peak 2.0 --cont 1.6

安全提示：真实运动 + 夹持力。退出、实测超力矩和电机故障都会停止控制并失能。
"""

from __future__ import annotations

import argparse
import fcntl
import os
import sys
import termios
import time
import tty
from typing import Optional

from xense.taccap import (
    FollowerGripper,
    ForcePositionConfig,
    ForcePositionController,
    GRIPPER_ENVELOPE_ENFORCE,
    find_left,
    find_right,
)


# ── 状态位（与 TaccapMotorState.msg 一致） ────────────────────────────────────

_STATUS_BITS = [
    (1,   'EN'),
    (2,   'FAULT'),
    (4,   'STALL'),
    (8,   'OVER_TEMP'),
    (16,  'OVER_CURR'),
    (32,  'OVER_VOLT'),
    (64,  'UNDER_VOLT'),
    (128, 'ENC_ERR'),
    (256, 'DRIVER_FAULT'),
    (512, 'POS_INIT_ERR'),
    (1024, 'HW_ID_ERR'),
    (2048, 'ENC_UNCAL'),
]

_SERIOUS_STATUS_MASK = 2 | 8 | 16 | 32 | 64 | 128 | 256 | 512 | 1024 | 2048


def _status_str(mask) -> str:
    if mask is None:
        return '-'
    if mask == 0:
        return 'DISABLED'
    bits = [name for bit, name in _STATUS_BITS if mask & bit]
    return '|'.join(bits) if bits else f'0x{mask:04X}'


# ── 键盘原始模式 ──────────────────────────────────────────────────────────────

class RawKeyboard:
    """将 stdin 切换到 non-blocking 原始模式，逐字节 poll。"""

    def __enter__(self):
        self._saved = termios.tcgetattr(sys.stdin.fileno())
        tty.setraw(sys.stdin.fileno())
        flags = fcntl.fcntl(sys.stdin.fileno(), fcntl.F_GETFL)
        fcntl.fcntl(sys.stdin.fileno(), fcntl.F_SETFL, flags | os.O_NONBLOCK)
        return self

    def __exit__(self, *_):
        termios.tcsetattr(sys.stdin.fileno(), termios.TCSANOW, self._saved)
        flags = fcntl.fcntl(sys.stdin.fileno(), fcntl.F_GETFL)
        fcntl.fcntl(sys.stdin.fileno(), fcntl.F_SETFL, flags & ~os.O_NONBLOCK)

    @staticmethod
    def poll() -> Optional[str]:
        try:
            b = os.read(sys.stdin.fileno(), 1)
            return b.decode('utf-8', errors='replace') if b else None
        except (BlockingIOError, InterruptedError):
            return None


# ── UI ────────────────────────────────────────────────────────────────────────

def _redraw(side_name: str, target_rad: float,
            actual_rad: float, vel: float, torque: float,
            norm_pos: Optional[float], status: int,
            submit_hz: float, stream_hz: float,
            commanded_torque: float, controller_state: str,
            last_key: str, kp: float, kd: float,
            grasp_torque: float, abort_torque: float,
            pos_min: float, pos_max: float) -> None:
    norm_str = f'{norm_pos:.3f}' if norm_pos is not None else '  N/A'
    lines = [
        f'=== Gripper Console [{side_name.upper()}]'
        f'  kp={kp:.2f} kd={kd:.2f} grasp={grasp_torque:.2f}Nm'
        f' abort={abort_torque:.2f}Nm cmd={submit_hz:.0f}Hz rx={stream_hz:.0f}Hz'
        f' range=[{pos_min:.2f}, {pos_max:.2f}] rad ===',
        '  j/k=点动-/+  o=open  c=close  e=enable  d=disable  f=fault_clear  q=quit',
        f'{"Tgt(rad)":>10}{"Act(rad)":>10}{"Norm[0-1]":>11}'
        f'{"Vel(r/s)":>10}{"Torq(Nm)":>10}{"Cmd(Nm)":>10}{"State":>28}',
        '-' * 99,
        f'{target_rad:>10.3f}{actual_rad:>10.3f}{norm_str:>11}'
        f'{vel:>10.3f}{torque:>10.3f}{commanded_torque:>10.3f}'
        f'{controller_state + "/" + _status_str(status):>28}',
        f'  last key: {last_key}',
    ]
    out = '\033[H' + '\r\n'.join(line + '\033[K' for line in lines)
    sys.stdout.write(out)
    sys.stdout.flush()


def _point_target(controller: ForcePositionController, pos_map,
                  actual_rad: float, step: float, delta: float,
                  pos_min: float, pos_max: float):
    """Create one point-to-point target from the latest actual position.

    Keyboard control is intentionally jog/point control: every j/k press moves
    one step from where the jaw is now, instead of accumulating a queue of
    stale targets while the motor is still travelling.
    """
    target = max(pos_min, min(pos_max, actual_rad + delta * step))
    controller.set_target(pos_map.to_position(target))
    return target


# ── main ──────────────────────────────────────────────────────────────────────

def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--side', choices=['left', 'right', 'auto'], default='auto',
                    help='要打开的夹爪（默认 auto 自动发现）')
    ap.add_argument('--kp', type=float, default=20.0,
                    help='有界位置保持刚度 Nm/rad（默认 20）')
    ap.add_argument('--kd', type=float, default=1.0,
                    help='有界位置保持阻尼 Nm·s/rad（默认 1）')
    ap.add_argument('--hz', type=float, default=100.0,
                    help='电机状态流频率 Hz（默认 100，最大 100）')
    ap.add_argument('--step', type=float, default=0.05,
                    help='j/k 点动步长（rad，默认 0.05；每次从实际位置计算）')
    ap.add_argument('--pos-min', type=float, default=None, dest='pos_min',
                    help='raw rad 下限（默认本机标定下限）')
    ap.add_argument('--pos-max', type=float, default=None, dest='pos_max',
                    help='raw rad 上限（默认本机标定上限）')
    ap.add_argument('--grasp-torque', type=float, default=0.8,
                    help='接触后纯力矩保持值 Nm（默认 0.8）')
    ap.add_argument('--abort-torque', type=float, default=2.2,
                    help='实测力矩持续超过此值就零力矩并失能 Nm（默认 2.2）')
    ap.add_argument('--abort-samples', type=int, default=3,
                    help='连续多少帧超力矩才失能（默认 3，100 Hz 时约 30 ms）')
    ap.add_argument('--close-speed', type=float, default=0.5,
                    help='有界开合速度 rad/s（默认 0.5）')
    ap.add_argument('--allow-unprotected', action='store_true',
                    help='允许未启用固件包络时运行（仅受控诊断）')
    args = ap.parse_args()

    if not 0.0 < args.hz <= 100.0:
        ap.error('--hz must be in (0, 100]')
    if args.step <= 0.0:
        ap.error('--step must be > 0')
    if not 0.08 <= args.grasp_torque <= 1.8:
        ap.error('--grasp-torque must be in [0.08, 1.8] Nm')
    if args.abort_torque <= 0.0:
        ap.error('--abort-torque must be > 0 Nm')
    if args.abort_samples <= 0:
        ap.error('--abort-samples must be > 0')
    if args.close_speed <= 0.0:
        ap.error('--close-speed must be > 0')
    if args.abort_torque < args.grasp_torque:
        ap.error('--abort-torque must be >= --grasp-torque')

    # ── 打开夹爪 ──
    if args.side == 'left':
        ep = find_left()
        g = FollowerGripper(ep.mcu_device)
        side_name = 'left'
    elif args.side == 'right':
        ep = find_right()
        g = FollowerGripper(ep.mcu_device)
        side_name = 'right'
    else:
        g = FollowerGripper.open()
        side_name = 'auto'

    motor = g.motor

    fw = g.firmware_version
    if fw is None or fw.tuple < (1, 1, 6):
        print(f'[error] follower firmware 1.1.6+ required, got {fw}')
        return 2

    try:
        pos_map = g.position_map()
    except Exception as exc:
        print(f'[error] gripper not calibrated: {exc}')
        return 2

    open_rad = pos_map.to_rad(1.0)
    close_rad = pos_map.to_rad(0.0)
    cal_min, cal_max = sorted((open_rad, close_rad))
    pos_min = cal_min if args.pos_min is None else max(cal_min, min(cal_max, args.pos_min))
    pos_max = cal_max if args.pos_max is None else max(cal_min, min(cal_max, args.pos_max))
    if pos_min >= pos_max:
        print('[error] requested range is outside the calibrated travel')
        return 2
    open_rad = max(pos_min, min(pos_max, open_rad))
    close_rad = max(pos_min, min(pos_max, close_rad))

    try:
        env = g.get_envelope()
        envelope_on = bool(env.flags & GRIPPER_ENVELOPE_ENFORCE)
    except Exception as exc:
        envelope_on = False
        print(f'[warn] cannot read firmware safety envelope: {exc}')
    if not envelope_on:
        print('[warn] firmware safety envelope is disabled; configure once with:')
        print('  python python/examples/impedance_control.py '
              '--set-envelope --show-envelope --peak 2.0 --cont 1.6')
        if not args.allow_unprotected:
            print('[error] refusing motion without the firmware backstop; '
                  'use --allow-unprotected only for controlled diagnostics')
            return 2

    cfg = ForcePositionConfig()
    cfg.position_kp = args.kp
    cfg.position_kd = args.kd
    cfg.close_speed_radps = args.close_speed
    cfg.grasp_torque_nm = args.grasp_torque
    cfg.motor_stream_hz = int(args.hz)
    controller = ForcePositionController(g, cfg)

    target_rad = close_rad
    stream_hz = 0.0
    rate_t0 = time.monotonic()
    rate_seq = 0
    last_key = '-'
    abort_reason = None
    over_torque_count = 0

    try:
        motor.clear_fault()
        controller.start()
        motor.enable()

        initial = controller.snapshot()
        if initial.observation.valid:
            target_rad = max(pos_min, min(pos_max, initial.observation.raw_pos))
            rate_seq = initial.observation.seq
        rate_t0 = time.monotonic()

        sys.stdout.write('\033[2J')
        sys.stdout.flush()
        with RawKeyboard():
            while True:
                ch = RawKeyboard.poll()
                if ch is not None:
                    last_key = repr(ch)

                if ch == 'j':
                    s = controller.snapshot()
                    base = s.observation.raw_pos if s.observation.valid else target_rad
                    target_rad = _point_target(controller, pos_map, base, args.step,
                                               -1.0, pos_min, pos_max)
                elif ch == 'k':
                    s = controller.snapshot()
                    base = s.observation.raw_pos if s.observation.valid else target_rad
                    target_rad = _point_target(controller, pos_map, base, args.step,
                                               1.0, pos_min, pos_max)
                elif ch == 'o':
                    target_rad = open_rad
                    controller.set_target(pos_map.to_position(open_rad))
                elif ch == 'c':
                    target_rad = close_rad
                    controller.set_target(pos_map.to_position(close_rad))
                elif ch == 'e':
                    s = controller.snapshot()
                    controller.hold_position()
                    if s.observation.valid:
                        target_rad = s.observation.raw_pos
                    motor.enable()
                elif ch == 'd':
                    s = controller.snapshot()
                    controller.hold_position()
                    if s.observation.valid:
                        target_rad = s.observation.raw_pos
                    motor.disable()
                elif ch == 'f':
                    motor.clear_fault()
                    controller.reset()
                    s = controller.snapshot()
                    if s.observation.valid:
                        target_rad = s.observation.raw_pos
                elif ch in ('q', '\x1b', '\x03'):
                    break

                s = controller.snapshot()
                obs = s.observation
                state_name = str(s.state).split('.')[-1].upper()
                if state_name == 'FAULT':
                    abort_reason = s.fault_reason or 'controller fault'
                    break
                if obs.status & _SERIOUS_STATUS_MASK:
                    abort_reason = f'motor status {_status_str(obs.status)}'
                    break
                if obs.valid and abs(obs.torque) > args.abort_torque:
                    over_torque_count += 1
                else:
                    over_torque_count = 0
                if over_torque_count >= args.abort_samples:
                    abort_reason = (f'feedback torque {abs(obs.torque):.3f} Nm exceeded '
                                    f'--abort-torque {args.abort_torque:.3f} Nm for '
                                    f'{over_torque_count} consecutive samples')
                    break

                now = time.monotonic()
                rate_dt = now - rate_t0
                if rate_dt >= 1.0:
                    stream_hz = (obs.seq - rate_seq) / rate_dt
                    rate_seq = obs.seq
                    rate_t0 = now
                _redraw(side_name, target_rad,
                        obs.raw_pos, obs.velocity, obs.torque,
                        obs.position if obs.valid else None, obs.status,
                        stream_hz, stream_hz,
                        s.commanded_torque_nm, state_name, last_key,
                        args.kp, args.kd,
                        args.grasp_torque, args.abort_torque, pos_min, pos_max)
                time.sleep(0.01)
    finally:
        try:
            controller.stop()
        finally:
            try:
                motor.disable()
            except Exception:
                pass
        if abort_reason:
            sys.stdout.write(f'\r\n[abort] {abort_reason}\r\n')
        sys.stdout.write('[exit] controller stopped, motor disabled\r\n')
        sys.stdout.flush()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
