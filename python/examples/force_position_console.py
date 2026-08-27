#!/usr/bin/env python3
# Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
"""Interactive force-position hybrid grasp test for one follower gripper.

Unlike gripper_console.py, this tool never closes a blocked jaw with a fixed
position error. It approaches with velocity damping, detects contact, then
switches to a pure feed-forward torque hold (kp=kd=0).

Keys:
  g       move to the selected target/contact/force-hold
  t       enter an exact normalized target and optional grasp torque
  o       open the gripper with bounded velocity damping
  h       cancel force/motion and hold the current position
  e / d   enable / disable the motor
  f       clear fault, reset the controller at the current position, enable
  q/ESC   stop, zero the controller command, disable, and quit

The controller separates a 6 Nm motion/device limit from a 1.8 Nm force-hold
limit. It refuses to start if the persisted motor startup limit exceeds the
configured motion limit. Use --set-device-limit once, then physically
power-cycle the gripper before running a motion test.
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
    FORCE_POSITION_MAX_HOLD_TORQUE_NM,
    FORCE_POSITION_MAX_MOTION_TORQUE_NM,
    FollowerGripper,
    ForcePositionConfig,
    ForcePositionController,
    find_left,
    find_right,
    log,
)


MAX_HOLD_TORQUE_NM = float(FORCE_POSITION_MAX_HOLD_TORQUE_NM)
MAX_MOTION_TORQUE_NM = float(FORCE_POSITION_MAX_MOTION_TORQUE_NM)

_STATUS_BITS = [
    (0x0001, "EN"),
    (0x0002, "FAULT"),
    (0x0004, "STALL"),
    (0x0008, "OVER_TEMP"),
    (0x0010, "OVER_CURR"),
    (0x0020, "OVER_VOLT"),
    (0x0040, "UNDER_VOLT"),
    (0x0080, "ENC_ERR"),
    (0x0100, "DRIVER_FAULT"),
    (0x0200, "POS_INIT_ERR"),
    (0x0400, "HW_ID_ERR"),
    (0x0800, "ENC_UNCAL"),
]


def _status_str(mask: int) -> str:
    if mask == 0:
        return "DISABLED"
    names = [name for bit, name in _STATUS_BITS if mask & bit]
    known = sum(bit for bit, _ in _STATUS_BITS)
    unknown = mask & ~known
    if unknown:
        names.append(f"0x{unknown:04X}")
    return "|".join(names) if names else f"0x{mask:04X}"


class RawKeyboard:
    def __enter__(self):
        self._fd = sys.stdin.fileno()
        self._saved = termios.tcgetattr(self._fd)
        self._saved_flags = fcntl.fcntl(self._fd, fcntl.F_GETFL)
        self._active = False
        self.resume()
        return self

    def __exit__(self, *_):
        self.suspend()

    def resume(self) -> None:
        if self._active:
            return
        tty.setraw(self._fd)
        fcntl.fcntl(self._fd, fcntl.F_SETFL,
                    self._saved_flags | os.O_NONBLOCK)
        self._active = True

    def suspend(self) -> None:
        if not self._active:
            return
        termios.tcsetattr(self._fd, termios.TCSANOW, self._saved)
        fcntl.fcntl(self._fd, fcntl.F_SETFL, self._saved_flags)
        self._active = False

    @staticmethod
    def poll() -> Optional[str]:
        try:
            data = os.read(sys.stdin.fileno(), 1)
            return data.decode("utf-8", errors="replace") if data else None
        except (BlockingIOError, InterruptedError):
            return None


def _open_gripper(side: str) -> FollowerGripper:
    if side == "left":
        return FollowerGripper(find_left().mcu_device)
    if side == "right":
        return FollowerGripper(find_right().mcu_device)
    return FollowerGripper.open()


def _state_name(state) -> str:
    text = str(state)
    return text.rsplit(".", 1)[-1].lower()


def _prompt_target(keyboard: RawKeyboard, current_position: float,
                   current_torque: float, max_torque: float):
    keyboard.suspend()
    try:
        print("\nEnter: <position 0..1> [grasp torque Nm]")
        text = input(
            f"target [{current_position:.3f} {current_torque:.3f}]: "
        ).strip()
    finally:
        keyboard.resume()

    if not text:
        return current_position, current_torque
    fields = text.split()
    if len(fields) not in (1, 2):
        raise ValueError("expected: <position> [grasp torque Nm]")
    position = float(fields[0])
    torque = float(fields[1]) if len(fields) == 2 else current_torque
    if not 0.0 <= position <= 1.0:
        raise ValueError("position must be in [0,1]")
    if not 0.0 < torque <= max_torque:
        raise ValueError(f"torque must be in (0,{max_torque}]")
    return position, torque


def _redraw(snapshot, last_key: str, cfg: ForcePositionConfig,
            selected_position: float, selected_torque: float) -> None:
    obs = snapshot.observation
    fault = snapshot.fault_reason or "-"
    lines = [
        "=== Force-Position Grasp Console ===",
        (f"  selected={selected_position:.3f} @ {selected_torque:.2f}Nm  "
         f"hold-max={cfg.hold_torque_limit_nm:.2f}Nm  "
         f"motion-max={cfg.motion_torque_limit_nm:.2f}Nm  "
         f"device-limit={snapshot.device_limit_nm:.2f}Nm  speed={cfg.close_speed_radps:.2f}rad/s"),
        "  t=set exact target  g=run target  o=open  h=hold  e=enable  d=disable  f=reset  q=quit",
        "-" * 96,
        (f"  state={_state_name(snapshot.state):<18} target={snapshot.target_position:6.3f}  "
         f"hold={snapshot.hold_position:6.3f}  force={snapshot.grasp_torque_nm:5.3f}Nm  "
         f"contact={snapshot.contact_count}"),
        (f"  pos={obs.position:7.4f}  raw={obs.raw_pos:+8.4f}rad  "
         f"vel={obs.velocity:+8.4f}rad/s  torque={obs.torque:+7.3f}Nm  "
         f"cmd~={snapshot.commanded_torque_nm:5.3f}Nm"),
        f"  motor={_status_str(obs.status)}  age={obs.age_ms:6.1f}ms  seq={obs.seq}",
        f"  controller fault: {fault}",
        f"  last key: {last_key}",
    ]
    sys.stdout.write("\033[H" + "\r\n".join(line + "\033[K" for line in lines))
    sys.stdout.flush()


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--side", choices=["left", "right", "auto"], default="auto")
    ap.add_argument("--grasp-torque", type=float, default=0.35,
                    help="pure force-hold torque in Nm (default 0.35)")
    ap.add_argument("--hold-torque-limit", "--max-torque",
                    dest="hold_torque_limit", type=float,
                    default=MAX_HOLD_TORQUE_NM,
                    help="force-hold ceiling; cannot exceed 1.8 Nm")
    ap.add_argument("--motion-torque-limit", type=float,
                    default=MAX_MOTION_TORQUE_NM,
                    help="motion/device ceiling; cannot exceed 6 Nm")
    ap.add_argument("--contact-torque", type=float, default=0.080,
                    help="contact torque FLOOR in Nm (default 0.080, the "
                         "firmware's own value). Contact always also requires "
                         "the jaw to have stopped -- that is what separates "
                         "travel from contact, not this number.")
    ap.add_argument("--close-speed", type=float, default=0.5, help="rad/s")
    ap.add_argument("--close-position", type=float, default=0.0,
                    help="normalized close target [0,1], 0=fully closed")
    ap.add_argument("--kp", type=float, default=8.0, help="position-hold Kp")
    ap.add_argument("--kd", type=float, default=1.0, help="position-hold Kd")
    ap.add_argument("--set-device-limit", type=float, metavar="NM",
                    help="persist a new boot limit, verify it, then exit; power-cycle afterward")
    args = ap.parse_args()

    if not 0.0 < args.hold_torque_limit <= MAX_HOLD_TORQUE_NM:
        ap.error(f"--hold-torque-limit must be in (0, {MAX_HOLD_TORQUE_NM}]")
    if not args.hold_torque_limit <= args.motion_torque_limit <= MAX_MOTION_TORQUE_NM:
        ap.error("--motion-torque-limit must be between the hold limit and 6.0 Nm")
    if not 0.0 < args.grasp_torque <= args.hold_torque_limit:
        ap.error("--grasp-torque must be > 0 and no greater than the hold limit")

    gripper = _open_gripper(args.side)
    motor = gripper.motor

    if args.set_device_limit is not None:
        value = args.set_device_limit
        if not 0.0 < value <= MAX_MOTION_TORQUE_NM:
            ap.error(f"--set-device-limit must be in (0, {MAX_MOTION_TORQUE_NM}]")
        motor.set_startup_limit_torque(value)
        readback = motor.get_startup_limit_torque()
        print(f"stored MCU startup/motion torque limit: {readback:.3f} Nm")
        print("Now unplug/replug the gripper so the MCU reapplies 0x700B before motion.")
        return 0

    config = ForcePositionConfig()
    config.close_position = args.close_position
    config.close_speed_radps = args.close_speed
    config.grasp_torque_nm = args.grasp_torque
    config.hold_torque_limit_nm = args.hold_torque_limit
    config.motion_torque_limit_nm = args.motion_torque_limit
    config.contact_torque_nm = args.contact_torque
    config.position_kp = args.kp
    config.position_kd = args.kd

    controller = ForcePositionController(gripper, config)
    motor.clear_fault()
    controller_started = False
    selected_position = args.close_position
    selected_torque = args.grasp_torque
    last_key = "-"
    try:
        controller.start()  # verifies persisted motor limit before enable/motion
        controller_started = True
        motor.enable()
        sys.stdout.write("\033[2J")
        sys.stdout.flush()
        with RawKeyboard() as keyboard:
            while True:
                key = RawKeyboard.poll()
                if key is not None:
                    last_key = repr(key)
                if key == "g":
                    controller.set_target(selected_position, selected_torque)
                elif key == "t":
                    try:
                        selected_position, selected_torque = _prompt_target(
                            keyboard, selected_position, selected_torque,
                            config.hold_torque_limit_nm)
                        controller.set_target(selected_position, selected_torque)
                        last_key = (f"target={selected_position:.3f}, "
                                    f"torque={selected_torque:.3f}Nm")
                    except (EOFError, ValueError) as exc:
                        last_key = f"target rejected: {exc}"
                elif key == "o":
                    controller.release()
                elif key == "h":
                    controller.hold_position()
                elif key == "e":
                    motor.enable()
                elif key == "d":
                    controller.hold_position()
                    motor.disable()
                elif key == "f":
                    motor.clear_fault()
                    controller.reset()
                    motor.enable()
                elif key in ("q", "\x1b", "\x03"):
                    break

                snapshot = controller.snapshot()
                _redraw(snapshot, last_key, config,
                        selected_position, selected_torque)
                time.sleep(0.05)
    finally:
        if controller_started:
            controller.stop()  # sends a zero-torque MIT command first
        try:
            motor.disable()
        except Exception as exc:
            log.error(f"failed to disable motor on exit: {exc}")
        sys.stdout.write("\r\n[exit] controller stopped if started; motor disable requested\r\n")
        sys.stdout.flush()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
