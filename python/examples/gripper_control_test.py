#!/usr/bin/env python3
# Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
"""
Interactive gripper open/close control test (Python end).

Exercises BOTH normalized control paths on a real follower gripper, in the
0..1 "position" coordinate (0 = fully closed, 1 = fully open):

  Part A — one-shot commands via FollowerGripper.set_position(0..1)
  Part B — realtime ControlLoop (set_target + observation)

Interactive: you press Enter before each move so you can watch it happen and
abort (Ctrl-C) at any time. After every move it reads the position back and
prints commanded vs reached.

Usage:
    python python/examples/gripper_control_test.py

SAFETY: this commands real motor motion. Keep the jaws clear (no fingers /
objects you don't intend to grip). The motor is disabled on exit and on abort.
The gripper must already be calibrated (GripperConfig Valid); if not, calibrate
first (zero at full close, then write max_open).
"""

from __future__ import annotations

import math
import time

from xense.taccap import ControlLoop, FollowerGripper, log

KP, KD = 8.0, 1.0   # gentle impedance gains for the test


def ask(msg: str) -> None:
    """Wait for the operator before each move; Ctrl-C / EOF aborts cleanly."""
    try:
        input(f"  >> {msg}  [Enter to go, Ctrl-C to abort] ")
    except (EOFError, KeyboardInterrupt):
        print("\n[abort] operator stopped the test")
        raise SystemExit(0)


def goto_oneshot(g: FollowerGripper, target: float,
                 settle_s: float = 1.5, hz: float = 100.0) -> float:
    """Drive to a normalized target with set_position(), resubmitting at `hz`
    so the firmware target stays fresh, then read the reached position back.

    set_position() is fire-and-forget (no ACK); a single frame would go stale,
    so a discrete move resubmits the target for a short settle window."""
    period = 1.0 / hz
    t0 = time.perf_counter()
    deadline = t0
    while time.perf_counter() - t0 < settle_s:
        g.set_position(target, KP, KD)
        deadline += period
        s = deadline - time.perf_counter()
        if s > 0:
            time.sleep(s)
    return g.position()


def main() -> int:
    log.set_level("info")

    print("[open] FollowerGripper.open() ...")
    g = FollowerGripper.open()
    cfg = g.get_gripper_config()
    calibrated = bool(cfg.flags & 0x0001)
    print(f"[config] calibrated={calibrated} reverse={bool(cfg.flags & 0x0002)} "
          f"max_open={cfg.max_open_rad:.4f} rad")
    if not calibrated:
        print("[error] gripper is not calibrated — aborting. Calibrate first "
              "(zero at full close, then write max_open via set_gripper_config).")
        return 1
    print(f"[init] current position = {g.position():.3f}  (0=closed, 1=open)")

    g.motor.clear_fault()
    g.motor.enable()
    print("[motor] enabled")
    try:
        # ---------- Part A: one-shot set_position(0..1) ----------
        print("\n=== Part A: one-shot set_position(0..1) ===")
        for name, tgt in [("OPEN  (1.0)", 1.0), ("CLOSE (0.0)", 0.0),
                          ("HALF  (0.5)", 0.5), ("OPEN  (0.9)", 0.9)]:
            ask(f"move to {name}")
            reached = goto_oneshot(g, tgt)
            print(f"     commanded={tgt:.2f}  reached={reached:.3f}  "
                  f"err={reached - tgt:+.3f}")

        # ---------- Part B: realtime ControlLoop ----------
        print("\n=== Part B: realtime ControlLoop (set_target + observation) ===")
        loop = ControlLoop(g, hz=200, kp=KP, kd=KD, motor_stream_hz=100)
        loop.start()   # seeds target = current position (no jump)
        try:
            for name, tgt in [("OPEN  (1.0)", 1.0), ("CLOSE (0.0)", 0.0),
                              ("HALF  (0.5)", 0.5)]:
                ask(f"loop -> {name}")
                loop.set_target(tgt)
                time.sleep(1.5)                       # loop holds while it settles
                obs = loop.observation()
                print(f"     target={tgt:.2f}  obs.position={obs.position:.3f}  "
                      f"age={obs.age_ms:.1f}ms  submit_hz={loop.submit_hz:.0f}  "
                      f"obs_seq={obs.seq}")

            ask("smooth open<->close sweep for 4s")
            t0 = time.perf_counter()
            while time.perf_counter() - t0 < 4.0:
                phase = 0.5 - 0.5 * math.cos(2.0 * math.pi * 0.33 *
                                             (time.perf_counter() - t0))
                loop.set_target(phase)                # 0 -> 1 -> 0 ...
                time.sleep(1.0 / 50.0)                # policy at 50Hz
            o = loop.observation()
            print(f"     sweep done: obs.position={o.position:.3f}  "
                  f"submit_hz={loop.submit_hz:.0f}")
        finally:
            loop.stop()
            print("[loop] stopped")

        print("\n[done] all control paths exercised")
        return 0
    finally:
        g.motor.disable()
        print(f"[exit] motor disabled; final position={g.position():.3f}")


if __name__ == "__main__":
    raise SystemExit(main())
