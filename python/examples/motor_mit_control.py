#!/usr/bin/env python3
# Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
"""
Primitive demo of the follower gripper's MIT force-position control API.

This drives the follower motor directly with the SDK's high-rate, no-ACK
submission path (`Motor.submit_impedance`), the same primitive the firmware's
500Hz slave control task consumes. The MIT impedance frame is itself the
force-position hybrid: the kp/kd terms track `target_pos`, while
`feedforward_torque` adds a force component.

What this example IS:
  - a minimal "submit MIT targets at a fixed rate, watch the health channel"
    loop, demonstrating submit_impedance() + control_stats() + read_status().

What this example is NOT (these live in the upper layer, taccap_gripper_ros2):
  - the master->slave follow / teleoperation loop,
  - the grasp state machine (contact detection, torque latching, hold),
  - the open-coordinate mapping (encoder rad <-> gripper open span).

submit_impedance() is fire-and-forget: no ACK, no NACK, no retry, no throw on a
rejected target (the only exception is on a stopped transport). Health is
out-of-band — we poll control_stats() / read_status() off the hot loop.

Usage:
    python python/examples/motor_mit_control.py
    python python/examples/motor_mit_control.py --hz 200 --seconds 5 --amp 0.2

SAFETY: this commands real motor motion. Keep the gripper clear of obstacles
and be ready to kill the process. The motor is disabled on exit.
"""

from __future__ import annotations

import argparse
import math
import time

from xense.taccap import FollowerGripper, log


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--hz", type=float, default=200.0,
                    help="target submission rate (Hz); firmware control loop runs at 500Hz")
    ap.add_argument("--seconds", type=float, default=5.0, help="run duration")
    ap.add_argument("--amp", type=float, default=0.15,
                    help="sinusoid amplitude around zero (rad)")
    ap.add_argument("--freq", type=float, default=0.5, help="sinusoid frequency (Hz)")
    ap.add_argument("--kp", type=float, default=10.0, help="position stiffness (Nm/rad)")
    ap.add_argument("--kd", type=float, default=1.0, help="position damping (Nm·s/rad)")
    args = ap.parse_args()

    log.info("[open] FollowerGripper.open() (MCU-only) ...")
    g = FollowerGripper.open()
    motor = g.motor

    period = 1.0 / args.hz
    omega = 2.0 * math.pi * args.freq

    motor.clear_fault()
    motor.enable()
    try:
        start = time.perf_counter()
        deadline = start
        next_report = start + 0.25
        sent = 0
        while True:
            now = time.perf_counter()
            elapsed = now - start
            if elapsed >= args.seconds:
                break

            target_pos = args.amp * math.sin(omega * elapsed)
            # Force-position hybrid: track target_pos via kp/kd, no extra
            # feed-forward torque in this demo (0.0).
            motor.submit_impedance(target_pos, args.kp, args.kd, 0.0)
            sent += 1

            # Low-rate health probe, OFF the submission cadence.
            if now >= next_report:
                next_report += 0.25
                st = motor.control_stats()
                status = motor.read_status()
                log.info(
                    "[health] submit#%d actual_hz=%.1f applied_seq=%u "
                    "err=%u age=%ums | pos=%.3f tgt=%.3f",
                    sent, st.actual_hz, st.applied_seq, st.error_count,
                    st.target_age_ms, status.actual_pos, target_pos,
                )

            # Deadline pacing (busy-tolerant): sleep most of the remaining slice.
            deadline += period
            slack = deadline - time.perf_counter()
            if slack > 0:
                time.sleep(slack)
            else:
                deadline = time.perf_counter()  # we fell behind; resync

        log.info("[done] submitted %d frames over %.1fs (~%.0f Hz)",
                 sent, args.seconds, sent / args.seconds)
    finally:
        motor.disable()
        log.info("[exit] motor disabled")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
