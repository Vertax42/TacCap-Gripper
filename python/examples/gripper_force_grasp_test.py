#!/usr/bin/env python3
# Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
"""
Gentle force grasp test — the "force" half of force-position hybrid control,
via host-side contact detection.

The firmware's position-mode ``max_torque`` is NOT a tight force cap, so relying
on it crushes soft objects. Instead this closes the jaws step by step with a
soft impedance (low kp) and detects contact from the *at-rest* torque.

Contact is detected from **position progress**, not torque: after each small
close step we resubmit the target until the motor is at rest (velocity-gated),
then check whether the jaws actually advanced. When closing freely they follow
the target step for step; when they meet an object (or the mechanical closed
stop) they stall — the actual position stops decreasing even though we keep
commanding "closer". Two consecutive stalled steps = contact, and we freeze
there and report the holding torque as the grip force.

Why not a torque threshold? The gripper has a position-dependent restoring
torque (it springs toward open), so the holding torque grows as you close even
with nothing in the jaws — an absolute or baseline-relative torque threshold
false-triggers on that. Position stall is immune to it.

Usage:
    python python/examples/gripper_force_grasp_test.py
    python python/examples/gripper_force_grasp_test.py --contact-delta 0.06

SAFETY: real motion + grip force. Put a SOFT object in the jaws (or nothing).
Grip force is bounded by --contact-delta above baseline and hard-capped by
--abort-torque; the motor opens and is disabled on exit / abort.
"""

from __future__ import annotations

import argparse
import time

from xense.taccap import FollowerGripper, log


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--min-progress", type=float, default=0.3,
                    help="fraction of --step the jaws must actually close each step; "
                         "below this = stalled (blocked by object or hard-stop)")
    ap.add_argument("--grip-margin", type=float, default=0.045,
                    help="max normalized position the target may lead the actual jaw "
                         "by; caps grip force at ~kp*this (in rad). Must exceed the "
                         "gripper's own restoring torque or the jaws won't even close.")
    ap.add_argument("--abort-torque", type=float, default=0.30,
                    help="hard safety cap (Nm) — abort + disable if exceeded")
    ap.add_argument("--kp", type=float, default=5.0, help="impedance stiffness (Nm/rad)")
    ap.add_argument("--kd", type=float, default=1.0, help="impedance damping (Nm·s/rad)")
    ap.add_argument("--step", type=float, default=0.015,
                    help="normalized position closed per increment")
    ap.add_argument("--vel-rest", type=float, default=0.05,
                    help="|actual_vel| (rad/s) below which the motor is 'at rest'")
    ap.add_argument("--settle-max", type=float, default=0.5,
                    help="max time to wait for rest per step (s)")
    ap.add_argument("--hold", type=float, default=2.0, help="hold time after contact (s)")
    args = ap.parse_args()

    log.set_level("info")
    g = FollowerGripper.open()
    cfg = g.get_gripper_config()
    if not (cfg.flags & 0x0001):
        print("[error] gripper not calibrated — aborting")
        return 1
    m = g.motor

    def settle(target: float):
        """Resubmit `target` until the motor is at rest (|vel| < vel_rest) or
        settle_max elapses; return the last MotorStatusSample (read at rest)."""
        t0 = time.perf_counter()
        st = m.read_status()
        while time.perf_counter() - t0 < args.settle_max:
            g.set_position(target, args.kp, args.kd)
            st = m.read_status()
            if time.perf_counter() - t0 >= 0.10 and abs(st.actual_vel) < args.vel_rest:
                break
            time.sleep(0.015)
        return st

    start = g.position()
    m.clear_fault()
    m.enable()

    st0 = settle(start)
    prev_pos = g.rad_to_pos(st0.actual_pos)
    stall_thresh = args.step * args.min_progress
    print(f"[init] position={start:.3f}  step={args.step}  "
          f"stall<{stall_thresh:.4f}/step  abort={args.abort_torque} Nm  kp={args.kp}")

    aborted = False
    contact = False
    goal = prev_pos
    hold_tgt = prev_pos
    tq = 0.0
    over = 0
    try:
        # ---- Close step by step. The commanded target is clamped to never lead
        # the ACTUAL jaw position by more than --grip-margin, so the impedance
        # force is capped at ~kp*grip_margin: the jaws close freely, then hold an
        # object at that gentle force instead of ramping up to the abort cap.
        # Contact = the jaws stall (stop advancing) for two consecutive steps.
        while goal > 0.0:
            goal = max(0.0, goal - args.step)
            actual = g.rad_to_pos(m.read_status().actual_pos)
            hold_tgt = max(goal, actual - args.grip_margin)   # force clamp
            st = settle(hold_tgt)                             # waits for rest
            cur_pos = g.rad_to_pos(st.actual_pos)
            tq = abs(st.actual_torque)
            progress = prev_pos - cur_pos                     # how much it closed
            print(f"  [close] goal={goal:.3f} tgt={hold_tgt:.3f} pos={cur_pos:.3f} "
                  f"moved={progress:+.4f} torque={st.actual_torque:+.4f} Nm")
            if tq >= args.abort_torque:
                print(f"  !! torque {tq:.3f} > abort {args.abort_torque} -> ABORT")
                aborted = True
                break
            over = over + 1 if progress < stall_thresh else 0
            prev_pos = cur_pos
            if over >= 2:
                contact = True
                break

        # ---- Hold the grasp and check the force is stable ----
        if contact and not aborted:
            print(f"  [stall] jaws stopped at pos={g.position():.3f} — holding {args.hold}s")
            t0 = time.perf_counter()
            peak = tq
            last_print = 0.0
            while time.perf_counter() - t0 < args.hold:
                g.set_position(hold_tgt, args.kp, args.kd)
                el = time.perf_counter() - t0
                if el - last_print >= 0.25:
                    last_print = el
                    st = m.read_status()
                    peak = max(peak, abs(st.actual_torque))
                    print(f"    hold t={el:4.2f} pos={g.position():.3f} "
                          f"torque={st.actual_torque:+.4f} Nm")
                    if abs(st.actual_torque) > args.abort_torque:
                        print("    !! over-torque during hold -> ABORT"); aborted = True; break
                time.sleep(0.005)
            tq = peak

        print("\n[result]")
        p = g.position()
        if aborted:
            print("  ABORTED on over-torque")
        elif contact and p < 0.08:
            print(f"  closed to {p:.3f} (~fully closed) holding ~{tq:.3f} Nm at the "
                  f"mechanical stop — nothing between the jaws.")
        elif contact:
            print(f"  GRASPED: jaws stalled on an object at position {p:.3f}, holding "
                  f"~{tq:.3f} Nm — force-aware, gentle (stopped on the object, not crushed).")
        else:
            print(f"  reached {p:.3f} without stalling — bottom of travel.")
    finally:
        # Release: open back up, then disable.
        try:
            t0 = time.perf_counter()
            while time.perf_counter() - t0 < 1.5:
                g.set_position(1.0, args.kp, args.kd)
                time.sleep(0.01)
        except Exception:
            pass
        m.disable()
        print(f"[exit] motor disabled; final position={g.position():.3f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
