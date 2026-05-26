#!/usr/bin/env python3
# Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
"""
Encoder zero calibration for a TacCap leader gripper.

Pick which gripper to calibrate by its firmware SN (so when both sides
are plugged in, you don't accidentally zero the wrong one). The script:

  1. Resolves the SN to mcu / wrist-camera / tactile endpoints.
  2. Opens the LeaderGripper and prints the current encoder reading.
  3. Asks you to hold the gripper FULLY CLOSED, then latches that pose
     as the new zero via Encoder::set_zero (wire Cmd::SetEncoderZero).
  4. Re-reads the encoder to confirm post-zero ≈ 0.
  5. (Optional) Asks you to OPEN the gripper to its mechanical limit,
     reads the angle, and sanity-checks against an expected range so
     you catch mechanical / firmware-offset issues immediately.

Usage:
    python python/examples/calibrate.py SN000003                # right
    python python/examples/calibrate.py SN000003 --skip-open-probe
    python python/examples/calibrate.py SN000002 --expected-max-open-rad 1.5

Tip: list available SNs with
    python -c "from xense.taccap import scan_grippers, Side; \\
               [print(f'{\"L\" if g.side==Side.Left else \"R\"} fw={g.firmware_sn} mcu={g.mcu_serial}') for g in scan_grippers()]"
"""

from __future__ import annotations

import argparse
import math
import sys
import time

from xense.taccap import LeaderGripper, Side, scan_grippers

# Mechanical design baseline for TC-GU-01: the leader gripper opens to
# roughly 1.7 rad (~97°) at the hard stop. Override with
# --expected-max-open-rad if your sample varies (linkage tolerance,
# different revision, etc.). Tolerance is intentionally wide — this is
# a sanity bar, not a calibration metric.
DEFAULT_EXPECTED_MAX_OPEN_RAD = 1.7
DEFAULT_OPEN_TOLERANCE_RAD = 0.5

# Tolerance for the "is the new zero actually zero" post-latch check.
# Firmware latches what it sees the moment it processes the command, so
# any residual is mostly hand-jitter between read and latch.
POST_ZERO_TOLERANCE_RAD = 0.01

# Mild ANSI colors for the human-facing prompts. Skipped if stdout
# isn't a tty (piped to a log file, etc.) so the log stays grep-clean.
_TTY = sys.stdout.isatty()
def _c(code: str, s: str) -> str:
    return f"\033[{code}m{s}\033[0m" if _TTY else s
def _cyan(s):   return _c("36", s)
def _yellow(s): return _c("33", s)
def _green(s):  return _c("32", s)
def _red(s):    return _c("31", s)
def _bold(s):   return _c("1",  s)


def _rad_to_deg(r: float) -> float:
    return r * 180.0 / math.pi


def _resolve_sn(sn: str):
    all_eps = scan_grippers()
    matches = [e for e in all_eps if e.firmware_sn == sn]
    if not matches:
        listing = (
            ", ".join(
                f"{e.firmware_sn} ({'L' if e.side == Side.Left else 'R'})"
                for e in all_eps
            )
            or "(none)"
        )
        sys.exit(
            f"error: no gripper with firmware SN={sn!r} is plugged in.\n"
            f"       currently visible: {listing}"
        )
    if len(matches) > 1:
        sys.exit(
            f"error: {len(matches)} grippers report SN={sn!r} — firmware-SN "
            "collision, check firmware burning."
        )
    return matches[0]


def _open_gripper(eps) -> LeaderGripper:
    return LeaderGripper(
        eps.mcu_device,
        eps.wrist_video,
        eps.tactile_left_serial,
        eps.tactile_right_serial,
    )


def _read_positions_rad(g: LeaderGripper) -> tuple[float, float]:
    """Return (raw, cooked) position in rad.

    `cooked` is what SDK consumers see (clamped to >= 0 by Encoder's
    post-zero normalisation). `raw` is the firmware-side value before
    clamping — calibration UX must show this, otherwise pre-latch drift
    (e.g. 0.09 rad) would silently display as 0.00 and the user would
    think calibration isn't needed.
    """
    s = g.encoder.read_once()
    return float(s.raw_position_rad), float(s.position_rad)


def _prompt(msg: str) -> None:
    """Block until the user hits Enter; surface ANSI emphasis on the prompt."""
    try:
        input(f"  {msg} ")
    except (EOFError, KeyboardInterrupt):
        print()
        sys.exit(_red("aborted."))


def calibrate(sn: str, *, skip_open_probe: bool,
              expected_max_open_rad: float,
              open_tolerance_rad: float) -> int:
    eps = _resolve_sn(sn)
    side_str = "Left" if eps.side == Side.Left else "Right"

    print()
    print(_cyan("=" * 64))
    print(_cyan(f"  TacCap leader-gripper encoder calibration"))
    print(_cyan("=" * 64))
    print(f"  firmware SN  : {_bold(eps.firmware_sn)}")
    print(f"  side         : {_bold(side_str)}")
    print(f"  mcu serial   : {eps.mcu_serial}")
    print(f"  mcu device   : {eps.mcu_device}")
    print(f"  tactile L/R  : {eps.tactile_left_serial} / {eps.tactile_right_serial}")
    print()

    g = _open_gripper(eps)

    # ---- 1. Current reading ------------------------------------------------
    cur_raw, cur_cooked = _read_positions_rad(g)
    print(f"  current encoder: "
          f"{_bold(f'raw={cur_raw:+.4f} rad')}  ({_rad_to_deg(cur_raw):+.2f}°)"
          f"   cooked={cur_cooked:+.4f} rad")
    print()

    # ---- 2. Latch zero -----------------------------------------------------
    print(_yellow("Step 1/2: hold the gripper FULLY CLOSED."))
    print(_yellow(f"          (Ctrl+C any time to abort without changing zero.)"))
    _prompt(_yellow("→ press [Enter] when held closed:"))

    pre_raw, pre_cooked = _read_positions_rad(g)
    print(f"  pre-latch reading : raw={pre_raw:+.4f} rad "
          f"({_rad_to_deg(pre_raw):+.2f}°)   cooked={pre_cooked:+.4f}")

    try:
        g.encoder.set_zero()
    except Exception as e:
        print(_red(f"  ✗ set_zero failed: {type(e).__name__}: {e}"))
        return 1
    time.sleep(0.05)  # let firmware settle one streaming tick

    post_raw, post_cooked = _read_positions_rad(g)
    print(f"  post-latch reading: raw={post_raw:+.4f} rad "
          f"({_rad_to_deg(post_raw):+.2f}°)   cooked={post_cooked:+.4f}")

    # Validate against the RAW value — cooked is clamped at 0 so it
    # would always look "perfect" even for residual drift up to +inf
    # on the negative side. raw tells the truth.
    if abs(post_raw) <= POST_ZERO_TOLERANCE_RAD:
        print(_green(f"  ✓ zero latched OK (|raw post-zero| ≤ {POST_ZERO_TOLERANCE_RAD:.3f} rad)"))
    else:
        print(_yellow(
            f"  ⚠ raw post-zero is {post_raw:+.4f} rad. Firmware latched what "
            "it saw the instant the cmd arrived — most likely you moved the "
            "gripper between read and latch. Re-run if you want it tighter."
        ))
    print()

    # ---- 3. Optional: sanity-check max-open angle --------------------------
    if skip_open_probe:
        print(_cyan("  --skip-open-probe set, done."))
        return 0

    print(_yellow("Step 2/2: open the gripper to its MECHANICAL LIMIT."))
    print(_yellow(
        f"          Expected full-open angle ≈ {expected_max_open_rad:.2f} rad "
        f"({_rad_to_deg(expected_max_open_rad):.0f}°), tolerance "
        f"±{open_tolerance_rad:.2f} rad."
    ))
    _prompt(_yellow("→ press [Enter] when fully open:"))

    open_raw, _ = _read_positions_rad(g)
    print(f"  fully-open reading: "
          f"{_bold(f'{open_raw:+.4f} rad')}  ({_rad_to_deg(open_raw):+.2f}°)")
    deviation = abs(open_raw - expected_max_open_rad)
    if deviation <= open_tolerance_rad:
        print(_green(
            f"  ✓ within expected range (|measured - expected| = "
            f"{deviation:.3f} rad ≤ {open_tolerance_rad:.2f} rad)"
        ))
    else:
        print(_yellow(
            f"  ⚠ deviation {deviation:.3f} rad exceeds tolerance "
            f"{open_tolerance_rad:.2f} rad. Possible causes: linkage "
            "slipped, encoder direction flipped, or expected angle wrong "
            "for this revision. Re-run with --expected-max-open-rad if you "
            "know the real value."
        ))
    print()

    # ---- 4. Optional live readout for visual confirmation ------------------
    print(_cyan("  Live encoder readout (10 Hz, raw | cooked; Ctrl+C to exit):"))
    try:
        while True:
            raw, cooked = _read_positions_rad(g)
            sys.stdout.write(
                f"\r    raw={raw:+.4f} rad ({_rad_to_deg(raw):+6.2f}°) | "
                f"cooked={cooked:+.4f} rad     "
            )
            sys.stdout.flush()
            time.sleep(0.1)
    except KeyboardInterrupt:
        print()
        print(_green("  done."))
    return 0


def main() -> int:
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument(
        "sn",
        help="Firmware SN of the leader gripper to calibrate (e.g. SN000003).",
    )
    p.add_argument(
        "--skip-open-probe",
        action="store_true",
        help="Skip the optional 'open to mechanical limit' sanity check.",
    )
    p.add_argument(
        "--expected-max-open-rad",
        type=float,
        default=DEFAULT_EXPECTED_MAX_OPEN_RAD,
        help=f"Expected full-open angle in rad "
             f"(default {DEFAULT_EXPECTED_MAX_OPEN_RAD:.2f}).",
    )
    p.add_argument(
        "--open-tolerance-rad",
        type=float,
        default=DEFAULT_OPEN_TOLERANCE_RAD,
        help=f"Tolerance for the open-angle check in rad "
             f"(default {DEFAULT_OPEN_TOLERANCE_RAD:.2f}).",
    )
    args = p.parse_args()
    return calibrate(
        args.sn,
        skip_open_probe=args.skip_open_probe,
        expected_max_open_rad=args.expected_max_open_rad,
        open_tolerance_rad=args.open_tolerance_rad,
    )


if __name__ == "__main__":
    sys.exit(main())
