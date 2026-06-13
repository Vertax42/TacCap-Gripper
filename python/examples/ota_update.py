#!/usr/bin/env python3
# Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
"""
TacCap-Gripper firmware over-the-air (OTA) update demo.

Pushes a firmware .bin to the MCU's inactive Flash bank, verifies its
CRC32, and triggers the bank-swap reboot. No SWD probe needed.

Usage:

    # Push firmware to whichever gripper is plugged in (single-gripper)
    python python/examples/ota_update.py path/to/tc-gu-01.bin

    # Bilateral: pick a side explicitly
    python python/examples/ota_update.py path/to/tc-gu-01.bin --side left

    # Tag the target version (informational; firmware uses it for the
    # post-install verification log + bank metadata).
    python python/examples/ota_update.py path/to/tc-gu-01.bin \\
        --target-version 1.6.2.3

    # Just probe — don't flash anything
    python python/examples/ota_update.py --get-status

Notes:

  - After the final OtaApply ACK the firmware reboots; the SDK
    Transport's next command on the same /dev/ttyACM* will time out,
    which is expected. Wait ~3 s for USB re-enumeration, then
    re-open the gripper.
  - This script opens LeaderGripper to print the pre-update firmware
    version + SN. That itself sends Cmd::GetVersion + GetSn, which
    incidentally also drains any leftover DATA backlog before the OTA
    starts (same trick as discovery::scan_all).
"""
from __future__ import annotations

import argparse
import os
import sys
import time
from typing import Optional

from xense.taccap import (
    LeaderGripper,
    OtaSession,
    OtaTargetVersion,
    Side,
    crc32_iso_hdlc,
    find_left,
    find_one,
    find_right,
    log,
)


def _open_gripper(side: str) -> tuple[LeaderGripper, object]:
    """Resolve `side` ('auto' | 'left' | 'right') → LeaderGripper + endpoints."""
    if side == "auto":
        eps = find_one()
    elif side == "left":
        eps = find_left()
    elif side == "right":
        eps = find_right()
    else:
        raise ValueError(f"bad --side: {side!r}")
    # OTA only needs the MCU control link; cameras stay off (the default).
    g = LeaderGripper(mcu_device=eps.mcu_device)
    return g, eps


def _parse_version(spec: Optional[str]) -> OtaTargetVersion:
    if spec is None:
        return OtaTargetVersion(0, 0, 0, 0)
    parts = spec.split(".")
    if len(parts) != 4:
        raise SystemExit(
            f"--target-version must be MAJOR.MINOR.PATCH.BUILD, got {spec!r}")
    try:
        nums = [int(p) for p in parts]
    except ValueError:
        raise SystemExit(f"--target-version components must be integers: {spec!r}")
    if any(n < 0 or n > 255 for n in nums):
        raise SystemExit(f"--target-version components must each fit in uint8: {spec!r}")
    return OtaTargetVersion(*nums)


def _format_size(n: int) -> str:
    return f"{n:,} B ({n/1024.0:.1f} KiB)"


def _make_progress_callback(total_bytes: int, quiet: bool):
    """Return an `(written, total) -> None` printer with a 5 Hz throttle."""
    t_start = time.monotonic()
    last_print = [t_start]

    def cb(written: int, total: int) -> None:
        now = time.monotonic()
        # Always print the final 100% line; throttle others to 5 Hz so the
        # terminal doesn't flicker on fast hosts (OTA write throughput on
        # USART3 @ 3M baud routinely hits hundreds of KB/s).
        if quiet:
            return
        if written < total and (now - last_print[0]) < 0.20:
            return
        last_print[0] = now
        pct = 100.0 * written / total
        elapsed = max(now - t_start, 1e-6)
        kbps = (written / 1024.0) / elapsed
        bar_len = 40
        filled = int(bar_len * written / total)
        bar = "#" * filled + "-" * (bar_len - filled)
        sys.stdout.write(
            f"\r  [{bar}] {pct:5.1f}%  "
            f"{written:>7,}/{total:,} B  {kbps:7.1f} KB/s")
        sys.stdout.flush()
        if written >= total:
            sys.stdout.write("\n")

    return cb


def _cmd_get_status(g: LeaderGripper) -> int:
    st = g.ota.get_status()
    state_names = {
        0: "Idle", 1: "Started", 2: "Receiving",
        3: "Verified", 4: "Applying", 5: "Error",
    }
    print(f"  state         = {state_names.get(st.state, 'Unknown')} ({st.state})")
    print(f"  error_code    = 0x{st.error_code:02X}")
    print(f"  bytes_written = {st.bytes_written}")
    print(f"  progress_ppt  = {st.progress_ppt}  ({st.progress_ppt/10:.1f}%)")
    return 0


def _cmd_update(args: argparse.Namespace, g: LeaderGripper) -> int:
    fw_path = args.firmware
    if not os.path.isfile(fw_path):
        print(f"[ERROR] firmware file not found: {fw_path}", file=sys.stderr)
        return 1
    fw_size = os.path.getsize(fw_path)
    with open(fw_path, "rb") as f:
        fw_bytes = f.read()
    if len(fw_bytes) != fw_size:
        print(f"[ERROR] short read: expected {fw_size}, got {len(fw_bytes)}",
              file=sys.stderr)
        return 1

    crc = crc32_iso_hdlc(fw_bytes)
    target = _parse_version(args.target_version)

    print("=== OTA update ===")
    print(f"  firmware     : {fw_path}")
    print(f"  size         : {_format_size(fw_size)}")
    print(f"  CRC32        : 0x{crc:08X}")
    print(f"  target ver   : {target.major}.{target.minor}.{target.patch}.{target.build}")
    print()

    if not args.yes:
        try:
            resp = input("Proceed? Firmware will be flashed + MCU will reboot. [y/N] ")
        except EOFError:
            resp = ""
        if resp.strip().lower() not in ("y", "yes"):
            print("Aborted.")
            return 0

    progress = _make_progress_callback(fw_size, quiet=args.no_progress)
    t0 = time.monotonic()
    try:
        # Use update_from_bytes to avoid re-reading the file inside the
        # SDK (we already have the bytes for the CRC32 pre-print above).
        g.ota.update_from_bytes(fw_bytes, target, progress)
    except Exception as e:
        elapsed = time.monotonic() - t0
        print(f"\n[ERROR] OTA aborted after {elapsed:.1f}s: "
              f"{type(e).__name__}: {e}", file=sys.stderr)
        try:
            st = g.ota.get_status(timeout_ms=500)
            print(f"        firmware OTA state = {st.state}, "
                  f"err = 0x{st.error_code:02X}", file=sys.stderr)
        except Exception:
            pass
        return 1

    elapsed = time.monotonic() - t0
    print(f"\n=== OTA complete in {elapsed:.1f}s "
          f"({fw_size/elapsed/1024:.1f} KB/s avg) ===")
    print("Firmware is rebooting now.")
    print("Wait ~3 s for USB re-enumeration, then re-open the gripper")
    print("to confirm GetVersion returns the new version.")
    return 0


def main(argv=None) -> int:
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("firmware", nargs="?",
                   help="Path to firmware .bin (omit with --get-status)")
    p.add_argument("--side", choices=("auto", "left", "right"),
                   default="auto",
                   help="Which gripper to flash (default: auto = single-gripper)")
    p.add_argument("--target-version", default=None,
                   help="Target version MAJOR.MINOR.PATCH.BUILD "
                        "(informational; default 0.0.0.0)")
    p.add_argument("--no-progress", action="store_true",
                   help="Suppress per-block progress bar")
    p.add_argument("--yes", "-y", action="store_true",
                   help="Skip the interactive confirmation prompt")
    p.add_argument("--get-status", action="store_true",
                   help="Print current firmware-side OTA state machine + exit")
    args = p.parse_args(argv)

    if not args.get_status and not args.firmware:
        p.error("firmware argument required unless --get-status is given")

    log.set_level("info")
    g, eps = _open_gripper(args.side)
    side = "left" if eps.side == Side.Left else "right"
    print(f"[discovery] {side}  ch343={eps.mcu_serial}  fw_sn={eps.firmware_sn!r}")
    print(f"            {eps.mcu_device}")
    print()

    if args.get_status:
        print("=== current OTA state ===")
        try:
            return _cmd_get_status(g)
        except Exception as e:
            print(f"[ERROR] get_status: {type(e).__name__}: {e}", file=sys.stderr)
            return 1
        finally:
            del g

    rc = _cmd_update(args, g)
    # Don't try to stop_streaming / clean shutdown — after OtaApply the
    # firmware is rebooting and any wire command will time out, which
    # would dirty the output. Let Python tear the gripper down on exit.
    return rc


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
