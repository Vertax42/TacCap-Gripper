#!/usr/bin/env python3
# Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
"""
TacCap-Gripper firmware over-the-air (OTA) update demo.

Pushes a firmware .bin to the MCU's inactive Flash bank, verifies its
CRC32, and triggers the bank-swap reboot. No SWD probe needed.

POWER-CYCLE THE GRIPPER AFTERWARDS. The bank-swap reboot is a soft reset: it
restarts the MCU but never powers down the USB-serial bridge, and the device
comes back in a degraded state that is indistinguishable from a healthy one --
right version string, stream running, counters clean. The only symptom is that
it quietly drops status frames. Measured on hardware, same unit, same firmware,
same cable, 60-second runs: 35-39 frames lost per run after OTA alone, zero
after unplugging and replugging, three runs each way. Treat the replug as part
of the update, not as troubleshooting.

The released images ship in this repo under `firmware/`, so the usual
argument is just their name — it resolves against that directory from any
working directory, including a parent repo that vendors this one.

Usage:

    # Push firmware to whichever gripper is plugged in (single-gripper)
    python python/examples/ota_update.py tc-gu-01-master.bin

    # Bilateral: pick a side explicitly
    python python/examples/ota_update.py tc-gu-01-master.bin left
    python python/examples/ota_update.py --get-status right

    # Tag the target version (informational; firmware uses it for the
    # post-install verification log + bank metadata).
    python python/examples/ota_update.py tc-gu-01-master.bin \\
        --target-version 1.2.2

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
import json
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
    log,
)

import _calib_flow


def _open_gripper(target: str | None) -> tuple[LeaderGripper, object]:
    """Resolve the repo-wide selector (`left` / `right` / SN / None) → gripper."""
    eps, _by_side, _all = _calib_flow.resolve_target(target)
    # OTA only needs the MCU control link; cameras stay off (the default).
    g = LeaderGripper(mcu_device=eps.mcu_device)
    return g, eps


def _parse_version(spec: Optional[str]) -> OtaTargetVersion:
    """Parse MAJOR.MINOR.PATCH, or the legacy four-part form.

    Versions are presented as three parts everywhere now, so that is what
    people will type. The wire still carries a fourth "build" byte; it
    defaults to 0, which is what firmware has always set it to. The 4-part
    form keeps working so older scripts and docs do not break.
    """
    if spec is None:
        return OtaTargetVersion(0, 0, 0, 0)
    parts = spec.split(".")
    if len(parts) not in (3, 4):
        raise SystemExit(
            f"--target-version must be MAJOR.MINOR.PATCH, got {spec!r}")
    try:
        nums = [int(p) for p in parts]
    except ValueError:
        raise SystemExit(f"--target-version components must be integers: {spec!r}")
    if any(n < 0 or n > 255 for n in nums):
        raise SystemExit(f"--target-version components must each fit in uint8: {spec!r}")
    if len(nums) == 3:
        nums.append(0)
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


def _dim(t: str) -> str:
    return f"\033[2m{t}\033[0m" if sys.stdout.isatty() else t


def _sdk_root() -> str:
    """This repo's root — two levels up from python/examples/."""
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.abspath(os.path.join(here, "..", ".."))


def _firmware_dir() -> str:
    return os.path.join(_sdk_root(), "firmware")


def _load_manifest() -> dict:
    """Read firmware/manifest.json if it is there. Absent is fine."""
    try:
        with open(os.path.join(_firmware_dir(), "manifest.json"), "rb") as f:
            return json.load(f)
    except (OSError, ValueError):
        return {}


def _resolve_firmware(path: str) -> Optional[str]:
    """Find the image whether `path` is relative to the cwd or to this repo.

    The images ship inside this repo, but the repo is usually vendored as a
    submodule of something else — so `firmware/tc-gu-01-master.bin`, the path
    our docs print because it works from the SDK root, is not the path that
    works from the parent repo's root. Rather than making every downstream
    README carry its own prefix, accept both: the literal path first, then the
    same path and the bare filename under our own firmware/.
    """
    if os.path.isfile(path):
        return path
    for cand in (os.path.join(_sdk_root(), path),
                 os.path.join(_firmware_dir(), os.path.basename(path))):
        if os.path.isfile(cand):
            return cand
    return None


def _resolve_or_report(path: str) -> Optional[str]:
    resolved = _resolve_firmware(path)
    if resolved is None:
        print(f"[ERROR] firmware file not found: {path}\n"
              f"        shipped images live in {_firmware_dir()}",
              file=sys.stderr)
    return resolved


def _check_role(fw_bytes: bytes, firmware_sn: str, force: bool) -> int:
    """Refuse to flash an image built for the other role.

    A gripper's role is the last character of its firmware SN, NOT which hand
    it is on — two grippers on opposite sides of a rig are often both masters.
    Flashing the wrong role's image bricks the MCU and needs an SWD probe to
    recover, so this is worth blocking rather than warning about.

    Identification is by CRC32 against firmware/manifest.json, so it only
    fires for our released images; a hand-built or third-party .bin is
    unidentifiable and passes through with a note.
    """
    manifest = _load_manifest()
    images = manifest.get("images", {})
    if not images or not firmware_sn:
        return 0

    # Compare as integers, not strings: "0x...".upper() also uppercases the
    # "x" in the prefix, so a string compare silently never matches.
    def _crc_of(meta) -> int:
        try:
            return int(str(meta.get("crc32", "")), 16)
        except ValueError:
            return -1

    crc = crc32_iso_hdlc(fw_bytes)
    matched = next((role for role, meta in images.items()
                    if _crc_of(meta) == crc), None)
    if matched is None:
        print(f"  role check  : {_dim('image not in manifest, cannot verify')}")
        return 0

    want = images[matched].get("sn_suffix", "")
    have = firmware_sn[-1:]
    if have == want:
        print(f"  role check  : OK — {matched} image, SN ends {have!r}")
        return 0

    other = next((r for r, m in images.items() if m.get("sn_suffix") == have),
                 "unknown")
    msg = (f"[ROLE MISMATCH] this is the {matched.upper()} image "
           f"(expects SN ending {want!r}), but {firmware_sn} ends {have!r} "
           f"— it is a {other.upper()}.")
    if not force:
        alt = images.get(other, {}).get("file", f"tc-gu-01-{other}.bin")
        print(f"\n{msg}\n"
              f"Flashing the wrong role bricks the MCU and needs an SWD probe "
              f"to recover.\nUse {alt} instead (shipped in {_firmware_dir()}), "
              f"or --force if you really mean it.", file=sys.stderr)
        return 1
    print(f"\n{msg}\n--force given, proceeding anyway.", file=sys.stderr)
    return 0


def _cmd_update(args: argparse.Namespace, g: LeaderGripper, eps) -> int:
    fw_path = _resolve_or_report(args.firmware)
    if fw_path is None:
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
    print(f"  target ver   : {_calib_flow.format_version(target.major, target.minor, target.patch)}")
    if _check_role(fw_bytes, getattr(eps, "firmware_sn", "") or "", args.force):
        return 1
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
    print()
    print("!! POWER-CYCLE THE GRIPPER BEFORE YOU TRUST ANY MEASUREMENT.")
    print("   The reboot above is a SOFT reset: it restarts the MCU but does")
    print("   not power the USB-serial bridge down, and the device comes back")
    print("   in a degraded state that looks completely healthy. Measured on")
    print("   hardware, same unit, same firmware, same cable, 60s runs:")
    print("     after OTA alone   35-39 status frames lost per run")
    print("     after power cycle 0 lost, three runs in a row")
    print("   Nothing in the version string, the stream, or the counters")
    print("   distinguishes the two states -- the only symptom is that your")
    print("   numbers are quietly wrong. Unplug and replug it.")
    return 0


def main(argv=None) -> int:
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("firmware", nargs="?",
                   help="Path to firmware .bin (omit with --get-status)")
    p.add_argument("target", nargs="?", metavar="left|right|SN",
                   help="Which gripper to flash: 'left' / 'right' (side comes "
                        "from the firmware SN), or an explicit SN. Omit when "
                        "exactly one gripper is plugged in.")
    p.add_argument("--target-version", default=None,
                   help="Target version MAJOR.MINOR.PATCH "
                        "(informational; default 0.0.0)")
    p.add_argument("--no-progress", action="store_true",
                   help="Suppress per-block progress bar")
    p.add_argument("--yes", "-y", action="store_true",
                   help="Skip the interactive confirmation prompt")
    p.add_argument("--force", action="store_true",
                   help="flash even if the image's role does not match the "
                        "gripper's firmware SN (bricks the MCU if wrong)")
    p.add_argument("--get-status", action="store_true",
                   help="Print current firmware-side OTA state machine + exit")
    args = p.parse_args(argv)

    # `--get-status` takes no firmware, so a lone positional there is the
    # gripper selector, not a path. Without this, `--get-status right` reports
    # "firmware file not found: right", which is a confusing way to say
    # "positional order differs when you are not flashing".
    if args.get_status and args.firmware and not args.target:
        args.firmware, args.target = None, args.firmware
    if args.get_status and args.firmware:
        p.error("--get-status takes no firmware image "
                f"(got {args.firmware!r}); pass only the gripper selector")
    if not args.get_status and not args.firmware:
        p.error("firmware argument required unless --get-status is given")

    # Resolve before touching hardware: a typo'd path should not cost a
    # discovery + open round-trip to find out about.
    if args.firmware:
        args.firmware = _resolve_or_report(args.firmware)
        if args.firmware is None:
            return 1

    log.set_level("info")
    g, eps = _open_gripper(args.target)
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

    rc = _cmd_update(args, g, eps)
    # Don't try to stop_streaming / clean shutdown — after OtaApply the
    # firmware is rebooting and any wire command will time out, which
    # would dirty the output. Let Python tear the gripper down on exit.
    return rc


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
