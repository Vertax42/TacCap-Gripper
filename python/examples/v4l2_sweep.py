#!/usr/bin/env python3
# Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
"""
Sweep through fourcc/resolution combos for the trio
(/dev/video0 + /dev/video2 + /dev/video4) and print which ones get
non-zero frames on every camera.

Goal: find a combination that fits inside the USB-2.0 isoc budget on
this debug machine where two PC Cameras + one IMX385 share a single
external port.

Each combo runs ~2 s; the cameras are released between combos.
"""

from __future__ import annotations

import argparse
import time
from collections import defaultdict

from xense.taccap import Camera, log

DEV = ["/dev/video0", "/dev/video2", "/dev/video4"]
NAMES = ["cam0(PC)", "cam1(IMX)", "cam2(PC)"]


def yuyv_bw_mbs(w: int, h: int, fps: int) -> float:
    return w * h * 2 * fps / 1e6


def fmt_label(w: int, h: int, fps: int, mjpg: bool) -> str:
    return f"{w}x{h}@{fps}{'M' if mjpg else 'Y'}"


# (cam0, cam1, cam2) where each entry is (w, h, fps, mjpg).
# Ordered roughly by ascending isoc-bandwidth pressure.
PC = "PC"   # PC Camera (video0/video4)
IMX = "IMX" # IMX385 (video2)

COMBOS = [
    # All YUYV at smallest size — minimum bandwidth (~4.6MB/s × 3 = 14MB/s)
    ("3×YUYV 320x240@30",
     (320, 240, 30, False), (320, 240, 30, False), (320, 240, 30, False)),

    # PCs YUYV small, IMX MJPG large (MJPG is variable bitrate so 1280x720
    # may still fit if PCs are tiny)
    ("PC YUYV 320x240@30, IMX MJPG 1280x720@60",
     (320, 240, 30, False), (1280, 720, 60, True), (320, 240, 30, False)),

    # PCs YUYV small, IMX MJPG mid
    ("PC YUYV 320x240@30, IMX MJPG 640x480@60",
     (320, 240, 30, False), (640, 480, 60, True), (320, 240, 30, False)),

    # PCs MJPG small (chip ignores fps so always 100), IMX MJPG mid
    ("PC MJPG 320x240@100, IMX MJPG 640x480@60",
     (320, 240, 100, True), (640, 480, 60, True), (320, 240, 100, True)),

    # PCs MJPG small, IMX YUYV 320 (lowest IMX bandwidth: ~4.6MB/s)
    ("PC MJPG 320x240@100, IMX YUYV 320x240@30",
     (320, 240, 100, True), (320, 240, 30, False), (320, 240, 100, True)),

    # PCs YUYV 640 (~18MB/s × 2 = 36MB/s) — likely too much
    ("PC YUYV 640x480@30, IMX MJPG 640x480@60",
     (640, 480, 30, False), (640, 480, 60, True), (640, 480, 30, False)),

    # All MJPG max sizes (status-quo failure mode)
    ("PC MJPG 640x480@100, IMX MJPG 1280x720@60",
     (640, 480, 100, True), (1280, 720, 60, True), (640, 480, 100, True)),
]


def make_cb(name: str, ctr: dict[str, int]):
    def cb(_f):
        ctr[name] = ctr.get(name, 0) + 1
    return cb


def run_combo(label: str, cfgs, duration: float,
              open_order: tuple[int, ...] = (0, 1, 2)) -> dict[str, int]:
    log.info(f"[combo] {label}  (open order: {open_order})")
    cams: dict[int, Camera] = {}
    ctr: dict[str, int] = defaultdict(int)
    try:
        # Open + start in the given order. start() triggers VIDIOC_STREAMON
        # on the worker's first read, which is when the kernel actually
        # commits the isoc altsetting — earlier openers win the bandwidth.
        for i in open_order:
            w, h, fps, mjpg = cfgs[i]
            try:
                cam = Camera(DEV[i], w, h, float(fps), mjpg)
            except Exception as e:
                log.error(f"  [open] {NAMES[i]} {DEV[i]} {fmt_label(w,h,fps,mjpg)}: "
                          f"{type(e).__name__}: {e}")
                return ctr
            cams[i] = cam
            cam.start(make_cb(NAMES[i], ctr))
            # Give the worker a moment to actually pull a frame and lock
            # in the altsetting before the next device opens.
            time.sleep(0.3)
        time.sleep(duration)
    finally:
        for cam in cams.values():
            try:
                cam.stop()
            except Exception:
                pass
    return ctr


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--per-combo", type=float, default=2.0,
                    help="Seconds per combo (default 2).")
    ap.add_argument("--log-level", default="info")
    args = ap.parse_args()
    log.set_level(args.log_level)

    # Try every combo twice: default open order (0,1,2) and IMX-first (1,0,2).
    # The latter lets cam1 grab its altsetting before the PC Cameras inflate
    # their isoc claim.
    orders = [(0, 1, 2), (1, 0, 2)]
    log.info(f"[sweep] running {len(COMBOS)}×{len(orders)} combos, "
             f"{args.per_combo}s each")
    rows = []
    for label, *cfgs in COMBOS:
        for order in orders:
            ctr = run_combo(label, cfgs, args.per_combo, order)
            time.sleep(0.3)
            rows.append((label, order, [ctr.get(n, 0) for n in NAMES]))

    log.info("[sweep] results")
    log.info(f"  {'combo':62s}  {'order':10s}  "
             f"{NAMES[0]:>12s}  {NAMES[1]:>12s}  {NAMES[2]:>12s}  status")
    for label, order, counts in rows:
        ok_all = all(c > 0 for c in counts)
        ok_part = any(c > 0 for c in counts)
        status = "ALL ✓" if ok_all else ("partial" if ok_part else "FAIL")
        log.info(f"  {label:62s}  {str(order):10s}  "
                 f"{counts[0]:12d}  {counts[1]:12d}  {counts[2]:12d}  {status}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
