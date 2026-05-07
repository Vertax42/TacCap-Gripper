#!/usr/bin/env python3
# Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
"""
Manual /dev/video* probe — bypasses TacCap's firmware-keyed discovery.

Use when the gripper hardware hasn't been flashed yet (no OG/XC serials),
so `LeaderGripper.open()` would fail at the `Camera: empty device path`
step. We open each requested /dev/videoN as a plain UVC `Camera` (the
same C++ class wrist-cam uses) and push frames into a rerun viewer.

This does NOT touch libxense rectification — the OGs come through as raw
UVC frames here. It only proves that:

  - the kernel has bound uvcvideo to the hardware
  - we can negotiate the requested format and pull frames at speed
  - the frame plumbing all the way out to rerun works

Defaults match the factory debug machine where lsusb shows three cameras:
  /dev/video0  PC Camera (1bcf:28c4)   — OG candidate
  /dev/video2  LRCP imx385 (1bcf:384f) — wrist candidate
  /dev/video4  PC Camera (1bcf:28c4)   — OG candidate

Usage:
    python python/examples/v4l2_probe.py
    python python/examples/v4l2_probe.py --duration 10
    python python/examples/v4l2_probe.py --cam0 /dev/video0 --cam1 /dev/video4
    python python/examples/v4l2_probe.py --no-spawn --save out.rrd
"""

from __future__ import annotations

import argparse
import sys
import threading
import time
from collections import defaultdict

import numpy as np
import rerun as rr
import rerun.blueprint as rrb
from xense.taccap import Camera, log

# ---------------------------------------------------------------------------
# Args
# ---------------------------------------------------------------------------

parser = argparse.ArgumentParser(
    description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
)
parser.add_argument("--cam0", default="/dev/video0",
                    help="First camera device (default: /dev/video0).")
parser.add_argument("--cam1", default="/dev/video2",
                    help="Second camera device (default: /dev/video2).")
parser.add_argument("--cam2", default="/dev/video4",
                    help="Third camera device (default: /dev/video4).")
parser.add_argument("--cam0-size", default="640x480",
                    help="WxH for cam0 (default 640x480).")
parser.add_argument("--cam1-size", default="1280x720",
                    help="WxH for cam1 (default 1280x720, IMX385 sweet spot).")
parser.add_argument("--cam2-size", default="640x480",
                    help="WxH for cam2 (default 640x480).")
parser.add_argument("--cam0-fps", type=float, default=30.0)
parser.add_argument("--cam1-fps", type=float, default=60.0,
                    help="IMX385 only enumerates MJPG at 60Hz; requesting "
                    "30 makes the V4L2 backend silently drop frames.")
parser.add_argument("--cam2-fps", type=float, default=30.0)
parser.add_argument("--no-mjpg", action="store_true",
                    help="Use YUYV for all cameras (default is MJPEG).")
parser.add_argument("--cam0-no-mjpg", action="store_true",
                    help="Override cam0 to YUYV (per-cam fourcc).")
parser.add_argument("--cam1-no-mjpg", action="store_true",
                    help="Override cam1 to YUYV.")
parser.add_argument("--cam2-no-mjpg", action="store_true",
                    help="Override cam2 to YUYV.")
parser.add_argument("--duration", type=float, default=0.0,
                    help="Run for N seconds, then exit. 0 = until Ctrl+C.")
parser.add_argument("--save", metavar="FILE", default=None)
parser.add_argument("--connect", metavar="HOST:PORT", default=None)
parser.add_argument("--no-spawn", action="store_true")
parser.add_argument("--log-level", default="info",
                    help="spdlog level: trace/debug/info/warn/error.")
args = parser.parse_args()

log.set_level(args.log_level)


def parse_size(s: str) -> tuple[int, int]:
    w, _, h = s.partition("x")
    return int(w), int(h)


# ---------------------------------------------------------------------------
# Rerun init
# ---------------------------------------------------------------------------

if args.save:
    rr.init("TacCap-v4l2-probe")
    rr.save(args.save)
    log.info(f"[rerun] saving stream to {args.save}")
elif args.connect:
    host, _, port = args.connect.partition(":")
    rr.init("TacCap-v4l2-probe")
    rr.connect_grpc(f"rerun+http://{host}:{port or '9876'}/proxy")
    log.info(f"[rerun] connected to {host}:{port}")
elif args.no_spawn:
    rr.init("TacCap-v4l2-probe")
    log.info("[rerun] init only, no sink")
else:
    rr.init("TacCap-v4l2-probe", spawn=True)
    log.info("[rerun] spawned viewer")


CAM_NAMES = ["cam0", "cam1", "cam2"]
blueprint = rrb.Blueprint(
    rrb.Horizontal(
        rrb.Vertical(*[rrb.Spatial2DView(name=n, origin=f"/{n}") for n in CAM_NAMES],
                     row_shares=[1, 1, 1]),
        rrb.TimeSeriesView(
            name="observed fps",
            origin="/perf/fps",
            contents=[f"+ $origin/{n}" for n in CAM_NAMES],
        ),
        column_shares=[3, 2],
    ),
    rrb.BlueprintPanel(state="collapsed"),
    rrb.TimePanel(state="collapsed"),
)
rr.send_blueprint(blueprint)
for n in CAM_NAMES:
    rr.log(f"/perf/fps/{n}", rr.Scalars(0.0))


# ---------------------------------------------------------------------------
# Open cameras
# ---------------------------------------------------------------------------

cams: list[Camera] = []
per_cam_yuyv = [args.cam0_no_mjpg, args.cam1_no_mjpg, args.cam2_no_mjpg]
specs = [
    (args.cam0, parse_size(args.cam0_size), args.cam0_fps,
     not (args.no_mjpg or per_cam_yuyv[0])),
    (args.cam1, parse_size(args.cam1_size), args.cam1_fps,
     not (args.no_mjpg or per_cam_yuyv[1])),
    (args.cam2, parse_size(args.cam2_size), args.cam2_fps,
     not (args.no_mjpg or per_cam_yuyv[2])),
]

for i, (dev, (w, h), fps, use_mjpg) in enumerate(specs):
    log.info(f"[open] {CAM_NAMES[i]} dev={dev} {w}x{h}@{fps}Hz "
             f"fourcc={'MJPG' if use_mjpg else 'YUYV'}")
    try:
        cam = Camera(dev, w, h, fps, use_mjpg)
    except Exception as e:
        log.error(f"[open] {CAM_NAMES[i]} FAILED: {type(e).__name__}: {e}")
        # Tear down any cameras opened so far before bailing out.
        for prev in cams:
            try:
                prev.stop()
            except Exception:
                pass
        sys.exit(1)
    cams.append(cam)


# ---------------------------------------------------------------------------
# Streaming
# ---------------------------------------------------------------------------

counters: dict[str, int] = defaultdict(int)
counter_lock = threading.Lock()
mono_start = time.monotonic()


def _now() -> float:
    return time.monotonic() - mono_start


def _set_time() -> None:
    rr.set_time(timeline="host", duration=_now())


def _bgr_to_rgb(img: np.ndarray) -> np.ndarray:
    if img.ndim == 3 and img.shape[2] == 3:
        return img[..., ::-1]
    return img


def make_handler(name: str):
    def handler(f) -> None:
        with counter_lock:
            counters[name] += 1
        _set_time()
        rr.log(f"/{name}", rr.Image(_bgr_to_rgb(f.image), color_model="rgb"))
    return handler


t_start = time.time()
try:
    for i, cam in enumerate(cams):
        cam.start(make_handler(CAM_NAMES[i]))
    log.info(f"[run] {'Ctrl+C to stop' if args.duration <= 0 else f'duration={args.duration}s'}")

    last_print = time.time()
    last_counts: dict[str, int] = defaultdict(int)
    while True:
        time.sleep(0.5)
        now = time.time()
        elapsed = now - last_print
        with counter_lock:
            snap = dict(counters)
        deltas = {k: snap[k] - last_counts[k] for k in snap}
        last_counts = snap

        _set_time()
        for k, n in deltas.items():
            rr.log(f"/perf/fps/{k}", rr.Scalars(n / elapsed if elapsed > 0 else 0.0))

        # In-place TUI status — same pattern as rerun_visualize.py: a log
        # via spdlog would add a newline per tick and break the carriage
        # return.
        line = "  ".join(
            f"{k}={snap.get(k, 0):5d} ({deltas.get(k, 0) / elapsed:5.1f}fps)"
            for k in CAM_NAMES
        )
        sys.stdout.write(f"\r[stats] {line}     ")
        sys.stdout.flush()

        last_print = now
        if args.duration > 0 and (now - t_start) >= args.duration:
            sys.stdout.write("\n")
            sys.stdout.flush()
            log.info(f"[done] reached duration={args.duration}s")
            break

except KeyboardInterrupt:
    sys.stdout.write("\n")
    sys.stdout.flush()
    log.info("[done] Ctrl+C")

finally:
    log.info("[stop] cameras ...")
    for cam in cams:
        try:
            cam.stop()
        except Exception as e:
            log.warning(f"during stop: {type(e).__name__}: {e}")

elapsed = time.time() - t_start
log.info(f"[summary] {elapsed:.2f}s total")
with counter_lock:
    for k in CAM_NAMES:
        n = counters.get(k, 0)
        fps = (n / elapsed) if elapsed > 0 else 0.0
        log.info(f"  {k:6s} {n:6d} frames  ({fps:6.1f} fps)")
