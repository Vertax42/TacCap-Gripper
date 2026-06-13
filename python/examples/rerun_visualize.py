#!/usr/bin/env python3
# Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
"""
TacCap-Gripper async multimodal visualiser using rerun-sdk.

Discovers the plugged-in gripper, streams its MCU telemetry (IMU + encoder)
and pushes it into a rerun viewer organised by side ('left' or 'right').

The wrist camera and OG visuotactile sensors are owned by an external camera
service now — the SDK no longer discovers or starts them. To visualise them
too, pass their device IDs explicitly; they are then opened directly via the
standalone Camera / TactileSensor classes (not through the gripper):

    --wrist <device>          e.g. /dev/video2 or /dev/v4l/by-id/...-video-index0
    --tactile-left <serial>   OG serial -> /{side}/0_tactile
    --tactile-right <serial>  OG serial -> /{side}/1_tactile

Streams:
    /{side}/encoder/position                rad
    /{side}/encoder/velocity                rad/s
    /{side}/imu/accel/{x,y,z}               m/s²
    /{side}/imu/gyro/{x,y,z}                rad/s
    /{side}/imu/mag/{x,y,z}                 µT
    /{side}/imu/temperature                 °C
    /{side}/wrist_cam                       UVC wrist (opt-in, ~30 Hz, BGR8)
    /{side}/0_tactile/raw + /rectified      OG visuotactile #0 (opt-in)
    /{side}/1_tactile/raw + /rectified      OG visuotactile #1 (opt-in)
    /perf/fps/{stream}                      per-stream observed rate

Usage:
    python python/examples/rerun_visualize.py
    python python/examples/rerun_visualize.py --duration 30
    python python/examples/rerun_visualize.py --imu-hz 100 --encoder-hz 100
    python python/examples/rerun_visualize.py \\
        --wrist /dev/video2 --tactile-left OG000477 --tactile-right OG000478
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
from xense.taccap import (
    Camera,
    LeaderGripper,
    Side,
    TactileSensor,
    find_one,
    log,
)

# ---------------------------------------------------------------------------
# Args
# ---------------------------------------------------------------------------

parser = argparse.ArgumentParser(
    description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
)
parser.add_argument(
    "--duration",
    type=float,
    default=0.0,
    help="Run for N seconds, then exit. 0 = until Ctrl+C (default).",
)
parser.add_argument(
    "--imu-hz",
    type=int,
    default=100,
    help="Requested IMU stream rate in Hz (default 100).",
)
parser.add_argument(
    "--encoder-hz",
    type=int,
    default=100,
    help="Requested encoder stream rate in Hz (default 100).",
)
parser.add_argument(
    "--wrist",
    metavar="DEVICE",
    default=None,
    help="Open the wrist UVC camera at this /dev path and visualise it. "
    "Off by default (owned by the external camera service).",
)
parser.add_argument(
    "--tactile-left",
    metavar="SERIAL",
    default=None,
    help="Open the OG sensor with this serial as 0_tactile. Off by default.",
)
parser.add_argument(
    "--tactile-right",
    metavar="SERIAL",
    default=None,
    help="Open the OG sensor with this serial as 1_tactile. Off by default.",
)
parser.add_argument(
    "--no-rectify",
    action="store_true",
    help="Skip on-sensor rectification of OG cameras (raw only).",
)
parser.add_argument(
    "--save",
    metavar="FILE",
    default=None,
    help="Save the rerun stream to a .rrd file instead of "
    "spawning a viewer. Useful on headless machines.",
)
parser.add_argument(
    "--connect",
    metavar="HOST:PORT",
    default=None,
    help="Connect to an already-running rerun viewer "
    "(rerun --serve / rerun --connect). Mutually "
    "exclusive with --save and the default --spawn.",
)
parser.add_argument(
    "--no-spawn",
    action="store_true",
    help="Don't spawn a viewer (logs go to no sink unless --save / --connect was set).",
)
args = parser.parse_args()

# Which opt-in camera streams are enabled this run.
TAC_IDXS = []
if args.tactile_left:
    TAC_IDXS.append(0)
if args.tactile_right:
    TAC_IDXS.append(1)
WRIST_ENABLED = bool(args.wrist)


# ---------------------------------------------------------------------------
# Discovery + rerun init
# ---------------------------------------------------------------------------

log.info("[discovery] scanning ...")
eps = find_one()  # throws IoError if 0 or >1 grippers connected
side_str = "left" if eps.side == Side.Left else "right"
log.info(f"[discovery] side='{side_str}'  mcu={eps.mcu_serial}")

# Pick exactly one of: save / connect / spawn / nothing.
if args.save:
    rr.init("TacCap-Gripper")
    rr.save(args.save)
    log.info(f"[rerun] saving stream to {args.save}")
elif args.connect:
    host, _, port = args.connect.partition(":")
    rr.init("TacCap-Gripper")
    rr.connect_grpc(f"rerun+http://{host}:{port or '9876'}/proxy")
    log.info(f"[rerun] connected to {host}:{port}")
elif args.no_spawn:
    rr.init("TacCap-Gripper")
    log.info("[rerun] init only, no sink")
else:
    rr.init("TacCap-Gripper", spawn=True)
    log.info("[rerun] spawned viewer")


# Layout: video streams (if any opt-in cameras) on the left, encoder + IMU
# time-series on the right.
def _video(name, origin):
    return rrb.Spatial2DView(name=name, origin=origin)


def _ts(name, origin):
    return rrb.TimeSeriesView(name=name, origin=origin)


PERF_STREAMS = (
    ["imu", "encoder"]
    + [f"tac{i}" for i in TAC_IDXS]
    + (["wrist"] if WRIST_ENABLED else [])
)

ts_column = rrb.Vertical(
    _ts("encoder", f"/{side_str}/encoder"),
    _ts("imu / accel (m/s²)", f"/{side_str}/imu/accel"),
    _ts("imu / gyro (rad/s)", f"/{side_str}/imu/gyro"),
    _ts("imu / mag (µT)", f"/{side_str}/imu/mag"),
    _ts("imu / temperature (°C)", f"/{side_str}/imu/temperature"),
    # Be explicit about which series to plot — rerun's automatic
    # origin-based discovery sometimes leaves siblings off the
    # legend, especially when their entities show up after the
    # blueprint is sent.
    rrb.TimeSeriesView(
        name="observed fps",
        origin="/perf/fps",
        contents=[f"+ $origin/{s}" for s in PERF_STREAMS],
    ),
    row_shares=[2, 2, 2, 2, 1, 1],
)

# Only build a video column when at least one opt-in camera is enabled.
video_views = []
if WRIST_ENABLED:
    video_views.append(_video(f"{side_str}/wrist_cam", f"/{side_str}/wrist_cam"))
for i in TAC_IDXS:
    video_views.append(
        _video(f"{side_str}/{i}_tactile (rectified)", f"/{side_str}/{i}_tactile/rectified")
    )

if video_views:
    root = rrb.Horizontal(
        rrb.Vertical(*video_views, row_shares=[1] * len(video_views)),
        ts_column,
        column_shares=[3, 2],
    )
else:
    root = ts_column

blueprint = rrb.Blueprint(
    root,
    rrb.BlueprintPanel(state="collapsed"),
    rrb.TimePanel(state="collapsed"),
)
rr.send_blueprint(blueprint)

# Pre-create the perf entities at t=0 so the TimeSeriesView legend is fully
# populated before any data arrives. Without this, rerun's blueprint
# resolution can drop series whose first sample lands after the view is
# created, leaving them off the legend.
for s in PERF_STREAMS:
    rr.log(f"/perf/fps/{s}", rr.Scalars(0.0))

# ---------------------------------------------------------------------------
# Streaming
# ---------------------------------------------------------------------------

# Per-stream sample counters for the FPS time-series.
counters = defaultdict(int)
lock = threading.Lock()
# `t_start` / `mono_start` are reset to "right after start_streaming()" so
# the summary FPS reflects steady-state streaming rate, not (open + start +
# streaming) wall time. libxense + V4L2 init alone can eat several seconds.
t_start = 0.0
mono_start = 0.0


def _bump(stream: str) -> None:
    with lock:
        counters[stream] += 1


def _now() -> float:
    """Seconds since this script started, on a monotonic clock.

    Used as the unified `host` timeline for everything we log to rerun
    (callbacks, perf samples). We deliberately ignore the per-sample
    s.host_time / f.host_time fields here because those carry the C++
    steady_clock epoch (e.g. seconds since boot) which doesn't share a
    zero point with anything Python sees, so plotting them on the same
    rerun timeline as wall-clock-derived perf samples produces a visible
    gap.
    """
    return time.monotonic() - mono_start


def _set_time() -> None:
    rr.set_time(timeline="host", duration=_now())


def _bgr_to_rgb(img: np.ndarray) -> np.ndarray:
    if img.ndim == 3 and img.shape[2] == 3:
        return img[..., ::-1]
    return img


def on_imu(s) -> None:
    _bump("imu")
    _set_time()
    a = s.accel_mps2
    g = s.gyro_radps
    m = s.mag_uT
    rr.log(f"/{side_str}/imu/accel/x", rr.Scalars(float(a[0])))
    rr.log(f"/{side_str}/imu/accel/y", rr.Scalars(float(a[1])))
    rr.log(f"/{side_str}/imu/accel/z", rr.Scalars(float(a[2])))
    rr.log(f"/{side_str}/imu/gyro/x", rr.Scalars(float(g[0])))
    rr.log(f"/{side_str}/imu/gyro/y", rr.Scalars(float(g[1])))
    rr.log(f"/{side_str}/imu/gyro/z", rr.Scalars(float(g[2])))
    rr.log(f"/{side_str}/imu/mag/x", rr.Scalars(float(m[0])))
    rr.log(f"/{side_str}/imu/mag/y", rr.Scalars(float(m[1])))
    rr.log(f"/{side_str}/imu/mag/z", rr.Scalars(float(m[2])))
    rr.log(f"/{side_str}/imu/temperature", rr.Scalars(float(s.temperature_c)))


def on_encoder(s) -> None:
    _bump("encoder")
    _set_time()
    rr.log(f"/{side_str}/encoder/position", rr.Scalars(float(s.position_rad)))
    rr.log(f"/{side_str}/encoder/velocity", rr.Scalars(float(s.velocity_rad_s)))


def make_tactile_handler(idx: int):
    def handler(f) -> None:
        _bump(f"tac{idx}")
        _set_time()
        rr.log(
            f"/{side_str}/{idx}_tactile/raw",
            rr.Image(_bgr_to_rgb(f.raw), color_model="rgb"),
        )
        if not args.no_rectify and f.rectified.size > 0:
            rr.log(
                f"/{side_str}/{idx}_tactile/rectified",
                rr.Image(_bgr_to_rgb(f.rectified), color_model="rgb"),
            )

    return handler


def on_wrist(f) -> None:
    _bump("wrist")
    _set_time()
    rr.log(f"/{side_str}/wrist_cam", rr.Image(_bgr_to_rgb(f.image), color_model="rgb"))


# ---------------------------------------------------------------------------
# Run
# ---------------------------------------------------------------------------

log.info("[open] LeaderGripper.open() (MCU-only) ...")
g = LeaderGripper.open()

# Opt-in cameras/tactile: opened directly here (not by the gripper) only when
# the user passed device IDs on the CLI.
tactiles = []  # list of (idx, TactileSensor)
if args.tactile_left:
    tactiles.append((0, TactileSensor(args.tactile_left, rectify=not args.no_rectify)))
if args.tactile_right:
    tactiles.append((1, TactileSensor(args.tactile_right, rectify=not args.no_rectify)))
wrist_cam = Camera(args.wrist) if WRIST_ENABLED else None

# Wrap everything that touches background threads (subscribe + start +
# main loop) in try/finally. If start_streaming or any callback throws,
# we still stop the camera/tactile worker threads BEFORE Python begins
# tearing the interpreter down — otherwise a worker thread that's mid-
# callback during Python shutdown can crash with
#   `FATAL: exception not rethrown / Aborted (core dumped)`
# from pybind11's GIL acquire path on a half-dead interpreter.
try:
    log.info("[subscribe] callbacks attached")
    g.imu.on_data(on_imu)
    g.encoder.on_data(on_encoder)
    for idx, ts in tactiles:
        ts.start(make_tactile_handler(idx))  # idx 0 = --tactile-left, 1 = --tactile-right
    if wrist_cam is not None:
        wrist_cam.start(on_wrist)

    log.info(f"[stream] starting (imu={args.imu_hz}Hz, encoder={args.encoder_hz}Hz) ...")
    g.start_streaming(imu_hz=args.imu_hz, encoder_hz=args.encoder_hz)

    # Anchor the perf clocks to "streaming actually started" so the FPS
    # summary at the end isn't dragged down by ~4s of libxense/V4L2 init.
    t_start = time.time()
    mono_start = time.monotonic()

    log.info(
        f"[run]    {'Ctrl+C to stop' if args.duration <= 0 else f'duration={args.duration}s'}"
    )

    last_print = time.time()
    last_counts = defaultdict(int)
    while True:
        time.sleep(0.5)

        now = time.time()
        elapsed = now - last_print
        with lock:
            snap = dict(counters)
        deltas = {k: snap[k] - last_counts[k] for k in snap}
        last_counts = snap

        # Per-stream observed fps -> rerun. Use the same monotonic
        # timeline as the callbacks (_now()) so perf points align with
        # the data points instead of landing in a separate timeline gap.
        _set_time()
        for k, n in deltas.items():
            fps = n / elapsed if elapsed > 0 else 0.0
            rr.log(f"/perf/fps/{k}", rr.Scalars(fps))

        # In-place TUI status line — kept on raw stdout so the carriage
        # return keeps overwriting one line. spdlog appends a newline per
        # entry, which would break that animation.
        line = "  ".join(f"{k}={snap[k]:5d}" for k in sorted(snap))
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
    # Always stop the worker threads, even if an exception bubbled up
    # before we reached the success path.
    log.info("[stop] streaming + workers ...")
    stop_fns = [lambda ts=ts: ts.stop() for _, ts in tactiles]
    if wrist_cam is not None:
        stop_fns.append(lambda: wrist_cam.stop())
    stop_fns.append(lambda: g.stop_streaming() if g.is_streaming else None)
    for stop_fn in stop_fns:
        try:
            stop_fn()
        except Exception as e:
            log.warning(f"during stop: {type(e).__name__}: {e}")

elapsed = time.time() - t_start
log.info(f"[summary] {elapsed:.2f}s total")
with lock:
    for k, n in sorted(counters.items()):
        log.info(f"  {k:10s} {n:6d} frames  ({n / elapsed:6.1f} fps)")
