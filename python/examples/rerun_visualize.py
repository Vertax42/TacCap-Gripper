#!/usr/bin/env python3
# Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
"""
TacCap-Gripper async multimodal visualiser using rerun-sdk.

Discovers the plugged-in gripper, streams everything the SDK can produce,
and pushes it into a rerun viewer organised by side ('left' or 'right').

Streams:
    /{side}/wrist_cam                       UVC wrist (~30 Hz, BGR8)
    /{side}/0_tactile/raw                   OG visuotactile #0 raw
    /{side}/0_tactile/rectified             OG visuotactile #0 rectified
    /{side}/1_tactile/raw                   OG visuotactile #1 raw
    /{side}/1_tactile/rectified             OG visuotactile #1 rectified
    /{side}/encoder/position                rad
    /{side}/encoder/velocity                rad/s
    /{side}/imu/accel/{x,y,z}               m/s²
    /{side}/imu/gyro/{x,y,z}                rad/s
    /{side}/imu/mag/{x,y,z}                 µT
    /{side}/imu/temperature                 °C
    /perf/fps/{stream}                      per-stream observed rate

Index:
    - 0_tactile is the OG sensor whose serial ends in an odd digit
    - 1_tactile is the OG sensor whose serial ends in an even digit
    (i.e. 0 = "left within this gripper", 1 = "right within this gripper",
     mirroring the LeaderGripper.tactile_left / tactile_right accessors.)

Usage:
    python python/examples/rerun_visualize.py
    python python/examples/rerun_visualize.py --duration 30
    python python/examples/rerun_visualize.py --imu-hz 100 --encoder-hz 100
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
    LeaderGripper,
    Side,
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


# ---------------------------------------------------------------------------
# Discovery + rerun init
# ---------------------------------------------------------------------------

log.info("[discovery] scanning ...")
eps = find_one()  # throws IoError if 0 or >1 grippers connected
side_str = "left" if eps.side == Side.Left else "right"
log.info(f"[discovery] side='{side_str}'  mcu={eps.mcu_serial}")
log.info(
    f"[discovery] tactile_0 ({eps.tactile_left_serial})  "
    f"tactile_1 ({eps.tactile_right_serial})"
)

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


# Layout: 2 columns
#   left column  = the three video streams stacked vertically
#   right column = encoder + IMU time-series stacked vertically
def _video(name, origin):
    return rrb.Spatial2DView(name=name, origin=origin)


def _ts(name, origin):
    return rrb.TimeSeriesView(name=name, origin=origin)


PERF_STREAMS = ["imu", "encoder", "tac0", "tac1", "wrist"]

blueprint = rrb.Blueprint(
    rrb.Horizontal(
        rrb.Vertical(
            _video(f"{side_str}/wrist_cam", f"/{side_str}/wrist_cam"),
            _video(
                f"{side_str}/0_tactile (rectified)", f"/{side_str}/0_tactile/rectified"
            ),
            _video(
                f"{side_str}/1_tactile (rectified)", f"/{side_str}/1_tactile/rectified"
            ),
            row_shares=[1, 1, 1],
        ),
        rrb.Vertical(
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
        ),
        column_shares=[3, 2],
    ),
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

log.info("[open] LeaderGripper.open() ...")
g = LeaderGripper.open()

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
    g.tactile_left.start(make_tactile_handler(0))  # 0 = odd-last-SN OG
    g.tactile_right.start(make_tactile_handler(1))  # 1 = even-last-SN OG
    g.wrist_camera.start(on_wrist)

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
    for stop_fn in (
        lambda: g.tactile_left.stop(),
        lambda: g.tactile_right.stop(),
        lambda: g.wrist_camera.stop(),
        lambda: g.stop_streaming() if g.is_streaming else None,
    ):
        try:
            stop_fn()
        except Exception as e:
            log.warning(f"during stop: {type(e).__name__}: {e}")

elapsed = time.time() - t_start
log.info(f"[summary] {elapsed:.2f}s total")
with lock:
    for k, n in sorted(counters.items()):
        log.info(f"  {k:10s} {n:6d} frames  ({n / elapsed:6.1f} fps)")
