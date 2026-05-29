#!/usr/bin/env python3
# Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
"""
Dual-leader rerun visualiser, augmented with Pico4 tracker poses.

The TacCap leader gripper is hand-held — there's no rigid arm to derive
its 6-DoF pose from. We attach a Pico4 motion tracker to each gripper
and pull the pose from the XenseVR PC Service via xensevr_pc_service_sdk.
This example overlays those tracker poses on top of the existing dual
gripper stream so you can see, in one viewer, what each gripper is
*sensing* (tactile / wrist / encoder / IMU) and *where it is in space*.

Per-side mapping is explicit: pass the firmware-burned tracker SN as
--left-tracker-sn / --right-tracker-sn so swap-in/swap-out events
can't silently relabel sides. If the PC Service isn't reachable or
no trackers report, the gripper streams keep flowing and the 3D pose
panel just stays empty until trackers come online.

Pico coordinate frame: X=right, Y=up, Z=forward/in. LEFT-handed Y-up
(despite the docs saying right-handed). World origin = headset position
the moment the VR app started.

Usage:
    python python/examples/rerun_dual_with_tracker.py \\
        --left-tracker-sn  PT-XXXXXXXXXXXX \\
        --right-tracker-sn PT-YYYYYYYYYYYY

Skip the tracker side entirely (degrades to plain dual-gripper view):
    python python/examples/rerun_dual_with_tracker.py --no-tracker

Prerequisites:
    1. XenseVR PC Service running locally (default :60061). See
       https://github.com/Vertax42/XRoboToolkit-PC-Service .
    2. Trackers paired with the Pico headset and reporting motion.
"""

from __future__ import annotations

import argparse
import gc
import os
import sys
import threading
import time
from collections import defaultdict, deque
from typing import Optional

# These flush knobs are read by the rerun SDK at rr.init() time, so we
# must seed them before any other rerun import / call. Small batch +
# short tick = low latency at the cost of more gRPC packets. We tried
# the opposite (64 KB / 30 ms) once: the larger batch held small
# Transform3D updates behind the bigger tactile/wrist JPEG batches,
# making the 3D ellipsoid look choppy. Keep this low-latency profile
# and instead decouple the xrt poller from rr.log via TrackerCache
# below — that's what actually keeps the world view smooth.
os.environ.setdefault("RERUN_FLUSH_NUM_BYTES", "4096")
os.environ.setdefault("RERUN_FLUSH_TICK_SECS", "0.01")

import cv2  # noqa: E402  — must come after env knobs to keep them paired
import numpy as np  # noqa: E402
import rerun as rr  # noqa: E402
import rerun.blueprint as rrb  # noqa: E402

from xense.taccap import LeaderGripper, Side, find_left, find_right, log  # noqa: E402

# Pico SDK is optional — fall back cleanly if it's not installed (e.g.
# someone runs this with --no-tracker on a machine without the service).
try:
    import xensevr_pc_service_sdk as xrt  # type: ignore
    _HAS_PICO_SDK = True
except ImportError:
    xrt = None  # type: ignore
    _HAS_PICO_SDK = False

IMU_HZ = 100
ENCODER_HZ = 100
# 90 Hz is the Pico tracker's native sampling rate — polling slower
# than this throws away real data; polling faster just returns duplicate
# samples. Now that JPEG-compressed image streams have freed up gRPC
# bandwidth, the viewer keeps up with the full tracker rate.
TRACKER_POLL_HZ = 90
TRACKER_TRAIL_MAX = 90

# JPEG quality for image streams. 90 is visually indistinguishable from
# raw on these sensors and gives ~6-8× wire-size reduction vs BGR8.
JPEG_QUALITY = 90
_JPEG_ENCODE_PARAMS = [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY]
# Visualiser tick rate: the consumer thread reads TrackerCache and calls
# rr.log at this rate, independent of how fast the producer pumps xrt.
# 60 Hz matches typical displays without burning gRPC bandwidth.
TRACKER_VIS_HZ = 60
# Trail re-log is the heaviest rr.log call in the visualiser (up to
# TRACKER_TRAIL_MAX points × 3 floats); decimating it independently from
# the per-tick Transform3D keeps the ellipsoid pose updates light.
TRAIL_LOG_HZ = 30
GRIPPER_STREAMS = ["imu", "encoder", "tac0", "tac1", "wrist"]
# One observed-fps stream per side. Counters bumped in the poller, scalar
# emitted from the 1 Hz status loop (same path as gripper streams) so the
# "observed fps" panel stays semantically clean — the previous
# tracker_x/y/z entries were *positions*, not FPS, and mixing them into
# the same panel produced nonsense traces.
TRACKER_STREAMS = ["tracker"]

# ---------------------- CLI ------------------------------------------------ #


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument("--left-tracker-sn", default=None,
                   help="Pico motion-tracker SN bound to the LEFT gripper.")
    p.add_argument("--right-tracker-sn", default=None,
                   help="Pico motion-tracker SN bound to the RIGHT gripper.")
    p.add_argument("--no-tracker", action="store_true",
                   help="Skip Pico SDK entirely; show only gripper streams.")
    p.add_argument("--with-raw", action="store_true",
                   help="Also log the unrectified tactile streams. Off by "
                        "default — most downstream tooling consumes the "
                        "rectified frames, and dropping `raw` ~halves "
                        "tactile bandwidth for tighter viewer latency.")
    p.add_argument("--imu-hz", type=int, default=IMU_HZ,
                   help=f"Requested IMU stream rate Hz (default {IMU_HZ}).")
    p.add_argument("--encoder-hz", type=int, default=ENCODER_HZ,
                   help=f"Requested encoder stream rate Hz (default {ENCODER_HZ}).")
    p.add_argument("--tracker-poll-hz", type=float, default=TRACKER_POLL_HZ,
                   help=f"Tracker poll rate Hz (default {TRACKER_POLL_HZ}).")
    return p.parse_args()


# ---------------------- Gripper handlers ----------------------------------- #


def _log_image_compressed(path: str, bgr: np.ndarray) -> None:
    """Encode `bgr` (BGR8, SDK-native) as JPEG and log as EncodedImage.

    `cv2.imencode` already expects BGR input — it handles BGR→YCbCr
    internally per the OpenCV convention. Pre-swapping to RGB here
    would land the file with R/B inverted, since imencode would then
    treat the swapped data as if it were BGR. So: pass `bgr` straight
    through; the JPEG on the wire is colour-correct and rerun decodes
    it as RGB on the viewer side.
    """
    if bgr.ndim != 3 or bgr.shape[2] != 3:
        # Mono / unusual shape — fall back to raw to avoid silent corruption.
        rr.log(path, rr.Image(bgr))
        return
    ok, buf = cv2.imencode(".jpg", bgr, _JPEG_ENCODE_PARAMS)
    if not ok:
        rr.log(path, rr.Image(bgr[..., ::-1], color_model="rgb"))
        return
    rr.log(path, rr.EncodedImage(contents=bytes(buf), media_type="image/jpeg"))


class StreamCounters:
    """Per-(side, stream) frame counter, shared across all SDK callbacks."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._counts: dict[tuple[str, str], int] = defaultdict(int)

    def bump(self, side: str, stream: str) -> None:
        with self._lock:
            self._counts[(side, stream)] += 1

    def snapshot(self) -> dict[tuple[str, str], int]:
        with self._lock:
            return dict(self._counts)


def make_imu_handler(side: str, counters: StreamCounters, set_time):
    def h(s):
        counters.bump(side, "imu")
        set_time()
        a, g, m = s.accel_mps2, s.gyro_radps, s.mag_uT
        rr.log(f"/{side}/imu/accel/x", rr.Scalars(float(a[0])))
        rr.log(f"/{side}/imu/accel/y", rr.Scalars(float(a[1])))
        rr.log(f"/{side}/imu/accel/z", rr.Scalars(float(a[2])))
        rr.log(f"/{side}/imu/gyro/x", rr.Scalars(float(g[0])))
        rr.log(f"/{side}/imu/gyro/y", rr.Scalars(float(g[1])))
        rr.log(f"/{side}/imu/gyro/z", rr.Scalars(float(g[2])))
        rr.log(f"/{side}/imu/mag/x", rr.Scalars(float(m[0])))
        rr.log(f"/{side}/imu/mag/y", rr.Scalars(float(m[1])))
        rr.log(f"/{side}/imu/mag/z", rr.Scalars(float(m[2])))
        rr.log(f"/{side}/imu/temperature", rr.Scalars(float(s.temperature_c)))
    return h


def make_encoder_handler(side: str, counters: StreamCounters, set_time):
    def h(s):
        counters.bump(side, "encoder")
        set_time()
        rr.log(f"/{side}/encoder/position", rr.Scalars(float(s.position_rad)))
        rr.log(f"/{side}/encoder/velocity", rr.Scalars(float(s.velocity_rad_s)))
    return h


def make_tactile_handler(side: str, idx: int, counters: StreamCounters,
                         set_time, with_raw: bool):
    def h(f):
        counters.bump(side, f"tac{idx}")
        set_time()
        if with_raw:
            _log_image_compressed(f"/{side}/{idx}_tactile/raw", f.raw)
        if f.rectified.size > 0:
            _log_image_compressed(f"/{side}/{idx}_tactile/rectified",
                                  f.rectified)
    return h


def make_wrist_handler(side: str, counters: StreamCounters, set_time):
    def h(f):
        counters.bump(side, "wrist")
        set_time()
        _log_image_compressed(f"/{side}/wrist_cam", f.image)
    return h


# ---------------------- Tracker producer / cache / consumer ---------------- #


class TrackerCache:
    """Producer→consumer slot for the latest tracker pose per side.

    The xrt poller writes here every time fresh data arrives; the
    visualiser thread reads at its own cadence. Decoupling them means a
    brief gRPC stall in rr.log can't backpressure the xrt drain — the
    poller keeps pumping into this cache and the viewer just sees one
    update collapsed to "latest" when it next ticks.
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        # side → pose7 tuple (x,y,z,qx,qy,qz,qw). Persists across reads
        # so a late consumer always has *some* pose to render.
        self._latest: dict[str, tuple] = {}
        # Sides updated since the last drain_new(); consumer skips
        # sides not in this set so we don't re-log identical poses.
        self._dirty: set[str] = set()
        # Per-side breadcrumb buffer; producer appends, consumer reads
        # the full deque under the lock when it's time to redraw.
        self._trails: dict[str, deque] = {
            s: deque(maxlen=TRACKER_TRAIL_MAX) for s in ("left", "right")
        }

    def put(self, side: str, pose7: tuple) -> None:
        with self._lock:
            self._latest[side] = pose7
            self._dirty.add(side)
            self._trails[side].append([pose7[0], pose7[1], pose7[2]])

    def drain_new(self) -> dict[str, tuple]:
        """Return {side: pose7} for sides updated since the previous call,
        clearing the dirty set. {} means "no new samples this tick".
        """
        with self._lock:
            out = {s: self._latest[s] for s in self._dirty}
            self._dirty.clear()
            return out

    def trail_snapshot(self, side: str) -> list:
        with self._lock:
            return list(self._trails[side])


class TrackerPoller(threading.Thread):
    """Producer thread: drains the xrt SDK at its native rate and writes
    every fresh sample to TrackerCache. Never calls rr.log — that's the
    visualiser's job. Lifecycle is independent of the gripper SDK
    callbacks so a hung Pico Service can't stall sensor streams.
    """

    def __init__(
        self,
        sn_to_side: dict[str, str],
        cache: TrackerCache,
        counters: StreamCounters,
        poll_hz: float,
    ) -> None:
        super().__init__(daemon=True, name="pico-tracker-poller")
        self._sn_to_side = sn_to_side
        self._cache = cache
        self._counters = counters
        self._period = 1.0 / max(1e-3, poll_hz)
        self._stop_evt = threading.Event()
        # Log "tracker SN X not seen" at most once per ~5s instead of spamming.
        self._missing_warned_at: dict[str, float] = {}

    def stop(self) -> None:
        self._stop_evt.set()

    def run(self) -> None:
        try:
            xrt.init()
            log.info("[pico] xrt.init OK")
        except Exception as e:
            log.warning(f"[pico] xrt.init failed: {type(e).__name__}: {e} — "
                        "tracker poses disabled")
            return

        try:
            while not self._stop_evt.wait(self._period):
                self._tick()
        finally:
            try:
                xrt.close()
                log.info("[pico] xrt.close OK")
            except Exception as e:
                log.warning(f"[pico] xrt.close failed: {type(e).__name__}: {e}")

    def _tick(self) -> None:
        try:
            n = xrt.num_motion_data_available()
        except Exception as e:
            log.warning(f"[pico] num_motion_data_available failed: {e}")
            return
        if n == 0:
            return

        try:
            poses = xrt.get_motion_tracker_pose()
            sns = xrt.get_motion_tracker_serial_numbers()
        except Exception as e:
            log.warning(f"[pico] tracker pose query failed: {e}")
            return

        seen: set[str] = set()
        for i in range(n):
            sn_raw = sns[i] if i < len(sns) else None
            if sn_raw is None:
                continue
            sn = sn_raw.decode() if isinstance(sn_raw, bytes) else str(sn_raw)
            seen.add(sn)
            side = self._sn_to_side.get(sn)
            if side is None:
                continue
            p = poses[i]
            pose7 = (
                float(p[0]), float(p[1]), float(p[2]),
                float(p[3]), float(p[4]), float(p[5]), float(p[6]),
            )
            self._cache.put(side, pose7)
            self._counters.bump(side, "tracker")

        # Warn if a configured SN is missing for the first time / again
        # after a long absence — easier to debug pairing issues this way.
        now = time.monotonic()
        for sn, side in self._sn_to_side.items():
            if sn in seen:
                continue
            last = self._missing_warned_at.get(sn, 0.0)
            if now - last >= 5.0:
                log.warning(f"[pico] tracker SN={sn} (configured as {side}) "
                            "not currently visible to PC Service")
                self._missing_warned_at[sn] = now


class TrackerVisualiser(threading.Thread):
    """Consumer thread: reads TrackerCache at TRACKER_VIS_HZ and pushes
    rr.log calls for the 3D view. Decoupled from the xrt drain so a
    transient gRPC flush stall blocks only this thread, never the
    producer or the gripper SDK callbacks.
    """

    def __init__(
        self,
        cache: TrackerCache,
        side_to_sn: dict[str, str],
        set_time,
        vis_hz: float,
    ) -> None:
        super().__init__(daemon=True, name="tracker-visualiser")
        self._cache = cache
        self._side_to_sn = side_to_sn
        self._set_time = set_time
        self._period = 1.0 / max(1e-3, vis_hz)
        self._stop_evt = threading.Event()
        # Static-per-side bits (mesh, label, axes) are logged the first
        # time each side appears, then live in the parent Transform3D's
        # local frame — every subsequent tick only updates the transform.
        self._static_logged: set[str] = set()
        # Trail re-log rate-limit (decoupled from per-tick transform).
        self._trail_log_period = 1.0 / max(1.0, TRAIL_LOG_HZ)
        self._last_trail_log_at: dict[str, float] = {}

    def stop(self) -> None:
        self._stop_evt.set()

    def run(self) -> None:
        while not self._stop_evt.wait(self._period):
            self._tick()

    def _tick(self) -> None:
        new = self._cache.drain_new()
        if not new:
            return
        self._set_time()
        for side, pose7 in new.items():
            self._log_static_once(side)
            self._log_pose(side, pose7)
            self._maybe_log_trail(side)

    def _log_static_once(self, side: str) -> None:
        if side in self._static_logged:
            return
        ent = f"world/{side}_gripper"
        color = (255, 80, 80, 220) if side == "left" else (80, 160, 255, 220)
        axes_len = 0.10
        sn_label = self._side_to_sn.get(side, "?")
        rr.log(f"{ent}/mesh", rr.Ellipsoids3D(
            centers=[[0.0, 0.0, 0.0]],
            half_sizes=[[0.035, 0.035, 0.02]],
            colors=[color],
        ))
        rr.log(f"{ent}/axes", rr.Arrows3D(
            origins=[[0, 0, 0], [0, 0, 0], [0, 0, 0]],
            vectors=[[axes_len, 0, 0],
                     [0, axes_len, 0],
                     [0, 0, axes_len]],
            colors=[[255, 80, 80], [80, 255, 80], [80, 80, 255]],
            radii=0.005,
        ))
        rr.log(f"{ent}/label", rr.Points3D(
            [[0, 0.10, 0]],
            labels=[f"{side.upper()} ({sn_label})"],
            colors=[color[:3]],
            radii=0.004,
        ))
        self._static_logged.add(side)

    def _log_pose(self, side: str, pose7: tuple) -> None:
        x, y, z, qx, qy, qz, qw = pose7
        rr.log(f"world/{side}_gripper", rr.Transform3D(
            translation=[x, y, z],
            quaternion=rr.Quaternion(xyzw=[qx, qy, qz, qw]),
        ))

    def _maybe_log_trail(self, side: str) -> None:
        now = time.monotonic()
        if now - self._last_trail_log_at.get(side, 0.0) < self._trail_log_period:
            return
        trail = self._cache.trail_snapshot(side)
        if not trail:
            return
        color = (255, 80, 80) if side == "left" else (80, 160, 255)
        rr.log(f"world/trails/{side}",
               rr.Points3D(trail, colors=[color], radii=0.004))
        self._last_trail_log_at[side] = now


# ---------------------- Rerun blueprint ------------------------------------ #


def _side_panel(side: str) -> rrb.ContainerLike:
    return rrb.Vertical(
        rrb.Horizontal(
            rrb.Vertical(
                rrb.Spatial2DView(name=f"{side}/wrist",
                                  origin=f"/{side}/wrist_cam"),
                rrb.Spatial2DView(name=f"{side}/tac0 (rect)",
                                  origin=f"/{side}/0_tactile/rectified"),
                rrb.Spatial2DView(name=f"{side}/tac1 (rect)",
                                  origin=f"/{side}/1_tactile/rectified"),
                row_shares=[1, 1, 1],
            ),
            rrb.Vertical(
                rrb.TimeSeriesView(name=f"{side}/encoder",
                                   origin=f"/{side}/encoder"),
                rrb.TimeSeriesView(name=f"{side}/accel",
                                   origin=f"/{side}/imu/accel"),
                rrb.TimeSeriesView(name=f"{side}/gyro",
                                   origin=f"/{side}/imu/gyro"),
                rrb.TimeSeriesView(name=f"{side}/mag",
                                   origin=f"/{side}/imu/mag"),
                row_shares=[1, 1, 1, 1],
            ),
            column_shares=[3, 2],
        ),
        rrb.TimeSeriesView(
            name=f"{side}/observed fps",
            origin=f"/perf/{side}",
            contents=[f"+ $origin/{s}" for s in GRIPPER_STREAMS]
                     + [f"+ $origin/{s}" for s in TRACKER_STREAMS],
        ),
        row_shares=[5, 1],
    )


def build_blueprint(with_world_view: bool) -> rrb.Blueprint:
    dual = rrb.Horizontal(_side_panel("left"), _side_panel("right"),
                          column_shares=[1, 1])
    if not with_world_view:
        return rrb.Blueprint(dual,
                             rrb.BlueprintPanel(state="collapsed"),
                             rrb.TimePanel(state="collapsed"))
    return rrb.Blueprint(
        rrb.Vertical(
            rrb.Spatial3DView(name="world", origin="/world"),
            dual,
            row_shares=[2, 5],
        ),
        rrb.BlueprintPanel(state="collapsed"),
        rrb.TimePanel(state="collapsed"),
    )


def init_world_scene() -> None:
    """Static scene (axes + grid) for the 3D view. Pico is LEFT-handed Y-up."""
    rr.log("world", rr.ViewCoordinates.LEFT_HAND_Y_UP, static=True)
    origin = np.array([0.0, 0.0, 0.0])
    axis_len, axis_rad = 0.6, 0.012
    rr.log("world/origin/axes",
           rr.Arrows3D(
               origins=[origin, origin, origin],
               vectors=[[axis_len, 0, 0], [0, axis_len, 0], [0, 0, axis_len]],
               colors=[[255, 50, 50], [50, 255, 50], [50, 50, 255]],
               radii=axis_rad,
           ),
           static=True)
    # Y=0 ground grid (left-handed Y-up).
    side, lines = 2.4, 25
    grid = []
    for i in range(lines):
        t = -side / 2 + i * side / (lines - 1)
        grid.append([[t, 0, -side / 2], [t, 0, side / 2]])
        grid.append([[-side / 2, 0, t], [side / 2, 0, t]])
    rr.log("world/grid",
           rr.LineStrips3D(grid, colors=[[110, 110, 110, 90]]),
           static=True)


# ---------------------- Main flow ------------------------------------------ #


def _open_gripper(eps) -> LeaderGripper:
    return LeaderGripper(
        eps.mcu_device,
        eps.wrist_video,
        eps.tactile_left_serial,
        eps.tactile_right_serial,
    )


def main() -> int:
    args = parse_args()

    use_tracker = not args.no_tracker
    if use_tracker and not _HAS_PICO_SDK:
        log.warning("[pico] xensevr_pc_service_sdk is not importable — "
                    "running without tracker (use --no-tracker to silence).")
        use_tracker = False
    if use_tracker and not (args.left_tracker_sn or args.right_tracker_sn):
        log.warning("[pico] no --left/--right-tracker-sn given — running "
                    "without tracker (specify SNs to enable).")
        use_tracker = False

    sn_to_side: dict[str, str] = {}
    if use_tracker:
        if args.left_tracker_sn:
            sn_to_side[args.left_tracker_sn] = "left"
        if args.right_tracker_sn:
            sn_to_side[args.right_tracker_sn] = "right"

    # ---- Gripper discovery + open ----
    log.info("[discovery] scanning for both sides ...")
    eps_L = find_left()
    eps_R = find_right()
    log.info(f"[discovery] L: mcu={eps_L.mcu_serial} fw={eps_L.firmware_sn}")
    log.info(f"[discovery] R: mcu={eps_R.mcu_serial} fw={eps_R.firmware_sn}")

    log.info("[open] constructing both LeaderGrippers ...")
    gL = _open_gripper(eps_L)
    gR = _open_gripper(eps_R)
    log.info("[open] both LeaderGrippers ready")

    # ---- Rerun init ----
    rr.init("TacCap-Gripper-Dual-Tracker", spawn=True)
    log.info("[rerun] spawned viewer")
    rr.send_blueprint(build_blueprint(with_world_view=use_tracker))
    if use_tracker:
        init_world_scene()
    for side in ("left", "right"):
        for s in GRIPPER_STREAMS:
            rr.log(f"/perf/{side}/{s}", rr.Scalars(0.0))
        if use_tracker:
            for s in TRACKER_STREAMS:
                rr.log(f"/perf/{side}/{s}", rr.Scalars(0.0))

    # ---- Shared time + counters ----
    counters = StreamCounters()
    mono_start = time.monotonic()

    def set_time() -> None:
        rr.set_time(timeline="host", duration=time.monotonic() - mono_start)

    # ---- Tracker producer + consumer ----
    poller: Optional[TrackerPoller] = None
    viz: Optional[TrackerVisualiser] = None
    if use_tracker:
        side_to_sn = {v: k for k, v in sn_to_side.items()}
        cache = TrackerCache()
        poller = TrackerPoller(sn_to_side, cache, counters,
                               args.tracker_poll_hz)
        viz = TrackerVisualiser(cache, side_to_sn, set_time, TRACKER_VIS_HZ)
        poller.start()
        viz.start()
        log.info(f"[pico] tracker poller ({args.tracker_poll_hz:.0f} Hz) + "
                 f"visualiser ({TRACKER_VIS_HZ} Hz) started — "
                 f"mapping {sn_to_side}")

    # ---- Subscribe callbacks ----
    try:
        for g, side in ((gL, "left"), (gR, "right")):
            log.info(f"[subscribe] {side} callbacks attached")
            g.imu.on_data(make_imu_handler(side, counters, set_time))
            g.encoder.on_data(make_encoder_handler(side, counters, set_time))
            g.tactile_left.start(make_tactile_handler(side, 0, counters, set_time, args.with_raw))
            g.tactile_right.start(make_tactile_handler(side, 1, counters, set_time, args.with_raw))
            g.wrist_camera.start(make_wrist_handler(side, counters, set_time))

        log.info(f"[stream] starting both sides "
                 f"(imu={args.imu_hz}Hz, enc={args.encoder_hz}Hz) ...")
        gL.start_streaming(imu_hz=args.imu_hz, encoder_hz=args.encoder_hz)
        gR.start_streaming(imu_hz=args.imu_hz, encoder_hz=args.encoder_hz)
        mono_start = time.monotonic()
        # Promote everything allocated during startup to the permanent
        # gen-2 set so long sessions don't pay a full-heap GC sweep every
        # few seconds — that sweep used to manifest as the 3D ellipsoid
        # freezing for ~100 ms while other panels (lots of historical
        # points) hid the pause visually.
        gc.freeze()
        log.info("[run] streaming; SIGINT/SIGTERM to stop")

        # ---- Status loop ----
        last_t = time.time()
        last_snap: dict[tuple[str, str], int] = defaultdict(int)
        while True:
            time.sleep(1.0)
            now = time.time()
            elapsed = now - last_t
            snap = counters.snapshot()
            set_time()
            for (side, stream), n in snap.items():
                delta = n - last_snap[(side, stream)]
                fps = delta / elapsed if elapsed > 0 else 0.0
                # Both gripper streams and the tracker fps land here so
                # the "observed fps" panel sees one consistent 1 Hz feed
                # per channel (poller no longer emits per-axis scalars).
                if stream in GRIPPER_STREAMS or stream in TRACKER_STREAMS:
                    rr.log(f"/perf/{side}/{stream}", rr.Scalars(fps))
            last_snap = snap
            last_t = now

            parts = []
            for side in ("left", "right"):
                sub = {k[1]: v for k, v in snap.items() if k[0] == side}
                ssum = " ".join(f"{k}={sub.get(k, 0):5d}" for k in GRIPPER_STREAMS)
                tr = f" trk={sub.get('tracker', 0):5d}" if use_tracker else ""
                parts.append(f"[{side[0].upper()}] {ssum}{tr}")
            log.info("  ".join(parts))

    except KeyboardInterrupt:
        log.info("[done] SIGINT received")

    finally:
        log.info("[stop] stopping both sides ...")
        # Visualiser first: stops rr.log calls so poller's last samples
        # don't fight a shutting-down rerun stream.
        if viz is not None:
            viz.stop()
            viz.join(timeout=2.0)
        if poller is not None:
            poller.stop()
            poller.join(timeout=2.0)
        for label, g in (("L", gL), ("R", gR)):
            for fn in (
                lambda gg=g: gg.tactile_left.stop(),
                lambda gg=g: gg.tactile_right.stop(),
                lambda gg=g: gg.wrist_camera.stop(),
                lambda gg=g: gg.stop_streaming() if gg.is_streaming else None,
            ):
                try:
                    fn()
                except Exception as e:
                    log.warning(f"[{label}] during stop: {type(e).__name__}: {e}")

        elapsed = time.time() - mono_start
        log.info(f"[summary] {elapsed:.2f}s total")
        snap = counters.snapshot()
        for (side, stream), n in sorted(snap.items()):
            log.info(f"  {side:5s} {stream:8s} {n:7d} frames  "
                     f"({n / max(elapsed, 1e-3):6.1f} fps)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
