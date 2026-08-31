#!/usr/bin/env python3
# Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
"""MIT-only gripper console with force-position control.

The ``xense.taccap`` SDK remains the transport/device layer (discovery,
status, streaming and MIT frame serialization).  The controller state machine
below is implemented in this script; it uses the SDK for device access and
MIT command transmission without instantiating the SDK ``ForcePositionController``
class.

The command-line habits are intentionally the same as the old console::

    python python/examples/gripper_console.py --mode force-position \
        --grasp-torque 1.2

Keys::

    j / k   + / - change the target by one angular increment; long press follows repeat
    o       release to fully open
    c       close to the calibrated zero and grasp on contact
    h       position-hold the current jaw position
    e / d   enable / disable the motor
    f       clear a motor fault and reset position hold
    q / ESC  quit

Only the MIT protocol is accepted.  The close/release phases use the MIT
impedance command with a velocity feed-forward term.  Private-protocol
position planning is not silently mixed into this example.
"""

from __future__ import annotations

import argparse
import fcntl
import math
import os
import queue
import select
import sys
import threading
import termios
import time
import tty
from typing import Optional

from xense.taccap import (
    FollowerGripper,
    GRIPPER_ENVELOPE_ENFORCE,
    GRIPPER_ENVELOPE_VALID,
    MotorProtocol,
    find_follower,
    find_left,
    find_right,
    log,
)


_STATUS_BITS = (
    (1, "EN"),
    (2, "FAULT"),
    (4, "STALL"),
    (8, "OVER_TEMP"),
    (16, "OVER_CURR"),
    (32, "OVER_VOLT"),
    (64, "UNDER_VOLT"),
    (128, "ENC_ERR"),
)

# These are the force-position control constants used by this example.
HYBRID_INTERVAL_S = 0.050
# Point-jog needs a finer host tick than the 50 ms hybrid phase cadence: at
# 0.8 rad/s a 0.020 rad step lasts only 25 ms. A 5 ms tick prevents the host
# from overshooting the step and repeatedly stopping/restarting the jaw.
JOG_TICK_INTERVAL_S = 0.005
# MIT submit_impedance() is deliberately fire-and-forget (no ACK).  Keep
# resending the active phase command at a fixed cadence so a
# single lost UART frame cannot leave a key press with no motor response.
COMMAND_RESEND_INTERVAL_S = HYBRID_INTERVAL_S
# A tty has no key-up event. Each received j/k byte advances a position target
# by this raw-angle increment; terminal auto-repeat makes a long press follow.
JOG_STEP_RAD = 0.020
JOG_INPUT_TIMEOUT_S = 0.65
HYBRID_STATUS_MAX_AGE_S = 0.350
HYBRID_CONTACT_COUNT = 2
HYBRID_CONTACT_TORQUE_MIN_NM = 0.25
HYBRID_CONTACT_TORQUE_RATIO = 0.35
HYBRID_CONTACT_TORQUE_MAX_NM = 1.20
HYBRID_FORCE_LIMIT_PROTECT_NM = 6.0
HYBRID_RELEASE_STEP_RAD = 0.006
HYBRID_CLOSING_FORCE_GAIN = 0.65
# The current motor/assembly reports a repeatable positive torque bias of about
# 0.05 Nm in the final force-hold feedback. Compensate it once in the command;
# do not use an accumulating feedback loop, which can drive the hold torque far
# below the requested value after a transient.
HOLD_TORQUE_OFFSET_DEFAULT_NM = 0.05
CLOSING_BRAKE_RAD = 0.10
RELEASE_BRAKE_RAD = 0.15
RELEASE_KD_VEL = 1.5
RELEASE_KD_BRAKE = 3.0
RELEASE_TIMEOUT_S = 8.0
RELEASE_ARRIVE_RAD = 0.03
# A single j/k step is normally much smaller than the full-release arrival
# tolerance.  Use a separate tight tolerance for point-jogging; otherwise the
# first tick after `j` would see the small target increment as already arrived,
# immediately switch to position hold, and make opening feel like a pause.
JOG_ARRIVE_RAD = 0.003
STARTUP_GUARD_S = 0.250

# MotorStatusBit::Stalled is a normal successful grasp condition.  It must not
# be treated as a fault; all the other known safety bits stop this controller.
SERIOUS_FAULT_MASK = 2 | 8 | 16 | 32 | 64 | 128


def _status_str(mask: Optional[int]) -> str:
    if mask is None:
        return "-"
    if mask == 0:
        return "DISABLED"
    names = [name for bit, name in _STATUS_BITS if mask & bit]
    return "|".join(names) if names else f"0x{mask:02X}"


class RawKeyboard:
    """Capture terminal input independently from motor control.

    The previous implementation read stdin from the control/UI thread.  A
    short serial write or terminal redraw could then delay the read and make a
    key appear to be ignored.  A dedicated reader drains the tty continuously
    into an unbounded queue; control consumes the queue at its own cadence.
    """

    def __init__(self):
        self._keys: queue.SimpleQueue[str] = queue.SimpleQueue()
        # `Event.clear()` must be serialized with the reader's `put()`. Without
        # this lock, a byte arriving between queue-empty detection and clear()
        # could leave the event cleared until the next control timeout.
        self._queue_lock = threading.Lock()
        self._wake = threading.Event()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._fd: Optional[int] = None

    def __enter__(self):
        self._fd = sys.stdin.fileno()
        self._saved = termios.tcgetattr(sys.stdin.fileno())
        tty.setraw(sys.stdin.fileno())
        flags = fcntl.fcntl(sys.stdin.fileno(), fcntl.F_GETFL)
        fcntl.fcntl(sys.stdin.fileno(), fcntl.F_SETFL, flags | os.O_NONBLOCK)
        self._thread = threading.Thread(
            target=self._reader_loop, name="gripper-keyboard", daemon=True
        )
        self._thread.start()
        return self

    def __exit__(self, *_):
        self._stop.set()
        self._wake.set()
        if self._thread is not None:
            self._thread.join(timeout=0.25)
        termios.tcsetattr(sys.stdin.fileno(), termios.TCSANOW, self._saved)
        flags = fcntl.fcntl(sys.stdin.fileno(), fcntl.F_GETFL)
        fcntl.fcntl(sys.stdin.fileno(), fcntl.F_SETFL, flags & ~os.O_NONBLOCK)

    def _reader_loop(self) -> None:
        fd = self._fd
        if fd is None:
            return
        while not self._stop.is_set():
            try:
                ready, _, _ = select.select([fd], [], [], 0.050)
                if not ready:
                    continue
                # Drain until the tty says there is no more data.  There is no
                # fixed queue limit, so rapid key-repeat bursts are retained.
                while not self._stop.is_set():
                    try:
                        value = os.read(fd, 256)
                    except (BlockingIOError, InterruptedError):
                        break
                    if not value:
                        break
                    with self._queue_lock:
                        for ch in value.decode("utf-8", errors="replace"):
                            self._keys.put(ch)
                        self._wake.set()
                    if len(value) < 256:
                        break
            except (OSError, ValueError):
                if not self._stop.is_set():
                    time.sleep(0.005)
                break

    def poll_many(self) -> list[str]:
        out: list[str] = []
        with self._queue_lock:
            while True:
                try:
                    out.append(self._keys.get_nowait())
                except queue.Empty:
                    # The reader cannot enqueue between the empty check and
                    # clear(); it must wait for this lock and then set wake.
                    self._wake.clear()
                    return out

    def wait(self, timeout: float) -> None:
        # The reader thread wakes this event as soon as input arrives.  The
        # timeout still lets the 50 ms controller cadence and UI refresh run.
        self._wake.wait(max(0.0, timeout))


class MitForcePositionController:
    """MIT-only closing/release state machine.

    ``on_status`` supplies the latest stream sample.  The 50 ms ``tick`` is
    the controller cadence: it evaluates contact/arrival and sends
    only the phase command that is currently needed.  MIT commands are
    fire-and-forget through the SDK transport.
    """

    def __init__(self, gripper: FollowerGripper, args):
        self.g = gripper
        self.motor = gripper.motor
        self.map = gripper.position_map()
        if not self.map.valid:
            raise RuntimeError("gripper is not calibrated; normalized position is unavailable")

        self.kp = float(args.kp)
        self.kd = float(args.kd)
        self.close_speed = float(args.close_speed)
        self.grasp_torque = float(args.grasp_torque)
        self.contact_torque = float(args.contact_torque)
        self.force_limit = float(args.force_limit)
        self.release_speed = float(args.release_speed)
        self.jog_step_rad = float(args.step_rad)
        # Keep direct programmatic construction compatible with older callers
        # that do not yet add the new CLI field to their Namespace.
        self.hold_torque_offset = float(
            getattr(args, "hold_torque_offset", HOLD_TORQUE_OFFSET_DEFAULT_NM)
        )

        if self.kp <= 0.0 or self.kd < 0.0:
            raise ValueError("--kp must be > 0 and --kd must be >= 0")
        if self.close_speed <= 0.0 or self.release_speed <= 0.0:
            raise ValueError("close/release speed must be > 0")
        if self.jog_step_rad <= 0.0:
            raise ValueError("--step-rad must be > 0")
        if not math.isfinite(self.hold_torque_offset) or self.hold_torque_offset < 0.0:
            raise ValueError("--hold-torque-offset must be >= 0")
        if self.grasp_torque <= 0.0:
            raise ValueError("--grasp-torque must be > 0")
        if not 0.0 < self.contact_torque <= self.grasp_torque:
            raise ValueError("--contact-torque must be in (0, --grasp-torque]")
        if not 0.0 < self.force_limit <= HYBRID_FORCE_LIMIT_PROTECT_NM:
            raise ValueError("--force-limit must be in (0, 6.0]")

        self.state = "pos_hold"
        self.target_open = 0.0
        self.hold_open = 0.0
        self.hold_torque = 0.0
        self.contact_count = 0
        self.phase = "move"
        self.command_sent = False
        self.started_at = 0.0
        self.release_started_at = 0.0
        self.pos_hold_sent = False
        self.fault_reason = ""
        self.commanded_torque = 0.0
        self.last_command = "-"
        self.command_seq = 0
        self.submit_failures = 0
        self.last_submit_error = ""
        self.last_submit_at = 0.0
        self.jog_direction = 0.0
        self.jog_active = False
        self.jog_target_open = 0.0
        self.jog_contact_count = 0

        self._latest = None
        self._latest_at = 0.0
        self._subscription = None
        self._stream_started = False

    # ---- transport/lifecycle ------------------------------------------

    def _on_status(self, sample) -> None:
        self._latest = sample
        self._latest_at = time.monotonic()

    def _sample(self):
        return self._latest, self._latest_at

    def start(self) -> None:
        protocol = self.motor.get_protocol()
        if protocol != MotorProtocol.Mit:
            raise RuntimeError(
                f"MIT-only console: device protocol is {protocol}; "
                "switch the motor to MIT and power-cycle it before use"
            )

        initial = self.motor.read_status(500)
        self._latest = initial
        self._latest_at = time.monotonic()
        self.target_open = self.map.to_position(initial.actual_pos)
        self.hold_open = self.target_open
        self.jog_target_open = self.target_open
        self.jog_active = False
        self.jog_direction = 0.0
        self.state = "pos_hold"
        self.pos_hold_sent = False

        self._subscription = self.motor.on_status(self._on_status)
        try:
            self.g.start_streaming(100)
            self._stream_started = True
        except Exception:
            if self._subscription is not None:
                self.motor.off(self._subscription)
                self._subscription = None
            raise

    def stop(self) -> None:
        self.jog_active = False
        self.jog_direction = 0.0
        sample, _ = self._sample()
        if sample is not None:
            try:
                self.motor.submit_impedance(sample.actual_pos, 0.0, 0.0, 0.0, 0.0)
            except Exception:
                pass
        if self._subscription is not None:
            try:
                self.motor.off(self._subscription)
            except Exception:
                pass
            self._subscription = None
        if self._stream_started:
            try:
                self.g.stop_streaming()
            except Exception:
                pass
            self._stream_started = False

    # ---- command primitives -------------------------------------------

    @property
    def direction_open_raw(self) -> float:
        return -1.0 if self.map.reverse else 1.0

    @property
    def direction_close_raw(self) -> float:
        return -self.direction_open_raw

    def _submit(self, raw_target: float, kp: float, kd: float,
                torque: float, velocity: float = 0.0,
                display_target_raw: Optional[float] = None) -> None:
        # No-ACK frames are safe to retry: the MCU keeps the latest command and
        # does not execute a command twice as an ACK request would. A transient
        # serial write error therefore gets one immediate retry; persistent
        # transport failures are still raised so the safety path can stop.
        payload = (
            float(raw_target), float(kp), float(kd), float(torque), float(velocity)
        )
        try:
            self.motor.submit_impedance(*payload)
        except Exception as first_exc:
            self.submit_failures += 1
            self.last_submit_error = f"{type(first_exc).__name__}: {first_exc}"
            try:
                self.motor.submit_impedance(*payload)
            except Exception as second_exc:
                self.submit_failures += 1
                self.last_submit_error = f"{type(second_exc).__name__}: {second_exc}"
                raise
        self.last_submit_error = ""
        self.command_seq += 1
        self.last_submit_at = time.monotonic()
        if display_target_raw is None:
            display_target_raw = raw_target
        self.last_command = (
            f"seq={self.command_seq} target_open={self.target_open:.4f} "
            f"pos={raw_target:.4f} target={display_target_raw:.4f} "
            f"kp={kp:.2f} kd={kd:.2f} "
            f"tau={torque:.3f} vel={velocity:.3f}"
        )

    def _send_position_hold(self, raw_target: float) -> None:
        self._submit(raw_target, self.kp, self.kd, 0.0, 0.0)
        self.commanded_torque = 0.0

    def _send_close_move(self, current_raw: float) -> None:
        # Velocity-damping formula.  Kp=0 makes the velocity phase immune
        # to a growing position error; Kd is capped at 5 and scaled by .65.
        kd_closing = max(
            0.3,
            min(5.0, self.grasp_torque /
                max(self.close_speed * HYBRID_CLOSING_FORCE_GAIN, 0.05)),
        )
        velocity = self.direction_close_raw * self.close_speed
        self._submit(
            current_raw, 0.0, kd_closing, 0.0, velocity,
            display_target_raw=self.map.to_rad(self.target_open),
        )
        self.commanded_torque = min(self.force_limit, kd_closing * self.close_speed)

    def _send_close_brake(self, close_raw: float) -> None:
        self._submit(
            close_raw, self.kp, self.kd, 0.0, 0.0,
            display_target_raw=close_raw,
        )
        self.commanded_torque = 0.0

    def _send_force_hold(self) -> None:
        # Normal map closes in the negative motor direction; Reverse maps close
        # in the positive direction.  Kp=Kd=0 keeps force hold independent of
        # position error.
        # Apply a fixed assembly/feedback bias correction. The result is always
        # non-negative and never exceeds the requested target or safety limit.
        command_torque = max(
            0.0,
            min(self.hold_torque, self.force_limit) - self.hold_torque_offset,
        )
        signed_torque = self.direction_close_raw * command_torque
        hold_raw = self.map.to_rad(self.hold_open)
        self._submit(
            hold_raw, 0.0, 0.0, signed_torque, 0.0,
            display_target_raw=hold_raw,
        )
        self.commanded_torque = abs(signed_torque)

    def _send_release_move(self, current_raw: float) -> None:
        self._submit(
            current_raw, 0.0, RELEASE_KD_VEL, 0.0,
            self.direction_open_raw * self.release_speed,
            display_target_raw=self.map.to_rad(self.target_open),
        )
        self.commanded_torque = min(self.force_limit, RELEASE_KD_VEL * self.release_speed)

    def _send_release_brake(self, current_raw: float) -> None:
        self._submit(
            current_raw, 0.0, RELEASE_KD_BRAKE, 0.0, 0.0,
            display_target_raw=current_raw,
        )
        self.commanded_torque = 0.0

    @property
    def contact_threshold(self) -> float:
        """Effective contact threshold used by the controller.

        The user sets a floor with ``--contact-torque``; the controller also
        keeps it above 35% of the requested grasp torque and below 1.20 Nm.
        """
        return max(
            self.contact_torque,
            HYBRID_CONTACT_TORQUE_MIN_NM,
            min(HYBRID_CONTACT_TORQUE_MAX_NM,
                self.grasp_torque * HYBRID_CONTACT_TORQUE_RATIO),
        )

    def parameters(self) -> dict:
        """Return the active motion/force parameters for the console UI."""
        return {
            "target": self.target_open,
            "close_speed": self.close_speed,
            "release_speed": self.release_speed,
            "grasp_torque": self.grasp_torque,
            "contact_floor": self.contact_torque,
            "contact_threshold": self.contact_threshold,
            "force_limit": self.force_limit,
            "kp": self.kp,
            "kd": self.kd,
            "jog_step_rad": self.jog_step_rad,
            "hold_torque_offset": self.hold_torque_offset,
        }

    # ---- user actions --------------------------------------------------

    def start_jog(self, direction_open: float) -> None:
        """Advance the target by one calibrated raw-angle increment."""
        sample, _ = self._sample()
        if sample is None:
            return
        direction_open = 1.0 if direction_open >= 0.0 else -1.0
        # Once a closing jog has made contact, keep the force hold alive.
        if self.state == "holding" and direction_open < 0.0:
            self.jog_active = True
            self.jog_direction = direction_open
            return
        current_open = self.map.to_position(sample.actual_pos)
        if (not self.jog_active or self.jog_direction != direction_open):
            # A terminal has no key-release event and the actual jaw can lag
            # behind a previous target (for example after `o`).  Starting a
            # fresh jog from that stale target can make the first `k` target
            # still be above the actual position, which sends an opening frame
            # before closing.  Always re-anchor a new/direction-switched jog
            # on measured position so the first frame follows the key.
            self.jog_target_open = current_open
        span = max(self.map.max_open_rad - self.map.min_open_rad, 1e-6)
        step_open = self.jog_step_rad / span
        next_target = max(
            0.0, min(1.0, self.jog_target_open + direction_open * step_open)
        )
        was_same_phase = (
            self.jog_active and self.jog_direction == direction_open and
            ((direction_open < 0.0 and self.state == "closing") or
             (direction_open > 0.0 and self.state == "releasing"))
        )
        self.jog_target_open = next_target
        self.jog_active = True
        self.jog_direction = direction_open
        if was_same_phase:
            # Extend the target without resetting the contact/startup timer;
            # otherwise a long-held k would keep postponing force detection.
            self.target_open = next_target
            self.command_sent = False
            self.phase = "move"
            # Push the updated target immediately. The 50 ms tick will resend
            # the phase frame as needed, but the key event must never wait for
            # a timer slot or appear to have been ignored.
            if direction_open < 0.0:
                self._send_close_move(float(sample.actual_pos))
            else:
                self._send_release_move(float(sample.actual_pos))
            return
        self.set_target(next_target)
        # The key event itself must produce a command. Do not wait for the
        # periodic safety tick, otherwise the UI can show a new target while
        # ``command`` remains unchanged for one or more frames.
        if self.state == "closing":
            self._send_close_move(float(sample.actual_pos))
            self.command_sent = True
        elif self.state == "releasing":
            self._send_release_move(float(sample.actual_pos))

    def stop_jog(self) -> None:
        """Stop manual jog and hold the current position."""
        if not self.jog_active:
            return
        self.jog_active = False
        self.jog_direction = 0.0
        # A closing jog that has contacted an object is now a force grasp, not
        # a free-running jog. Releasing the key must not drop the grasp torque.
        if self.state == "holding":
            return
        sample, _ = self._sample()
        if sample is None:
            self.state = "pos_hold"
            self.pos_hold_sent = False
            return
        self.hold_open = self.map.to_position(sample.actual_pos)
        self.target_open = self.hold_open
        self.state = "pos_hold"
        self.pos_hold_sent = False
        self.command_sent = False
        self.contact_count = 0
        self.jog_target_open = self.target_open

    def set_target(self, target_open: float) -> None:
        sample, _ = self._sample()
        if sample is None:
            return
        target_open = max(0.0, min(1.0, float(target_open)))
        current_open = self.map.to_position(sample.actual_pos)
        self.target_open = target_open
        self.contact_count = 0
        self.command_sent = False
        self.pos_hold_sent = False
        self.fault_reason = ""
        if target_open < current_open - 1e-4:
            self.state = "closing"
            self.phase = "move"
            self.started_at = time.monotonic()
        elif target_open > current_open + 1e-4:
            self.state = "releasing"
            self.release_started_at = time.monotonic()
        else:
            self.state = "pos_hold"
            self.hold_open = current_open

    def release(self) -> None:
        sample, _ = self._sample()
        if sample is None:
            return
        self.jog_active = False
        self.jog_direction = 0.0
        self.target_open = 1.0
        self.state = "releasing"
        self.release_started_at = time.monotonic()
        self.command_sent = False
        self.pos_hold_sent = False
        self.contact_count = 0

    def hold(self) -> Optional[float]:
        sample, _ = self._sample()
        if sample is None:
            return None
        self.jog_active = False
        self.jog_direction = 0.0
        self.hold_open = self.map.to_position(sample.actual_pos)
        self.target_open = self.hold_open
        self.state = "pos_hold"
        self.pos_hold_sent = False
        self.command_sent = False
        self.contact_count = 0
        self.hold_torque = 0.0
        return self.hold_open

    def reset_after_fault(self) -> None:
        sample, _ = self._sample()
        if sample is None:
            return
        self.fault_reason = ""
        self.hold_open = self.map.to_position(sample.actual_pos)
        self.target_open = self.hold_open
        self.state = "pos_hold"
        self.pos_hold_sent = False
        self.contact_count = 0

    # ---- force-position control tick ----------------------------------

    def _fault(self, reason: str) -> None:
        if self.state != "fault":
            self.state = "fault"
            self.fault_reason = reason
            self.commanded_torque = 0.0
            sample, _ = self._sample()
            if sample is not None:
                try:
                    self._submit(sample.actual_pos, 0.0, 0.0, 0.0, 0.0)
                except Exception:
                    pass

    def tick(self) -> None:
        sample, received_at = self._sample()
        if sample is None:
            return
        now = time.monotonic()
        age = now - received_at
        if age > HYBRID_STATUS_MAX_AGE_S:
            self._fault("motor status stream stale")
            return
        if not all(math.isfinite(float(v)) for v in
                   (sample.actual_pos, sample.actual_vel, sample.actual_torque)):
            self._fault("non-finite motor status")
            return
        if int(sample.status) & SERIOUS_FAULT_MASK:
            self._fault(f"motor status fault: {_status_str(int(sample.status))}")
            return
        if self.state == "fault":
            return

        current_raw = float(sample.actual_pos)
        current_open = self.map.to_position(current_raw)
        close_raw = self.map.to_rad(self.target_open)

        if self.state == "pos_hold":
            if not self.pos_hold_sent:
                self.hold_open = current_open
                self._send_position_hold(self.map.to_rad(self.hold_open))
                self.pos_hold_sent = True
            return

        if self.state == "releasing":
            if now - self.release_started_at > RELEASE_TIMEOUT_S:
                self.hold()
                return
            target_raw = self.map.to_rad(self.target_open)
            distance_raw = (target_raw - current_raw) * self.direction_open_raw
            # target_open/current_open are normalized [0,1], while these
            # tolerances are raw motor radians. Compare in raw coordinates;
            # point-jog uses the tighter tolerance so a small configured step
            # is not mistaken for an already-reached full-release target.
            # Keep the jog tolerance below the configured jog increment too;
            # users may select a step smaller than the default auto-sized step.
            jog_arrive_rad = min(JOG_ARRIVE_RAD, self.jog_step_rad * 0.25)
            arrive_rad = jog_arrive_rad if self.jog_active else RELEASE_ARRIVE_RAD
            if distance_raw <= arrive_rad:
                self.hold()
                # `hold()` changes state immediately.  Send the position-hold
                # frame in the same tick instead of leaving the last velocity
                # command active for another 50 ms; this removes the small
                # stop/restart hitch between repeated opening jog steps.
                self._send_position_hold(current_raw)
                self.pos_hold_sent = True
                return
            # The full-release brake window is 0.15 rad, but a point-jog
            # increment is only 0.020 rad by default. Applying any sizeable
            # brake window to a jog would turn the latter part of every `j`
            # step into a zero-velocity frame, causing a visible pause before
            # the next step. Point jog therefore runs velocity tracking right
            # up to its tight arrival check; full `o` release keeps the
            # configured 0.15 rad brake window.
            brake_rad = 0.0 if self.jog_active else RELEASE_BRAKE_RAD
            if distance_raw <= brake_rad:
                self._send_release_brake(current_raw)
            else:
                self._send_release_move(current_raw)
            return

        if self.state == "closing":
            actual_torque = abs(float(sample.actual_torque))
            actual_vel = abs(float(sample.actual_vel))
            threshold = self.contact_threshold
            close_distance_raw = (current_raw - close_raw) * self.direction_open_raw
            step_raw = max(0.0005, self.close_speed * HYBRID_INTERVAL_S)

            # A hard force-limit event immediately backs off by
            # 0.006 rad and switches to target force hold.
            if actual_torque >= self.force_limit:
                hold_raw = current_raw + self.direction_open_raw * HYBRID_RELEASE_STEP_RAD
                self.hold_open = self.map.to_position(hold_raw)
                self.hold_torque = min(self.grasp_torque, self.force_limit)
                self.state = "holding"
                self.contact_count = HYBRID_CONTACT_COUNT
                self.command_sent = False
                self._send_force_hold()
                return

            # Suppress the acceleration transient.  Thereafter, the controller counts
            # either torque contact or a torque+stall sample, for two frames.
            in_startup = now - self.started_at < STARTUP_GUARD_S
            contact_by_torque = actual_torque >= threshold
            contact_by_stall = (
                actual_torque >= threshold and
                actual_vel <= 0.03 and
                abs((current_raw - close_raw)) <= max(step_raw * 2.0, 0.004)
            )
            if in_startup:
                self.contact_count = 0
            elif contact_by_torque or contact_by_stall:
                self.contact_count += 1
            else:
                self.contact_count = 0

            if self.contact_count >= HYBRID_CONTACT_COUNT:
                self.state = "holding"
                self.hold_open = current_open
                self.hold_torque = min(self.grasp_torque, self.force_limit)
                self.command_sent = False
                self._send_force_hold()
                return

            # No-contact close target: use zero-feedforward PD, not force hold.
            if current_open <= self.target_open + 1e-4:
                self.state = "pos_hold"
                self.hold_open = self.target_open
                self.pos_hold_sent = False
                self._send_position_hold(close_raw)
                self.pos_hold_sent = True
                return

            # MIT has two phases. Unlike the old one-shot implementation,
            # resend the active frame every tick. submit_impedance() has no ACK
            # and a single dropped frame otherwise looks exactly like an
            # ignored key. Repeating the latest command is idempotent because
            # the firmware applies only the newest target.
            new_phase = "brake" if close_distance_raw <= CLOSING_BRAKE_RAD else "move"
            if (new_phase != self.phase or not self.command_sent or
                    now - self.last_submit_at >= COMMAND_RESEND_INTERVAL_S - 1e-6):
                if new_phase == "brake":
                    self._send_close_brake(close_raw)
                else:
                    self._send_close_move(current_raw)
                self.phase = new_phase
                self.command_sent = True
            return

        if self.state == "holding":
            self._send_force_hold()

    def snapshot(self) -> dict:
        sample, received_at = self._sample()
        now = time.monotonic()
        if sample is None:
            return {
                "state": self.state, "target": self.target_open,
                "position": None, "raw": None, "velocity": None,
                "torque": None, "temp": None, "status": None,
                "age": None, "contact": self.contact_count,
                "commanded": self.commanded_torque,
                "command_seq": self.command_seq,
                "submit_failures": self.submit_failures,
                "submit_error": self.last_submit_error,
            }
        return {
            "state": self.state,
            "target": self.target_open,
            "position": self.map.to_position(sample.actual_pos),
            "raw": float(sample.actual_pos),
            "velocity": float(sample.actual_vel),
            "torque": float(sample.actual_torque),
            "temp": float(sample.motor_temp_c),
            "status": int(sample.status),
            "age": max(0.0, now - received_at),
            "contact": self.contact_count,
            "commanded": self.commanded_torque,
            "command": self.last_command,
            "command_seq": self.command_seq,
            "submit_failures": self.submit_failures,
            "submit_error": self.last_submit_error,
            "fault": self.fault_reason,
        }


def _redraw(controller: MitForcePositionController, head: str, envline: str,
            last_key: str) -> None:
    s = controller.snapshot()
    p = controller.parameters()
    if s["position"] is None:
        row = "  waiting for motor status..."
        age = "age=—"
    else:
        row = (
            f"{s['target']:>8.3f}{s['position']:>8.3f}{s['raw']:>10.4f}"
            f"{s['velocity']:>10.3f}{s['torque']:>10.3f}{s['temp']:>8.1f}"
            f"{_status_str(s['status']):>18s}"
        )
        age = f"age={s['age'] * 1000.0:5.1f}ms"
    lines = [
        head,
        envline,
        "  j/k=+/-  o=open  c=close  h=hold  e/d=en/dis  f=fault_clear  q=quit",
        f"{'Tgt':>8}{'Act':>8}{'Act(rad)':>10}{'Vel(r/s)':>10}"
        f"{'Torq(Nm)':>10}{'Temp(C)':>8}{'State':>18}",
        "-" * 82,
        row,
        f"  state={s['state']}  contact={s['contact']}  "
        f"cmd_torque={s['commanded']:.3f}Nm  submit_seq={s.get('command_seq', 0)} "
        f"write_fail={s.get('submit_failures', 0)}  {age}",
        f"  params: step={p['jog_step_rad']:.3f}rad close={p['close_speed']:.2f}rad/s release={p['release_speed']:.2f}rad/s "
        f"grasp={p['grasp_torque']:.2f}Nm hold_offset={p['hold_torque_offset']:.2f}Nm contact={p['contact_threshold']:.2f}Nm "
        f"limit={p['force_limit']:.2f}Nm Kp={p['kp']:.1f} Kd={p['kd']:.2f}",
        f"  last key={last_key}  command={s.get('command', '-')}",
        (f"  submit error: {s['submit_error']}" if s.get("submit_error") else ""),
        (f"  fault: {s['fault']}" if s.get("fault") else ""),
    ]
    sys.stdout.write("\033[H" + "\r\n".join(line + "\033[K" for line in lines) + "\033[J")
    sys.stdout.flush()


def open_gripper(side: str) -> FollowerGripper:
    endpoint = {"left": find_left, "right": find_right}.get(side, find_follower)()
    print(f"[discovery] {endpoint.firmware_sn}  {endpoint.mcu_device}")
    return FollowerGripper(mcu_device=endpoint.mcu_device, ack_timeout_ms=2000, max_retries=0)


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("--side", default="follower", choices=("left", "right", "follower"))
    # Keep --mode for customer scripts, but make the currently supported path
    # explicit instead of exposing the old ControlLoop/private branch.
    ap.add_argument("--mode", default="force-position", choices=("force-position",),
                    help="MIT force-position control")
    ap.add_argument("--kp", type=float, default=15.0, help="末端位置保持刚度 Nm/rad")
    ap.add_argument("--kd", type=float, default=1.2, help="末端位置保持阻尼 Nm·s/rad")
    ap.add_argument("--hz", type=float, default=100.0, help="界面刷新率")
    ap.add_argument("--step-rad", "--step", dest="step_rad", type=float,
                    default=None,
                    help=("j/k 每个字符改变的目标角度（rad）；未指定时按速度自动计算，"
                          f"最小 {JOG_STEP_RAD:.3f}）"))
    ap.add_argument("--jog-timeout", type=float, default=JOG_INPUT_TIMEOUT_S,
                    help="终端停止发送重复字符后自动停止时间（秒，默认 0.65）")
    ap.add_argument("--grasp-torque", type=float, default=None,
                    help=("接触后的 MIT 纯前馈保持力矩 Nm；未指定且使用 "
                          "--set-envelope 时默认采用 --cont"))
    ap.add_argument("--hold-torque-offset", type=float,
                    default=HOLD_TORQUE_OFFSET_DEFAULT_NM,
                    help=("最终保持力矩补偿 Nm；用于修正反馈整体偏高，"
                          f"默认 {HOLD_TORQUE_OFFSET_DEFAULT_NM:.2f}"))
    ap.add_argument("--close-speed", type=float, default=0.5,
                    help="MIT 闭合速度 rad/s")
    ap.add_argument("--release-speed", type=float, default=0.5,
                    help="MIT 张开速度 rad/s")
    ap.add_argument("--contact-torque", type=float, default=0.080,
                    help="接触判定力矩下限 Nm")
    ap.add_argument("--force-limit", type=float, default=6.0,
                    help="力限保护阈值 Nm（默认 6.0）")
    ap.add_argument("--show-envelope", action="store_true", help="打印包络后退出")
    ap.add_argument("--set-envelope", action="store_true", help="写入包络后继续")
    ap.add_argument("--peak", type=float, default=2.0, help="运动瞬态力矩上限 Nm")
    ap.add_argument("--cont", type=float, default=1.6, help="可持续力矩上限 Nm")
    ap.add_argument("--temp-derate-start", type=int, default=0,
                    help="降额起点 °C，0=固件默认 90")
    ap.add_argument("--temp-wall", type=int, default=0,
                    help="温度墙 °C，0=固件默认 100")
    args = ap.parse_args()

    # --cont is the firmware continuous envelope limit, while
    # --grasp-torque is the application hold target.  When a user configures
    # the envelope and omits an explicit hold target, using the same continuous
    # value is the least surprising behaviour.  Keep the historical 0.35 Nm
    # default for ordinary runs without --set-envelope.
    if args.grasp_torque is None:
        args.grasp_torque = args.cont if args.set_envelope else 0.35

    if args.step_rad is None:
        # Keep one point-jog increment close to one hybrid control period. A
        # fixed 0.020 rad step takes only 25 ms at 0.8 rad/s, so the target is
        # reached and position-held before the next terminal repeat arrives.
        args.step_rad = max(
            JOG_STEP_RAD,
            max(float(args.close_speed), float(args.release_speed)) * HYBRID_INTERVAL_S,
        )
    if args.hz <= 0.0 or args.step_rad <= 0.0 or args.jog_timeout <= 0.0:
        ap.error("--hz must be > 0, --step-rad and --jog-timeout must be > 0")

    log.set_level("warn")
    g = open_gripper(args.side)
    print(f"[fw] {g.firmware_version}")

    if args.set_envelope:
        env = g.get_envelope()
        env.cont_torque_nm = args.cont
        env.peak_torque_nm = args.peak
        env.temp_derate_start_c = args.temp_derate_start
        env.temp_wall_c = args.temp_wall
        env.flags = GRIPPER_ENVELOPE_VALID | GRIPPER_ENVELOPE_ENFORCE
        g.set_envelope(env)
    env = g.get_envelope()
    print(f"[envelope] {env}")
    enforced = bool(env.flags & GRIPPER_ENVELOPE_ENFORCE)
    if not enforced:
        print("[warn] 固件运动包络未启用；建议先用 --set-envelope 写入并掉电保持")
    if args.show_envelope:
        return 0

    controller = MitForcePositionController(g, args)
    head = (
        f"=== Gripper Console [FORCE-POSITION] {args.side} "
        f"fw {g.firmware_version} kp={args.kp:.2f} kd={args.kd:.2f} ==="
    )
    envline = (
        f"  envelope: cont={env.cont_torque_nm:.3f} peak={env.peak_torque_nm:.3f} Nm  "
        f"temp={env.temp_derate_start_c or 90}/{env.temp_wall_c or 100}C  "
        + ("ENFORCED" if enforced else "*** INACTIVE ***")
    )

    last_key = "-"
    # A terminal has no key-up event. Repeated j/k bytes advance the target;
    # the watchdog stops accepting target changes after the repeat stream ends.
    last_jog_activity_at = 0.0
    sys.stdout.write("\033[2J")
    try:
        g.motor.clear_fault()
        controller.start()
        g.motor.enable()
        # Let the stream seed a real position before the first keyboard action.
        deadline = time.monotonic() + 3.0
        while controller.snapshot()["position"] is None and time.monotonic() < deadline:
            time.sleep(0.01)

        next_tick = time.monotonic()
        next_redraw = next_tick
        with RawKeyboard() as keyboard:
            while True:
                now = time.monotonic()
                if now >= next_tick:
                    if (controller.jog_active and last_jog_activity_at and
                            now - last_jog_activity_at >= args.jog_timeout):
                        controller.stop_jog()
                        last_jog_activity_at = 0.0
                    controller.tick()
                    # A small point-jog step can be traversed in less than the
                    # The normal 50 ms control period (for example,
                    # 0.020 rad at 0.8 rad/s takes about 25 ms). A 50 ms tick
                    # can then skip past the target and repeatedly enter
                    # position hold, which feels like a pause on every `j`.
                    # Run the active jog phase at 5 ms; retain the normal
                    # 50 ms cadence for full open/close actions.
                    tick_period = (
                        JOG_TICK_INTERVAL_S if controller.jog_active
                        else HYBRID_INTERVAL_S
                    )
                    next_tick += tick_period
                    if next_tick <= now:
                        next_tick = now + tick_period

                # select() wakes as soon as a key arrives instead of sleeping
                # for the UI period.  Drain the whole input queue so key-repeat
                # bursts are applied in order without one-byte backlog.
                keyboard.wait(min(
                    max(0.0, next_tick - time.monotonic()),
                    max(0.0, next_redraw - time.monotonic()),
                ))
                action_seen = False
                input_keys = keyboard.poll_many()
                for ch in input_keys:
                    last_key = repr(ch)
                    if ch == "j":
                        last_jog_activity_at = time.monotonic()
                        controller.start_jog(+1.0)
                        action_seen = True
                    elif ch == "k":
                        last_jog_activity_at = time.monotonic()
                        controller.start_jog(-1.0)
                        action_seen = True
                    elif ch == "o":
                        controller.stop_jog()
                        last_jog_activity_at = 0.0
                        controller.release()
                        action_seen = True
                    elif ch == "c":
                        controller.stop_jog()
                        last_jog_activity_at = 0.0
                        controller.set_target(0.0)
                        action_seen = True
                    elif ch == "h":
                        controller.stop_jog()
                        last_jog_activity_at = 0.0
                        controller.hold()
                        action_seen = True
                    elif ch == "e":
                        g.motor.enable()
                    elif ch == "d":
                        g.motor.disable()
                    elif ch == "f":
                        controller.stop_jog()
                        last_jog_activity_at = 0.0
                        g.motor.clear_fault()
                        controller.reset_after_fault()
                        action_seen = True
                    elif ch in ("q", "\x1b", "\x03"):
                        raise KeyboardInterrupt

                # Do not wait for the next 50 ms timer slot after a target
                # change.  One immediate command makes the key-to-motion path
                # feel immediate, while the regular tick remains the
                # safety/contact cadence.
                if action_seen:
                    controller.tick()
                    tick_period = (
                        JOG_TICK_INTERVAL_S if controller.jog_active
                        else HYBRID_INTERVAL_S
                    )
                    next_tick = time.monotonic() + tick_period

                now = time.monotonic()
                if now >= next_redraw:
                    _redraw(controller, head, envline, last_key)
                    next_redraw += 1.0 / max(args.hz, 1.0)
                    if next_redraw <= now:
                        next_redraw = now + 1.0 / max(args.hz, 1.0)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            controller.stop()
        except Exception as exc:
            print(f"\r\nstop: {exc}")
        try:
            g.motor.disable()
        except Exception as exc:
            print(f"\r\ndisable: {exc}")
        sys.stdout.write("\r\n[exit] MIT force-position controller stopped, motor disabled\r\n")
        sys.stdout.flush()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
