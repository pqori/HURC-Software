"""Damiao MIT-mode motor model, the 400 ms CAN watchdog, and the gripper force limiter.

Each actuated joint is a Damiao motor in MIT mode. Every CAN frame from the host carries
``(q_des, dq_des, kp, kd, tau_ff)``, and the motor's own fast loop applies

    tau = clip(kp * (q_des - q) + kd * (dq_des - dq) + tau_ff, -tau_max, tau_max)

until the next frame arrives. In the sim the PD part is a MuJoCo ``general`` actuator (see
``assembly.py``), so MuJoCo evaluates it on every physics substep, while the frame contents
change only at the host control rate. This module holds the per-frame bookkeeping:
:class:`MotorCommand`, the :class:`Watchdog`, MIT-mode quantisation, and a port of i2rt's
``GripperForceLimiter`` (i2rt @ 120c3c8, MIT).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable, Dict, Optional

import numpy as np


@dataclass
class MotorCommand:
    """One MIT-mode frame for every motor in the chain, in motor units (the gripper in rad)."""

    pos: np.ndarray
    vel: np.ndarray
    kp: np.ndarray
    kd: np.ndarray
    torque: np.ndarray

    @classmethod
    def zeros(cls, n: int) -> "MotorCommand":
        return cls(*(np.zeros(n) for _ in range(5)))

    def copy(self) -> "MotorCommand":
        return MotorCommand(self.pos.copy(), self.vel.copy(), self.kp.copy(), self.kd.copy(), self.torque.copy())


class Watchdog:
    """The Damiao CAN timeout: no frame for ``timeout`` seconds puts the motor in damping mode.

    ``feed="bus"`` (the default) matches i2rt on real hardware. i2rt's ``DMChainCanInterface``
    thread re-sends the latest command every loop iteration, so the motors are fed even while your
    code is idle, and the watchdog trips only if that loop stops (a crashed, killed or frozen
    process). In the sim the physics thread is the bus. Use ``YamSimRobot.stall_bus(seconds)`` to
    simulate the loop freezing.

    ``feed="command"`` is a stricter test mode. Only your own ``command_*`` calls feed the
    watchdog, which shows you what happens if you drive the motors without i2rt's resend thread.

    Once tripped the watchdog stays tripped (``latch=True``) until ``reinit()``/``reset()``, just
    as the real motors need to be re-enabled.
    """

    def __init__(self, timeout: float = 0.4, enabled: bool = True, feed: str = "bus", latch: bool = True):
        if feed not in ("bus", "command"):
            raise ValueError("watchdog feed must be 'bus' or 'command'")
        self.timeout = float(timeout)
        self.enabled = bool(enabled) and self.timeout > 0
        self.feed_source = feed
        self.latch = latch
        self.tripped = False
        self.trip_time: Optional[float] = None
        self._last_feed: Optional[float] = None

    def feed(self, now: float) -> None:
        self._last_feed = now
        if self.tripped and not self.latch:
            self.tripped = False

    def check(self, now: float) -> bool:
        """Return True if the motors are (now) in damping mode."""
        if not self.enabled or self._last_feed is None:
            return self.tripped
        if not self.tripped and now - self._last_feed > self.timeout:
            self.tripped = True
            self.trip_time = now
        return self.tripped

    def reset(self, now: Optional[float] = None) -> None:
        self.tripped = False
        self.trip_time = None
        self._last_feed = now


def quantize(x: np.ndarray, lo: np.ndarray, hi: np.ndarray, bits: int) -> np.ndarray:
    """Round-trip through the MIT-mode fixed-point encoding (``uint`` of ``bits`` over [lo, hi])."""
    span = hi - lo
    levels = (1 << bits) - 1
    xi = np.round((np.clip(x, lo, hi) - lo) / span * levels)
    return xi / levels * span + lo


# ------------------------------------------------------------------------------------------------
# Gripper force limiter: port of i2rt.robots.utils.GripperForceLimiter. It uses sim time and needs
# no real hardware. When the gripper stalls on an object it lowers the PD target so the motor
# pushes with a bounded torque (~max_force on the fingers) instead of the full PD torque.
# ------------------------------------------------------------------------------------------------


def linear_gripper_force_torque_map(motor_stroke: float, gripper_stroke: float, gripper_force: float, current_angle: float) -> float:
    return gripper_force * gripper_stroke / motor_stroke


def crank_gripper_force_torque_map(
    gripper_close_angle: float,
    gripper_open_angle: float,
    gripper_stroke: float,
    gripper_force: float,
    current_angle: float,
) -> float:
    """Torque for ``gripper_force`` on a zero-linkage crank gripper.

    ``current_angle`` is the sim's raw gripper coordinate: motor radians from fully closed. Upstream
    converts the real motor reading with ``-x + motor_reading_offset``. That offset depends on how
    the real motor was zeroed, so the sim measures the crank angle from ``gripper_close_angle``
    instead.
    """
    angle = gripper_close_angle + current_angle
    crank_radius = gripper_stroke / (2 * (np.cos(gripper_close_angle) - np.cos(gripper_open_angle)))
    return gripper_force * crank_radius * np.sin(angle)


@dataclass
class _TimedBuffer:
    window: float
    t: list = field(default_factory=list)
    v: list = field(default_factory=list)

    def put(self, t: float, v: float) -> None:
        self.t.append(t)
        self.v.append(v)
        while self.t and self.t[0] < t - self.window:
            self.t.pop(0)
            self.v.pop(0)

    def mean_abs(self) -> float:
        return float(np.abs(np.mean(self.v))) if self.v else 0.0


class GripperForceLimiter:
    def __init__(self, max_force: float, limiter_params: Dict, kp: float, average_torque_window: float = 0.1):
        self.max_force = max_force
        self._kp = kp
        self._is_clogged = False
        self._adjusted_qpos: Optional[float] = None
        self._buf = _TimedBuffer(average_torque_window)
        self.clog_force_threshold = float(limiter_params["clog_force_threshold"])
        self.clog_speed_threshold = float(limiter_params["clog_speed_threshold"])
        self.sign = float(limiter_params["sign"])
        kind = limiter_params["force_torque_map"]
        if kind == "linear":
            self._map: Callable[[float], float] = lambda angle: linear_gripper_force_torque_map(
                limiter_params["motor_stroke"], limiter_params["gripper_stroke"], max_force, angle
            )
        elif kind == "crank":
            self._map = lambda angle: crank_gripper_force_torque_map(
                limiter_params["gripper_close_angle"],
                limiter_params["gripper_open_angle"],
                limiter_params["gripper_stroke"],
                max_force,
                angle,
            )
        else:
            raise ValueError(f"Unknown force_torque_map {kind!r}")

    @property
    def is_clogged(self) -> bool:
        return self._is_clogged

    def _target_torque(self, s: Dict[str, float]) -> Optional[float]:
        avg = self._buf.mean_abs()
        if self._is_clogged:
            if s["current_normalized_qpos"] < s["target_normalized_qpos"] or avg < 0.2:
                self._is_clogged = False
        elif avg > self.clog_force_threshold and abs(s["current_qvel"]) < self.clog_speed_threshold:
            self._is_clogged = True
        if self._is_clogged:
            return self._map(s["current_qpos"]) + 0.3
        return None

    def update(self, now: float, s: Dict[str, float]) -> float:
        """Return the (possibly adjusted) raw gripper position target."""
        self._buf.put(now, s["current_eff"])
        target_eff = self._target_torque(s)
        if target_eff is None:
            self._adjusted_qpos = s["current_qpos"]
            return s["target_qpos"]
        command_sign = np.sign(s["target_qpos"] - s["current_qpos"]) * self.sign
        zero_eff_pos = s["last_command_qpos"] - command_sign * abs(s["current_eff"]) / self._kp
        target_raw = zero_eff_pos + command_sign * abs(target_eff) / self._kp
        a = 0.1
        if self._adjusted_qpos is None:
            self._adjusted_qpos = target_raw
        self._adjusted_qpos = (1 - a) * self._adjusted_qpos + a * target_raw
        return self._adjusted_qpos
