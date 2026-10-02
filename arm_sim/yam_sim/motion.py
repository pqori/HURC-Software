"""Backend-agnostic motion helpers used by the scripts: interpolation, pacing, safe moves.

Everything here uses only the i2rt ``Robot`` API, so it runs the same way on the sim and on the
real arm. Large position steps are dangerous on hardware (kp = 80 on the big joints), so the
scripts always interpolate to a new target instead of commanding it in one jump.
"""

from __future__ import annotations

import time
from typing import Any, Callable, Optional, Sequence

import numpy as np


def min_jerk(s: float) -> float:
    s = float(np.clip(s, 0.0, 1.0))
    return 10 * s**3 - 15 * s**4 + 6 * s**5


def min_jerk_vel(s: float) -> float:
    """d/ds of :func:`min_jerk`."""
    s = float(np.clip(s, 0.0, 1.0))
    return 30 * s**2 - 60 * s**3 + 30 * s**4


class Clock:
    """Fixed-rate loop pacing. A synchronous sim (no physics thread) is stepped instead of slept on."""

    def __init__(self, robot: Any, rate_hz: float):
        self.robot = robot
        self.dt = 1.0 / float(rate_hz)
        self._sync = hasattr(robot, "is_threaded") and not robot.is_threaded()
        self._next = time.perf_counter()

    def tick(self) -> None:
        if self._sync:
            self.robot.step_for(self.dt)
            return
        self._next += self.dt
        delay = self._next - time.perf_counter()
        if delay > 0:
            time.sleep(delay)
        else:
            self._next = time.perf_counter()

    def sleep(self, seconds: float) -> None:
        for _ in range(max(1, int(round(seconds / self.dt)))):
            self.tick()


def move_to(
    robot: Any,
    target: Sequence[float],
    duration: Optional[float] = None,
    max_speed: float = 0.8,
    rate_hz: float = 100.0,
    on_tick: Optional[Callable[[float, np.ndarray], None]] = None,
) -> np.ndarray:
    """Min-jerk joint-space move from the current position to ``target`` (num_dofs values).

    ``duration`` defaults to the largest arm-joint distance / ``max_speed`` (rad/s), at least 0.5 s.
    Returns the final measured joint position.
    """
    target = np.asarray(target, dtype=float)
    start = np.asarray(robot.get_joint_pos(), dtype=float).copy()
    assert start.shape == target.shape, f"target needs {start.shape[0]} values, got {target.shape[0]}"
    if duration is None:
        duration = max(0.5, float(np.max(np.abs(target[:6] - start[:6]))) / max_speed)
    clock = Clock(robot, rate_hz)
    n = max(1, int(round(duration * rate_hz)))
    for i in range(1, n + 1):
        s = min_jerk(i / n)
        q = start + s * (target - start)
        robot.command_joint_pos(q)
        if on_tick is not None:
            on_tick(i / rate_hz, q)
        clock.tick()
    return np.asarray(robot.get_joint_pos(), dtype=float)


class RateLimitedTarget:
    """A goal that the streamed command chases at a bounded joint speed (used by teleop/viewer)."""

    def __init__(self, q0: Sequence[float], max_speed: float = 1.0, gripper_index: Optional[int] = None, gripper_speed: float = 2.0):
        self.cmd = np.asarray(q0, dtype=float).copy()
        self.goal = self.cmd.copy()
        self.max_speed = max_speed
        self.gripper_index = gripper_index
        self.gripper_speed = gripper_speed

    def update(self, dt: float) -> np.ndarray:
        step = np.full_like(self.cmd, self.max_speed * dt)
        if self.gripper_index is not None:
            step[self.gripper_index] = self.gripper_speed * dt
        self.cmd = self.cmd + np.clip(self.goal - self.cmd, -step, step)
        return self.cmd.copy()
