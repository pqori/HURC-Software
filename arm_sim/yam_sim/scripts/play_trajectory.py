"""Record joint trajectories to CSV/JSON and replay them, on the sim or the real arm.

    # record 10 s at 50 Hz (in zero-gravity mode you can move the real arm by hand)
    python -m yam_sim.scripts.play_trajectory record traj.csv --duration 10 --zero-gravity
    # replay (glides to the first waypoint, then plays back interpolated at --rate)
    pixi run play replay traj.csv --speed 0.5
    # write a demo trajectory to try
    pixi run play demo demo.csv

File formats:
  CSV:  header t,j1,...,j6[,gripper], one row per sample (t in seconds, angles in rad)
  JSON: {"joint_names": [...], "t": [...], "q": [[...], ...]}
Waypoints do not need to be evenly spaced. Replay interpolates linearly in time.
"""

from __future__ import annotations

import argparse
import csv
import json
import sys
import time
from typing import Tuple

import numpy as np

from yam_sim.factory import add_robot_args, robot_from_args
from yam_sim.motion import Clock, move_to


def save_trajectory(path: str, t: np.ndarray, q: np.ndarray) -> None:
    names = [f"j{i + 1}" for i in range(6)] + (["gripper"] if q.shape[1] > 6 else [])
    if path.endswith(".json"):
        with open(path, "w") as f:
            json.dump({"joint_names": names, "t": t.tolist(), "q": q.tolist()}, f)
    else:
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t", *names])
            for ti, qi in zip(t, q):
                w.writerow([f"{ti:.4f}", *(f"{v:.6f}" for v in qi)])


def load_trajectory(path: str) -> Tuple[np.ndarray, np.ndarray]:
    if path.endswith(".json"):
        with open(path) as f:
            d = json.load(f)
        t, q = np.asarray(d["t"], dtype=float), np.asarray(d["q"], dtype=float)
    else:
        rows = list(csv.reader(open(path)))
        data = np.array([[float(v) for v in r] for r in rows[1:] if r], dtype=float)
        t, q = data[:, 0], data[:, 1:]
    if len(t) < 2 or np.any(np.diff(t) <= 0):
        raise ValueError(f"{path}: need >= 2 samples with strictly increasing t")
    return t - t[0], q


def interpolate(t: np.ndarray, q: np.ndarray, tq: float) -> np.ndarray:
    return np.array([np.interp(tq, t, q[:, j]) for j in range(q.shape[1])])


def record(robot, duration: float, rate: float) -> Tuple[np.ndarray, np.ndarray]:
    clock = Clock(robot, rate)
    ts, qs = [], []
    t0 = time.perf_counter()
    for i in range(int(round(duration * rate))):
        qs.append(np.asarray(robot.get_joint_pos(), dtype=float).copy())
        ts.append(i / rate)
        clock.tick()
    return np.array(ts), np.array(qs)


def replay(robot, t: np.ndarray, q: np.ndarray, speed: float = 1.0, rate: float = 100.0, approach_speed: float = 0.5):
    """Glide to the first sample, then stream the interpolated trajectory. Returns (t, commanded, measured)."""
    n = robot.num_dofs()
    if q.shape[1] != n:
        if q.shape[1] == 6 and n == 7:  # arm-only file: keep the gripper where it is
            q = np.hstack([q, np.full((len(q), 1), robot.get_joint_pos()[6])])
        else:
            raise ValueError(f"trajectory has {q.shape[1]} columns, robot has {n} DOFs")
    move_to(robot, q[0], max_speed=approach_speed)
    clock = Clock(robot, rate)
    T = t[-1] / speed
    cmd_log, meas_log, t_log = [], [], []
    for i in range(int(np.ceil(T * rate)) + 1):
        tt = min(i / rate, T)
        qc = interpolate(t, q, tt * speed)
        robot.command_joint_pos(qc)
        clock.tick()
        t_log.append(tt)
        cmd_log.append(qc)
        meas_log.append(np.asarray(robot.get_joint_pos(), dtype=float).copy())
    return np.array(t_log), np.array(cmd_log), np.array(meas_log)


def demo_trajectory() -> Tuple[np.ndarray, np.ndarray]:
    t = np.array([0.0, 2.0, 3.5, 5.0, 6.5, 8.0, 10.0])
    q = np.array(
        [
            [0.0, 1.0, 1.0, -0.3, 0.0, 0.0, 1.0],
            [0.5, 1.4, 1.2, -0.5, 0.3, 0.5, 1.0],
            [0.5, 1.6, 1.0, -0.2, 0.3, 0.5, 0.0],
            [-0.5, 1.6, 1.0, -0.2, -0.3, -0.5, 0.0],
            [-0.5, 1.4, 1.2, -0.5, -0.3, -0.5, 1.0],
            [0.0, 1.2, 1.1, -0.3, 0.0, 0.0, 1.0],
            [0.0, 1.0, 1.0, -0.3, 0.0, 0.0, 1.0],
        ]
    )
    return t, q


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="cmd", required=True)
    pr = add_robot_args(sub.add_parser("record", help="sample the robot's joints to a file"))
    pr.add_argument("file")
    pr.add_argument("--duration", type=float, default=10.0)
    pr.add_argument("--rate", type=float, default=50.0)
    pp = add_robot_args(sub.add_parser("replay", help="play a recorded / authored trajectory"))
    pp.add_argument("file")
    pp.add_argument("--speed", type=float, default=1.0, help="time scale (0.5 = half speed)")
    pp.add_argument("--rate", type=float, default=100.0, help="command rate (Hz)")
    pd = sub.add_parser("demo", help="write a demo trajectory file")
    pd.add_argument("file")
    args = p.parse_args(argv)

    if args.cmd == "demo":
        save_trajectory(args.file, *demo_trajectory())
        print(f"[play] wrote {args.file}")
        return 0
    if args.cmd == "replay":
        t, q = load_trajectory(args.file)  # validate before touching the robot
    robot = robot_from_args(args)
    try:
        if args.cmd == "record":
            print(f"[play] recording {args.duration:g} s at {args.rate:g} Hz ...")
            t, q = record(robot, args.duration, args.rate)
            save_trajectory(args.file, t, q)
            print(f"[play] wrote {len(t)} samples to {args.file}")
        else:
            print(f"[play] replaying {args.file}: {len(t)} waypoints, {t[-1]:.2f} s at speed {args.speed:g}")
            _, cmd, meas = replay(robot, t, q, args.speed, args.rate)
            err = cmd[:, :6] - meas[:, :6]
            print(f"[play] done. RMS tracking error per joint (rad): {np.round(np.sqrt(np.mean(err**2, axis=0)), 4).tolist()}")
    finally:
        robot.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
