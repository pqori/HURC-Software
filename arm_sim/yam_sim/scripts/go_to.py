"""Move the arm to a joint vector or an end-effector pose, smoothly.

    pixi run go-to --joints 0 1.0 1.0 -0.3 0 0 --grip 1.0
    pixi run go-to --deg --joints 0 60 60 -20 0 0
    pixi run go-to --pos 0.35 0.0 0.25 --rpy 0 1.57 0      # grasp_site pose (base frame), IK
    pixi run go-to --pos 0.35 0.1 0.25                     # position only
    python -m yam_sim.scripts.go_to --channel can0 --joints ...   # real arm

The move is a min-jerk interpolation over --duration (default: from --max-speed). After it, the
arm holds the target for --hold seconds and the script prints the final error.
"""

from __future__ import annotations

import argparse
import sys
from typing import Optional, Sequence

import numpy as np

from yam_sim.factory import add_robot_args, is_sim, robot_from_args
from yam_sim.kinematics import Kinematics, make_kinematics, pose_error, pose_from_pos_rpy
from yam_sim.motion import Clock, move_to


def resolve_target(
    current: np.ndarray,
    kin: Kinematics,
    joints: Optional[Sequence[float]] = None,
    deg: bool = False,
    pos: Optional[Sequence[float]] = None,
    rpy: Optional[Sequence[float]] = None,
    gripper: Optional[float] = None,
    site: str = "grasp_site",
    gripper_index: Optional[int] = None,
) -> np.ndarray:
    """Turn CLI-style inputs into a full num_dofs joint target. Raises ValueError if IK fails."""
    target = np.asarray(current, dtype=float).copy()
    if joints is not None:
        j = np.asarray(joints, dtype=float)
        if len(j) not in (6, len(target)):
            raise ValueError(f"--joints needs 6 or {len(target)} values")
        if deg:
            j = np.deg2rad(j) if len(j) == 6 else np.append(np.deg2rad(j[:6]), j[6:])
        target[: len(j)] = j
    elif pos is not None:
        if rpy is not None:
            T = pose_from_pos_rpy(pos, np.deg2rad(rpy) if deg else rpy)
            goal = T
        else:
            goal = np.asarray(pos, dtype=float)
        ok, q = kin.ik(goal, site, init_q=target[:6])
        if not ok:
            raise ValueError(f"IK did not converge for pos={list(pos)} rpy={rpy}; target may be out of reach")
        target[:6] = q
    if gripper is not None:
        if gripper_index is None:
            raise ValueError("this robot has no gripper")
        target[gripper_index] = float(np.clip(gripper, 0.0, 1.0))
    return target


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    add_robot_args(p)
    g = p.add_mutually_exclusive_group(required=True)
    g.add_argument("--joints", type=float, nargs="+", help="6 arm joints (or 7 incl. gripper)")
    g.add_argument("--pos", type=float, nargs=3, metavar=("X", "Y", "Z"), help="end-effector position (m, base frame)")
    p.add_argument("--rpy", type=float, nargs=3, help="end-effector roll pitch yaw (rad, or deg with --deg)")
    p.add_argument("--grip", type=float, default=None, help="gripper 0 (closed) .. 1 (open)")
    p.add_argument("--deg", action="store_true", help="joint angles / rpy in degrees")
    p.add_argument("--site", default="grasp_site")
    p.add_argument("--duration", type=float, default=None, help="move time (s)")
    p.add_argument("--max-speed", type=float, default=0.6, help="used when --duration is not given (rad/s)")
    p.add_argument("--hold", type=float, default=1.0, help="hold the target this long before exiting (s)")
    args = p.parse_args(argv)

    robot = robot_from_args(args)
    try:
        kin = make_kinematics(robot if is_sim(robot) else args.arm, args.gripper)
        gi = robot.get_robot_info().get("gripper_index")
        target = resolve_target(robot.get_joint_pos(), kin, args.joints, args.deg, args.pos, args.rpy, args.grip, args.site, gi)
        print(f"[go_to] target q = {np.round(target, 4).tolist()}")
        move_to(robot, target, duration=args.duration, max_speed=args.max_speed)
        Clock(robot, 100).sleep(args.hold)
        q = np.asarray(robot.get_joint_pos())
        err = q - target
        print(f"[go_to] final q   = {np.round(q, 4).tolist()}")
        print(f"[go_to] joint err = {np.round(err, 4).tolist()}  (max arm |err| {np.max(np.abs(err[:6])):.4f} rad)")
        T = kin.fk(q[:6], args.site)
        print(f"[go_to] {args.site} at {np.round(T[:3, 3], 4).tolist()} m")
        if args.pos is not None:
            Tg = kin.fk(target[:6], args.site)
            dp, dr = pose_error(T, Tg)
            print(f"[go_to] EE error: {dp * 1000:.1f} mm, {np.rad2deg(dr):.2f} deg")
    except ValueError as e:
        print(f"[go_to] error: {e}")
        return 1
    finally:
        robot.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
