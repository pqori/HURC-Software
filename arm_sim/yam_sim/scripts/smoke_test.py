"""Headless smoke test: run a trajectory and check joint tracking and torques.

    pixi run smoke                       # simulation, synchronous (faster than real time)
    pixi run smoke --realtime            # simulation with the real-time physics thread
    python -m yam_sim.scripts.smoke_test --channel can0 --i-know   # real arm (moves it!)

By default it streams ``command_joint_state`` with a velocity feedforward. Pass --no-vel-ff to send
positions only (``command_joint_pos``), which lags more because kd damps all motion when the
target velocity is zero (the real arm behaves the same way). The script prints per-joint RMS and max tracking error (commanded minus measured) and the peak
torque. It exits with status 1 if any arm joint's RMS error exceeds --max-rms, or if the sim
reports torque at the motor limit for more than 5 % of the run.
"""

from __future__ import annotations

import argparse
import sys

import numpy as np

from yam_sim.factory import add_robot_args, is_sim, robot_from_args
from yam_sim.motion import Clock, min_jerk, min_jerk_vel

WAYPOINTS = [
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 1.0, 1.0, -0.3, 0.0, 0.0, 1.0],
    [0.6, 1.4, 1.2, -0.6, 0.5, 0.8, 0.5],
    [-0.6, 1.8, 1.6, -0.9, -0.5, -0.8, 0.0],
    [0.0, 1.2, 0.9, 0.3, 0.0, 0.0, 1.0],
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
]


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    add_robot_args(p)
    p.add_argument("--segment-time", type=float, default=2.0, help="seconds per waypoint segment")
    p.add_argument("--rate", type=float, default=100.0, help="command rate (Hz)")
    p.add_argument("--max-rms", type=float, default=0.05, help="fail if any arm joint RMS error exceeds this (rad)")
    p.add_argument(
        "--no-vel-ff",
        action="store_true",
        help="send positions only (command_joint_pos). By default the script sends position + velocity "
        "(command_joint_state), which removes the kd*velocity lag",
    )
    p.add_argument("--realtime", action="store_true", help="(sim) use the real-time physics thread")
    p.add_argument("--i-know", action="store_true", help="required to run on real hardware (the arm moves)")
    args = p.parse_args(argv)
    if args.channel and not args.i_know:
        print("This moves the real arm through large motions. Clear the workspace and add --i-know.")
        return 2

    robot = robot_from_args(args, start_thread=args.realtime) if not args.channel else robot_from_args(args)
    try:
        n = robot.num_dofs()
        wps = np.array([w[:n] for w in WAYPOINTS])
        clock = Clock(robot, args.rate)
        start = np.asarray(robot.get_joint_pos())
        # Glide from wherever the arm is to the first waypoint, then run the trajectory.
        segs = [(start, wps[0])] + [(wps[i], wps[i + 1]) for i in range(len(wps) - 1)]
        steps = int(round(args.segment_time * args.rate))
        errs, taus = [], []
        for a, b in segs:
            for i in range(1, steps + 1):
                q = a + min_jerk(i / steps) * (b - a)
                if args.no_vel_ff:
                    robot.command_joint_pos(q)
                else:
                    v = min_jerk_vel(i / steps) * (b - a) / args.segment_time
                    robot.command_joint_state({"pos": q, "vel": v})
                clock.tick()
                obs = robot.get_observations()
                errs.append(q[:6] - obs["joint_pos"])
                taus.append(np.abs(obs["joint_eff"]))
        clock.sleep(1.0)
        final_err = np.abs(wps[-1][:6] - robot.get_joint_pos()[:6])
        errs = np.array(errs[steps:])  # skip the approach segment
        taus = np.array(taus[steps:])
        rms = np.sqrt(np.mean(errs**2, axis=0))
        mx = np.max(np.abs(errs), axis=0)
        tau_max = taus.max(axis=0)
        info = robot.get_robot_info()
        limits = np.asarray(info.get("torque_limits", [np.inf] * 7))[:6]
        sat_frac = np.mean(taus >= 0.999 * limits, axis=0)

        backend = "sim" if is_sim(robot) else "real"
        mode = "position only" if args.no_vel_ff else "position + velocity feedforward"
        print(f"YAM smoke test ({backend}, {info.get('gripper_type')}, {mode}) - {len(errs)} samples at {args.rate:g} Hz")
        print(f"{'joint':>6} {'RMS err':>10} {'max err':>10} {'final err':>10} {'max |tau|':>10} {'limit':>7} {'sat %':>6}")
        for j in range(6):
            print(
                f"{'j' + str(j + 1):>6} {rms[j]:>10.4f} {mx[j]:>10.4f} {final_err[j]:>10.4f} {tau_max[j]:>10.3f} {limits[j]:>7.1f} {100 * sat_frac[j]:>6.1f}"
            )
        ok = bool(np.all(rms < args.max_rms) and np.all(sat_frac < 0.05))
        if is_sim(robot):
            ok = ok and not info["watchdog"]["tripped"]
        print("RESULT:", "PASS" if ok else "FAIL", f"(threshold RMS < {args.max_rms} rad)")
        return 0 if ok else 1
    finally:
        robot.close()


if __name__ == "__main__":
    sys.exit(main())
