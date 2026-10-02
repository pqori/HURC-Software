"""Interactive viewer: ``pixi run sim`` (simulation) or ``... --channel can0`` (mirror and drive the real arm).

    pixi run sim                         # sim, linear_4310 gripper
    pixi run sim --gripper crank_4310 --objects
    pixi run sim --control               # start in CONTROL mode
    mjpython -m yam_sim.scripts.run_sim --channel can0   # real arm (Linux host: plain python)

See ``yam_sim/viewer.py`` for the key bindings (they are also shown in the window).
"""

from __future__ import annotations

import argparse
import sys

from yam_sim.factory import add_robot_args, robot_from_args


def main(argv=None) -> int:
    from yam_sim.viewer import ArmViewer, ensure_mjpython

    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    add_robot_args(p)
    p.add_argument("--control", action="store_true", help="start in CONTROL mode instead of VIS")
    p.add_argument("--site", default="grasp_site", help="end-effector site for IK (grasp_site or tcp_site)")
    p.add_argument("--jog-step", type=float, default=0.05, help="joint jog step (rad)")
    p.add_argument("--ee-step", type=float, default=0.01, help="end-effector nudge step (m)")
    p.add_argument("--max-speed", type=float, default=1.0, help="joint speed limit for viewer motions (rad/s)")
    p.add_argument("--duration", type=float, default=None, help="close the viewer after this many seconds")
    args = p.parse_args(argv)
    ensure_mjpython()
    # VIS mode is gravity-comp idle, so always start in zero-gravity mode (as i2rt's viewer does).
    args.zero_gravity = True
    robot = robot_from_args(args)
    try:
        ArmViewer(
            robot,
            arm=args.arm,
            gripper=args.gripper,
            ee_site=args.site,
            jog_step=args.jog_step,
            ee_step=args.ee_step,
            max_speed=args.max_speed,
            start_in_control=args.control,
        ).run(duration=args.duration)
    finally:
        robot.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
