"""``make_robot``: one call that returns either the simulated arm or the real one.

    robot = make_robot("sim")                   # YamSimRobot, works anywhere
    robot = make_robot("real", channel="can0")  # i2rt get_yam_robot(...), Linux host with i2rt

Both return an object with the same i2rt ``Robot`` API, so the code after this line is identical.
"""

from __future__ import annotations

import argparse
from typing import Any, Optional

REAL_INSTALL_HINT = (
    "The 'real' backend needs I2RT's `i2rt` package, which is not installed in this environment.\n"
    "It drives the arm over SocketCAN and only works on the Linux machine (e.g. the Jetson) wired\n"
    "to the arm's CAN adapter. On that machine:\n"
    "    git clone https://github.com/i2rt-robotics/i2rt.git && cd i2rt\n"
    "    pip install -e .            # or: uv pip install -e .\n"
    "    sudo ip link set can0 up type can bitrate 1000000\n"
    "then run the same script with --channel can0. On a Mac, use the simulator (--sim)."
)


class RealBackendUnavailable(ImportError):
    pass


# get_yam_robot keyword arguments that YamSimRobot also accepts, so they are passed through to both backends.
_SHARED_KW = {"zero_gravity_mode", "gravity_comp_factor", "use_coulomb_friction", "gripper_kp", "gripper_kd"}


def make_robot(
    backend: str = "sim",
    arm: str = "yam",
    gripper: str = "linear_4310",
    channel: str = "can0",
    **kw: Any,
):
    """Create a YAM robot.

    Args:
        backend: ``"sim"`` for :class:`yam_sim.robot.YamSimRobot`, ``"real"`` for
            ``i2rt.robots.get_robot.get_yam_robot``.
        arm, gripper: i2rt variant names (``"yam"``, ``"linear_4310"`` ...).
        channel: CAN interface (real backend only).
        **kw: passed to the backend. With ``"real"``, sim-only options (anything YamSimRobot
            accepts but get_yam_robot doesn't, e.g. ``start_thread``) are dropped.
    """
    backend = backend.lower()
    if backend == "sim":
        from yam_sim.robot import YamSimRobot

        return YamSimRobot(arm=arm, gripper=gripper, **kw)
    if backend != "real":
        raise ValueError(f"backend must be 'sim' or 'real', got {backend!r}")
    try:
        from i2rt.robots.get_robot import get_yam_robot
        from i2rt.robots.utils import ArmType, GripperType
    except ImportError as e:
        raise RealBackendUnavailable(REAL_INSTALL_HINT) from e

    import inspect

    accepted = set(inspect.signature(get_yam_robot).parameters)
    real_kw = {k: v for k, v in kw.items() if k in accepted}
    if isinstance(real_kw.get("gravity_comp_factor"), str):
        real_kw.pop("gravity_comp_factor")  # "hardware" is the real default anyway
    return get_yam_robot(
        channel=channel,
        arm_type=ArmType.from_string_name(arm),
        gripper_type=GripperType.from_string_name(gripper),
        **real_kw,
    )


def add_robot_args(parser: argparse.ArgumentParser) -> argparse.ArgumentParser:
    """Add the same robot-selection flags to every script: ``--sim`` (default) or ``--channel can0``."""
    g = parser.add_argument_group("robot")
    g.add_argument("--sim", action="store_true", help="use the MuJoCo simulation (default unless --channel is given)")
    g.add_argument("--channel", default=None, help="CAN interface of the real arm, e.g. can0 (implies the real backend)")
    g.add_argument("--arm", default="yam", help="arm variant (default: yam)")
    g.add_argument("--gripper", default="linear_4310", help="linear_4310 | crank_4310 | no_gripper")
    g.add_argument(
        "--zero-gravity",
        action="store_true",
        help="start in zero-gravity (gravity-comp only) mode instead of holding the current pose",
    )
    g.add_argument("--objects", nargs="?", const="cube", default=None,
                   help="(sim) add objects: --objects (a cube), --objects keyboard, --objects cube,keyboard")
    g.add_argument("--camera", default="c920", help="(sim) wrist webcam model: c920 | c270 | none (default: c920)")
    g.add_argument("--tool", default=None, help="(sim) tool held by the gripper: stylus (default: none)")
    return parser


def robot_from_args(args: argparse.Namespace, **sim_kw: Any):
    """Build the robot selected by :func:`add_robot_args` flags."""
    if args.channel and args.sim:
        raise SystemExit("Pass either --sim or --channel, not both.")
    if args.channel:
        return make_robot("real", args.arm, args.gripper, channel=args.channel, zero_gravity_mode=args.zero_gravity)
    sim_kw.setdefault("camera", getattr(args, "camera", "c920"))
    sim_kw.setdefault("tool", getattr(args, "tool", None))
    return make_robot("sim", args.arm, args.gripper, zero_gravity_mode=args.zero_gravity, objects=args.objects, **sim_kw)


def is_sim(robot: Any) -> bool:
    try:
        return bool(robot.get_robot_info().get("sim", False))
    except Exception:
        return False


def gripper_index(robot: Any) -> Optional[int]:
    try:
        return robot.get_robot_info().get("gripper_index")
    except Exception:
        return None
