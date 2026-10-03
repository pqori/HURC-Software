"""Type a string on the keyboard with the arm: ``pixi run press -- "hello world"``.

For every character: look up its key, move the press site (the stylus tip by default) to a hover
pose a few cm above the keycap pointing straight down, descend slowly along the vertical until the
key registers a press (or a maximum depth is reached), hold briefly, lift back to the hover pose,
and go on to the next key. Poses come from :class:`yam_sim.kinematics.Kinematics` IK (position +
orientation); all motion is streamed through the i2rt ``Robot`` API with ``command_joint_pos``.

    pixi run press -- "hello world"                      # sim, stylus, prints per-key timing
    pixi run press -- "Hi!" --shift                      # press Shift first for capitals/symbols
    pixi run press -- "abc" --press-site grasp_site --tool none   # press with the closed fingertips
    pixi run press -- "abc" --realtime                   # threaded sim at wall-clock speed

Real arm (Linux host with i2rt, stylus taped/held in the closed gripper, keyboard measured):

    python -m yam_sim.scripts.press_keys "hello" --channel can0 \\
        --keyboard-pose 0.34 0.0 0.0 -90     # x y z [m] of the keyboard base centre, yaw [deg]

On hardware nothing reports which key went down, so a press is "the commanded tip reached
``--press-depth`` (6 mm, commanded) below the keycap top" and the printed string is what *should* have been
typed; read the real result in a text editor. The keyboard pose is the centre of the bottom of
the keyboard's footprint in the robot base frame, yaw measured from base +X to the keyboard's
row direction (left to right as typed); the sim default is (0.34, 0, 0, -90 deg), i.e. space bar
towards the robot. Start with a large ``--hover`` and slow ``--descent-speed`` on hardware.

Exit code: 0 if the keys reported by ``Keyboard.pressed_keys`` (in event order) spell the input,
1 otherwise.
"""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass, field
from typing import Any, List, Optional, Sequence, Tuple

import numpy as np

from yam_sim.keyboard import DEFAULT_POS, DEFAULT_YAW, Keyboard, char_to_key, keys_to_text
from yam_sim.kinematics import Kinematics, pose_error
from yam_sim.motion import Clock, move_to

# Press-site orientation with its +Z (approach) pointing straight down, for a key straight ahead
# (+X) of the base. The gripper's top face (where the webcam is) then faces away from the robot.
R_DOWN = np.array([[-1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, -1.0]])


def press_rotation(key_xy: Sequence[float]) -> np.ndarray:
    """Straight-down orientation, turned about the vertical to follow the base yaw towards the key."""
    yaw = float(np.arctan2(key_xy[1], key_xy[0]))
    c, s = np.cos(yaw), np.sin(yaw)
    Rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])
    return Rz @ R_DOWN


@dataclass
class PressResult:
    key: str
    ok: bool
    events: List[str] = field(default_factory=list)  # every key that went down during this press
    t_total: float = 0.0  # robot time (sim seconds) for move + press + lift
    t_to_press: float = 0.0  # from the start of the descent to the press
    depth: float = 0.0  # commanded depth below the keycap top at the press (m)
    tip_depth: float = float("nan")  # measured press-site depth below the unpressed keycap top (sim only)


class KeyPresser:
    """Presses keys with the arm. Works on the sim (press detection from the keyboard joints) and on
    the real arm (press = reached ``press_depth``), through the same i2rt Robot API."""

    def __init__(
        self,
        robot: Any,
        kin: Kinematics,
        keyboard: Keyboard,
        press_site: str = "stylus_tip",
        hover: float = 0.03,
        max_depth: float = 0.012,
        press_depth: float = 0.006,
        descent_speed: float = 0.03,
        hold: float = 0.15,
        rate_hz: float = 100.0,
        sense: bool = True,
        max_speed: float = 1.2,
        verbose: bool = True,
    ):
        self.robot, self.kin, self.kb = robot, kin, keyboard
        self.site = press_site
        self.hover, self.max_depth, self.press_depth = hover, max_depth, press_depth
        self.descent_speed, self.hold, self.rate = descent_speed, hold, rate_hz
        self.sense = sense  # True: read key presses from the sim keyboard
        self.max_speed = max_speed
        self.verbose = verbose
        self.events: List[str] = []
        self._down: set = set()
        self.clock = Clock(robot, rate_hz)
        self.n = robot.num_dofs()
        self.gi = robot.get_robot_info().get("gripper_index")

    # --------------------------------------------------------------- sensing
    def _poll(self) -> List[str]:
        """New key-down events since the last poll (sim only)."""
        if not self.sense:
            return []
        lock = getattr(self.robot, "lock", None)
        if lock is not None:
            with lock:
                now = set(self.kb.pressed_keys(self.robot.data))
        else:
            now = set(self.kb.pressed_keys())
        new = [k for k in self.kb.key_names if k in now and k not in self._down]
        self._down = now
        self.events.extend(new)
        return new

    def _tick(self) -> List[str]:
        self.clock.tick()
        return self._poll()

    def _time(self) -> float:
        return float(getattr(self.robot, "sim_time", time.perf_counter()))

    # --------------------------------------------------------------- planning
    def _full_q(self, q_arm: np.ndarray) -> np.ndarray:
        q = np.asarray(self.robot.get_commanded_pos() if hasattr(self.robot, "get_commanded_pos") else self.robot.get_joint_pos(), dtype=float).copy()
        q[:6] = q_arm
        if self.gi is not None:
            q[self.gi] = 0.0  # keep the gripper closed (holding the stylus)
        return q

    def _ik(self, T: np.ndarray, seed: np.ndarray, restarts: int = 0) -> np.ndarray:
        ok, q = self.kin.ik(T, self.site, init_q=seed, restarts=restarts, rot_weight=0.5, max_iters=200)
        if not ok:
            dp, dr = pose_error(self.kin.fk(q, self.site), T)
            if dp > 1e-3 or dr > 0.02:
                raise RuntimeError(f"IK failed for {self.site} at {T[:3, 3].round(3)} (error {dp * 1000:.1f} mm, {np.rad2deg(dr):.1f} deg)")
        return q

    def plan(self, key: str, seed: np.ndarray) -> Tuple[np.ndarray, List[np.ndarray], np.ndarray]:
        """Returns (hover q, descent waypoints q every 1 mm down to max_depth, their depths)."""
        top = self._key_top(key)
        self._top_z = float(top[2])
        R = press_rotation(top[:2])
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = top + [0, 0, self.hover]
        q_hover = self._ik(T, seed, restarts=4)
        depths = np.arange(-self.hover, self.max_depth + 1e-9, 0.001)
        qs, q = [], q_hover
        for dz in depths:
            T[:3, 3] = top - [0, 0, dz]
            q = self._ik(T, q)
            qs.append(q)
        return q_hover, qs, depths

    def _key_top(self, key: str) -> np.ndarray:
        lock = getattr(self.robot, "lock", None) if self.sense else None
        if lock is None:
            return self.kb.key_pose(key)[:3, 3]
        with lock:
            return self.kb.key_pose(key, self.robot.data)[:3, 3]

    # --------------------------------------------------------------- execution
    def _stream(self, q_from: np.ndarray, q_to: np.ndarray, n: int) -> None:
        for i in range(1, n + 1):
            a = i / n
            self.robot.command_joint_pos(self._full_q((1 - a) * q_from + a * q_to))
            self._tick()

    def press(self, key: str) -> PressResult:
        key = self.kb.name(key)
        t0 = self._time()
        n_ev = len(self.events)
        seed = np.asarray(self.robot.get_joint_pos(), dtype=float)[:6]
        q_hover, qs, depths = self.plan(key, seed)
        move_to(self.robot, self._full_q(q_hover), max_speed=self.max_speed, rate_hz=self.rate,
                on_tick=lambda *_: self._poll())
        self.clock = Clock(self.robot, self.rate)  # re-sync pacing after move_to's own clock
        settle = int(0.15 * self.rate)
        for _ in range(settle):
            self._tick()
        # Descend 1 mm per waypoint at descent_speed.
        ticks_per_mm = max(1, int(round(0.001 / self.descent_speed * self.rate)))
        t_desc = self._time()
        pressed_at, prev = None, q_hover
        for i, q in enumerate(qs):
            for j in range(1, ticks_per_mm + 1):
                a = j / ticks_per_mm
                self.robot.command_joint_pos(self._full_q((1 - a) * prev + a * q))
                new = self._tick()
                if self.sense and key in new:
                    pressed_at = i
                    break
            prev = q
            if pressed_at is not None:
                break
            if not self.sense and depths[i] >= self.press_depth - 1e-9:
                pressed_at = i
                self.events.append(key)
                break
        t_press = self._time() - t_desc
        ok = pressed_at is not None
        tip_depth = float("nan")
        if hasattr(self.robot, "get_ee_pose"):
            tip_depth = float(self._top_z - self.robot.get_ee_pose(self.site)[2, 3])
        idx = pressed_at if ok else len(qs) - 1
        for _ in range(max(1, int(self.hold * self.rate))):
            self._tick()
        # Lift straight back up along the same waypoints, twice as fast.
        cur = prev
        for q in reversed(qs[:idx + 1]):
            self._stream(cur, q, max(1, ticks_per_mm // 2))
            cur = q
        self._stream(cur, q_hover, max(1, ticks_per_mm // 2))
        for _ in range(int(0.1 * self.rate)):
            self._tick()
        res = PressResult(key, ok, self.events[n_ev:], self._time() - t0, t_press, float(depths[idx]), tip_depth)
        if self.verbose:
            status = "ok" if ok and res.events == [key] else ("MISSED" if not ok else f"extra keys {res.events}")
            print(f"  {key:>10s}: {status:<14s} move+press+lift {res.t_total:5.2f} s, descent->press {res.t_to_press:4.2f} s, "
                  f"depth at press: commanded {res.depth * 1000:4.1f} mm, actual {res.tip_depth * 1000:4.1f} mm", flush=True)
        return res


def plan_keys(text: str, shift: bool) -> Tuple[List[str], str]:
    """Key sequence for ``text`` and the text that sequence produces."""
    keys = []
    for ch in text:
        k, needs_shift = char_to_key(ch)
        if needs_shift and shift:
            keys.append("lshift")
        keys.append(k)
    return keys, keys_to_text(keys)


def type_text(presser: KeyPresser, text: str, shift: bool = False) -> Tuple[str, str, List[PressResult]]:
    """Press every key for ``text``. Returns (expected text, typed text from events, per-key results)."""
    keys, expected = plan_keys(text, shift)
    results = [presser.press(k) for k in keys]
    return expected, keys_to_text(presser.events), results


def build_parser() -> argparse.ArgumentParser:
    from yam_sim.factory import add_robot_args

    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("text", nargs="?", default="hello world", help="string to type")
    add_robot_args(p)
    p.set_defaults(objects="keyboard", tool="stylus")
    p.add_argument("--press-site", default=None, help="stylus_tip (default with --tool stylus) or grasp_site (fingertips)")
    p.add_argument("--keyboard-pose", type=float, nargs=4, metavar=("X", "Y", "Z", "YAW_DEG"), default=None,
                   help="keyboard base centre [m] and yaw [deg] in the robot base frame (required with --channel)")
    p.add_argument("--layout", default="full", help="full | tkl")
    p.add_argument("--hover", type=float, default=0.03, help="hover height above the keycap (m)")
    p.add_argument("--max-depth", type=float, default=0.012, help="give up this far below the keycap top (m)")
    p.add_argument("--press-depth", type=float, default=0.006,
                   help="(real arm) commanded depth below the keycap top that counts as a press (m). The arm is compliant "
                        "(PD control), so the tip lags the command by a few mm; tune on hardware")
    p.add_argument("--descent-speed", type=float, default=0.03, help="vertical speed while pressing (m/s)")
    p.add_argument("--hold", type=float, default=0.15, help="hold time at the bottom (s)")
    p.add_argument("--shift", action="store_true", help="press Shift before capitals/symbols (sticky-keys style)")
    p.add_argument("--realtime", action="store_true", help="(sim) run the physics thread at wall-clock speed instead of stepping")
    p.add_argument("--quiet", action="store_true")
    return p


def main(argv: Optional[Sequence[str]] = None) -> int:
    from yam_sim.assembly import load_scene
    from yam_sim.factory import make_robot

    args = build_parser().parse_args(argv)
    real = bool(args.channel)
    if real and args.keyboard_pose is None:
        raise SystemExit("--channel needs --keyboard-pose X Y Z YAW_DEG (measure the real keyboard in the robot base frame)")
    if args.keyboard_pose is not None:
        x, y, z, yaw_deg = args.keyboard_pose
        kb_opts = {"layout": args.layout, "pos": (x, y, z), "yaw": np.deg2rad(yaw_deg)}
    else:
        kb_opts = {"layout": args.layout, "pos": DEFAULT_POS, "yaw": DEFAULT_YAW}
    tool = None if str(args.tool).lower() in ("none", "") else args.tool
    site = args.press_site or ("stylus_tip" if tool == "stylus" else "grasp_site")

    if real:
        robot = make_robot("real", args.arm, args.gripper, channel=args.channel, zero_gravity_mode=False)
        model, _ = load_scene(args.arm, args.gripper, tool=tool, camera=args.camera)
        kin = Kinematics(model=model)
        kb = Keyboard(**kb_opts)  # geometry only: no sensing on hardware
        sense = False
    else:
        robot = make_robot("sim", args.arm, args.gripper, zero_gravity_mode=False, objects="keyboard", tool=tool,
                           camera=args.camera, keyboard=kb_opts, start_thread=args.realtime)
        kin = Kinematics(model=robot.model)
        kb = Keyboard.from_model(robot.model, robot.data)
        sense = True
    try:
        if not real:
            # Start from a pose above the keyboard instead of the stretched-out home pose.
            move_to(robot, np.array([0.0, 1.0, 1.0, -0.6, 0.0, 0.0, 0.0][: robot.num_dofs()]), duration=1.5)
        presser = KeyPresser(robot, kin, kb, press_site=site, hover=args.hover, max_depth=args.max_depth,
                             press_depth=args.press_depth, descent_speed=args.descent_speed, hold=args.hold,
                             sense=sense, verbose=not args.quiet)
        print(f"[press] typing {args.text!r} with {site} on {kb} ({'REAL: no key sensing' if real else 'sim'})", flush=True)
        t0 = time.perf_counter()
        expected, typed, results = type_text(presser, args.text, shift=args.shift)
        wall = time.perf_counter() - t0
        sim_t = sum(r.t_total for r in results)
        print(f"[press] events: {presser.events}")
        print(f"[press] typed:    {typed!r}")
        print(f"[press] expected: {expected!r}")
        print(f"[press] {len(results)} keys in {sim_t:.1f} s robot time ({sim_t / max(1, len(results)):.2f} s/key), {wall:.1f} s wall")
        if not args.shift and expected != args.text:
            print("[press] note: capitals/symbols were typed without Shift (pass --shift)")
        ok = typed == expected
        print("[press] OK" if ok else "[press] MISMATCH")
        return 0 if ok else 1
    finally:
        robot.close()


if __name__ == "__main__":
    sys.exit(main())
