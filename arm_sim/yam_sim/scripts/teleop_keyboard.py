"""Terminal keyboard teleop (no GUI). Works on the sim and on the real arm.

    pixi run teleop                                  # simulation (physics runs in the background)
    python -m yam_sim.scripts.teleop_keyboard --channel can0   # real arm, on the Linux host

Keys (no Enter needed):
  1..6 / 7     select arm joint / gripper
  + (=) / -    jog the selected joint (gripper: +/- 0.1)
  w / s        end effector +x / -x  (base frame; forward / back)
  a / d        end effector +y / -y  (left / right)
  r / f        end effector +z / -z  (up / down)
  o / c / g    gripper open / close / toggle
  h / y        glide to home (all zeros) / ready pose
  space        toggle HOLD (PD position control) <-> FLOAT (gravity comp only)
  [ / ]        halve / double the step size
  p            print the full observation dict
  q, Esc, ^C   quit (the arm is left in gravity-comp idle)

Every motion is rate-limited (--max-speed rad/s), so a key press never makes the arm jump.
"""

from __future__ import annotations

import argparse
import os
import select
import sys
import time
from typing import Optional

import numpy as np

from yam_sim.factory import add_robot_args, is_sim, robot_from_args
from yam_sim.kinematics import make_kinematics
from yam_sim.motion import RateLimitedTarget

HOME = np.zeros(6)
READY = np.array([0.0, 1.0, 1.0, -0.3, 0.0, 0.0])


class _RawTerminal:
    def __init__(self, enabled: bool):
        self.enabled = enabled
        self._old = None

    def __enter__(self):
        if self.enabled:
            import termios
            import tty

            self._old = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
        return self

    def __exit__(self, *exc):
        if self._old is not None:
            import termios

            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._old)

    def read_key(self) -> Optional[str]:
        if not select.select([sys.stdin], [], [], 0)[0]:
            return None
        ch = os.read(sys.stdin.fileno(), 1).decode(errors="ignore")
        if ch == "\x1b":  # Esc, or the start of an arrow-key sequence: swallow the rest
            while select.select([sys.stdin], [], [], 0.01)[0]:
                os.read(sys.stdin.fileno(), 8)
        return ch


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    add_robot_args(p)
    p.add_argument("--jog-step", type=float, default=0.05, help="joint step (rad)")
    p.add_argument("--ee-step", type=float, default=0.01, help="end-effector step (m)")
    p.add_argument("--max-speed", type=float, default=0.8, help="joint speed limit (rad/s)")
    p.add_argument("--rate", type=float, default=100.0, help="command rate (Hz)")
    p.add_argument("--site", default="grasp_site")
    p.add_argument("--script", default=None, help="non-interactive: a key sequence to play back, one key every --script-dt s")
    p.add_argument("--script-dt", type=float, default=0.3)
    args = p.parse_args(argv)

    interactive = args.script is None
    if interactive and not sys.stdin.isatty():
        print("teleop needs an interactive terminal (or pass --script KEYS).")
        return 2

    robot = robot_from_args(args)
    kin = make_kinematics(robot if is_sim(robot) else args.arm, args.gripper)
    n = robot.num_dofs()
    gi = robot.get_robot_info().get("gripper_index")
    limits = np.asarray(robot.get_robot_info()["joint_limits"])
    q0 = np.asarray(robot.get_joint_pos(), dtype=float)
    target = RateLimitedTarget(q0, max_speed=args.max_speed, gripper_index=gi)
    hold = True
    robot.command_joint_pos(q0)
    selected, jog, ee = 0, args.jog_step, args.ee_step
    dt = 1.0 / args.rate
    script = list(args.script or "")
    next_script_t = time.time() + args.script_dt
    last_print, msg = 0.0, ""
    print(__doc__.split("Keys")[1] if interactive else f"[teleop] playing script {args.script!r}")

    def ee_move(axis: int, sign: float) -> str:
        T = kin.fk(target.goal[:6], args.site)
        T[axis, 3] += sign * ee
        ok, q = kin.ik(T, args.site, init_q=target.goal[:6], restarts=0, max_iters=80)
        if ok:
            target.goal[:6] = q
            return ""
        return "IK failed (out of reach?)"

    try:
        with _RawTerminal(interactive) as term:
            while True:
                t0 = time.time()
                key = term.read_key() if interactive else None
                if not interactive and script and t0 >= next_script_t:
                    key, next_script_t = script.pop(0), t0 + args.script_dt
                if key is not None:
                    msg = ""
                    if key in ("q", "\x1b", "\x03"):
                        break
                    if key.isdigit() and 1 <= int(key) <= n:
                        selected = int(key) - 1
                    elif key in "+=-":
                        s = 1.0 if key in "+=" else -1.0
                        if selected == gi:
                            target.goal[gi] = np.clip(target.goal[gi] + 0.1 * s, 0, 1)
                        else:
                            target.goal[selected] = np.clip(target.goal[selected] + s * jog, *limits[selected])
                    elif key in "wsadrf":
                        axis, sign = {"w": (0, 1), "s": (0, -1), "a": (1, 1), "d": (1, -1), "r": (2, 1), "f": (2, -1)}[key]
                        msg = ee_move(axis, sign)
                    elif key in "ocg" and gi is not None:
                        target.goal[gi] = {"o": 1.0, "c": 0.0, "g": 0.0 if target.goal[gi] > 0.5 else 1.0}[key]
                    elif key in "hy":
                        target.goal[:6] = HOME if key == "h" else READY
                    elif key == " ":
                        hold = not hold
                        if hold:
                            q = np.asarray(robot.get_joint_pos(), dtype=float)
                            target.cmd[:], target.goal[:] = q, q
                        else:
                            robot.enter_gravity_comp_idle()
                    elif key in "[]":
                        f = 0.5 if key == "[" else 2.0
                        jog, ee = float(np.clip(jog * f, 0.005, 0.5)), float(np.clip(ee * f, 0.001, 0.1))
                    elif key == "p":
                        print("\n" + "\n".join(f"  {k}: {np.round(v, 4)}" for k, v in robot.get_observations().items()))
                if hold:
                    robot.command_joint_pos(target.update(dt))
                elif script == [] and not interactive:
                    pass
                if not interactive and not script and t0 >= next_script_t + 1.0:
                    break
                if t0 - last_print > 0.1:
                    q = robot.get_joint_pos()
                    T = kin.fk(q[:6], args.site)
                    names = [f"j{i + 1}" for i in range(6)] + (["grip"] if gi is not None else [])
                    cells = " ".join(f"{'*' if i == selected else ' '}{names[i]}={q[i]:+.2f}" for i in range(n))
                    line = f"[{'HOLD ' if hold else 'FLOAT'}] {cells} | EE {T[0, 3]:+.3f} {T[1, 3]:+.3f} {T[2, 3]:+.3f} | step {jog:.3f}rad {ee * 1000:.0f}mm {msg}"
                    if interactive:
                        sys.stdout.write("\r\x1b[K" + line)
                        sys.stdout.flush()
                    last_print = t0
                time.sleep(max(0.0, dt - (time.time() - t0)))
    except KeyboardInterrupt:
        pass
    finally:
        q = robot.get_joint_pos()
        print(f"\n[teleop] final q = {np.round(q, 3).tolist()}")
        robot.enter_gravity_comp_idle()
        robot.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
