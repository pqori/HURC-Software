"""Tkinter slider panel: joints 1-6 (degrees) + gripper (0..1), on the sim or the real arm.

    pixi run sliders                         # simulation
    python -m yam_sim.scripts.slider_panel --channel can0   # real arm

The sliders set a goal; the panel streams command_joint_pos at --rate Hz, rate-limited to
--max-speed rad/s, so dragging a slider fast never makes the arm jump. "Float" switches to
gravity-comp idle (kp = 0); moving any slider switches back to holding.
"""

from __future__ import annotations

import argparse
import sys

import numpy as np

from yam_sim.factory import add_robot_args, robot_from_args
from yam_sim.motion import RateLimitedTarget

READY = [0.0, 1.0, 1.0, -0.3, 0.0, 0.0]


def main(argv=None) -> int:
    try:
        import tkinter as tk
    except ImportError as e:  # pragma: no cover
        print(f"slider_panel needs tkinter, which this Python lacks ({e}). Use `pixi run teleop` instead.")
        return 2
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    add_robot_args(p)
    p.add_argument("--rate", type=float, default=100.0)
    p.add_argument("--max-speed", type=float, default=0.8)
    p.add_argument("--duration", type=float, default=None, help="close automatically after N s (testing)")
    args = p.parse_args(argv)

    try:
        root = tk.Tk()
    except tk.TclError as e:
        print(f"Cannot open a Tk window ({e}). Is a display available?")
        return 2
    robot = robot_from_args(args)
    info = robot.get_robot_info()
    lim = np.asarray(info["joint_limits"])
    gi = info.get("gripper_index")
    q0 = np.asarray(robot.get_joint_pos(), dtype=float)
    target = RateLimitedTarget(q0, max_speed=args.max_speed, gripper_index=gi)
    state = {"hold": True, "syncing": False}
    robot.command_joint_pos(q0)

    root.title("YAM sliders (" + ("SIM" if info.get("sim") else "REAL") + ")")
    vars_, labels = [], []
    for j in range(robot.num_dofs()):
        is_g = j == gi
        frm = tk.Frame(root)
        frm.pack(fill="x", padx=8, pady=2)
        tk.Label(frm, text="gripper" if is_g else f"joint{j + 1} (deg)", width=14, anchor="w").pack(side="left")
        v = tk.DoubleVar(value=q0[j] if is_g else np.rad2deg(q0[j]))
        lo, hi = (0.0, 1.0) if is_g else (np.rad2deg(lim[j, 0]), np.rad2deg(lim[j, 1]))
        res = 0.01 if is_g else 0.5

        def on_move(_val, j=j, is_g=is_g, v=v):
            if state["syncing"]:
                return
            if not state["hold"]:
                resume_hold()
            target.goal[j] = v.get() if is_g else np.deg2rad(v.get())

        tk.Scale(frm, variable=v, from_=lo, to=hi, resolution=res, orient="horizontal", length=360, command=on_move).pack(side="left")
        lab = tk.Label(frm, text="", width=10, anchor="e", font=("Courier", 11))
        lab.pack(side="left")
        vars_.append(v)
        labels.append(lab)

    def set_goal(q):
        if not state["hold"]:
            resume_hold()
        target.goal[: len(q)] = q
        state["syncing"] = True
        for j, v in enumerate(vars_):
            v.set(target.goal[j] if j == gi else np.rad2deg(target.goal[j]))
        state["syncing"] = False

    def resume_hold():
        q = np.asarray(robot.get_joint_pos(), dtype=float)
        target.cmd[:] = q
        target.goal[:] = q
        state["hold"] = True
        mode.config(text="HOLD")

    def float_mode():
        state["hold"] = False
        robot.enter_gravity_comp_idle()
        mode.config(text="FLOAT")

    btns = tk.Frame(root)
    btns.pack(pady=6)
    tk.Button(btns, text="Home", command=lambda: set_goal(np.zeros(6))).pack(side="left", padx=4)
    tk.Button(btns, text="Ready", command=lambda: set_goal(READY)).pack(side="left", padx=4)
    if gi is not None:
        tk.Button(btns, text="Open", command=lambda: set_goal(np.append(target.goal[:6], 1.0))).pack(side="left", padx=4)
        tk.Button(btns, text="Close", command=lambda: set_goal(np.append(target.goal[:6], 0.0))).pack(side="left", padx=4)
    tk.Button(btns, text="Float", command=float_mode).pack(side="left", padx=4)
    mode = tk.Label(btns, text="HOLD", width=6)
    mode.pack(side="left", padx=8)

    period_ms = max(1, int(1000 / args.rate))

    def tick():
        if state["hold"]:
            robot.command_joint_pos(target.update(period_ms / 1000.0))
        q = robot.get_joint_pos()
        for j, lab in enumerate(labels):
            lab.config(text=f"{q[j]:.2f}" if j == gi else f"{np.rad2deg(q[j]):+7.1f}")
        root.after(period_ms, tick)

    def on_close():
        try:
            robot.enter_gravity_comp_idle()
        finally:
            robot.close()
            root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    if args.duration:
        root.after(int(args.duration * 1000), on_close)
    root.after(period_ms, tick)
    root.mainloop()  # Tk must own the main thread; the sim's physics runs in its own thread
    return 0


if __name__ == "__main__":
    sys.exit(main())
