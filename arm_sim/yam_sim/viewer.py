"""Interactive MuJoCo viewer for the YAM arm, for the sim or the real arm.

Launch with ``pixi run sim``. On macOS the passive viewer only works under ``mjpython``, so the
script restarts itself under ``mjpython`` if you start it with plain ``python``. Like i2rt's
``control_with_mujoco`` example, the window shows a mirror of the robot: the sim's own state, or
the real arm's measured joints, on a separate MjData. Commands go through the i2rt ``Robot`` API
only.

Modes (SPACE toggles, as in i2rt):
  VIS      gravity-comp idle (kp = 0). The marker follows the end effector. In the sim you can
           push the floating arm: double-click a link, then ctrl + right-drag.
  CONTROL  the arm PD-tracks a target. Move it by dragging the marker (double-click it, then
           ctrl + right-drag to translate or ctrl + left-drag to rotate), by jogging joints, or by
           nudging the end effector from the keyboard.

Keys (most letters are taken by MuJoCo's own visualisation toggles, so these avoid them):
  SPACE           VIS <-> CONTROL
  1 .. 6          select arm joint;  7 = gripper
  = / -           jog the selected joint + / - (keypad + / - also work)
  Up / Down       end effector +x / -x (base frame)     [CONTROL]
  Left / Right    end effector +y / -y                  [CONTROL]
  PgUp / PgDn     end effector +z / -z                  [CONTROL]
  End             toggle gripper open / closed           [CONTROL]
  Home            glide to the home pose (all zeros)    [CONTROL]
  Insert          glide to the "ready" pose             [CONTROL]
  ;  /  '         halve / double the jog and nudge step
  Backspace       (sim) reset the simulation to home
On a Mac keyboard: fn+Up/Down = PgUp/PgDn, fn+Left/Right = Home/End, and Insert does not exist,
so use the Python API or teleop for the ready pose.
"""

from __future__ import annotations

import copy
import os
import queue
import sys
import threading
import time
from typing import Any, Optional

import mujoco
import numpy as np

from yam_sim.assembly import load_scene
from yam_sim.kinematics import Kinematics
from yam_sim.motion import RateLimitedTarget

KEY = dict(
    SPACE=32, EQUAL=61, MINUS=45, KP_ADD=334, KP_SUB=333, RIGHT=262, LEFT=263, DOWN=264, UP=265,
    PAGE_UP=266, PAGE_DOWN=267, HOME=268, END=269, INSERT=260, SEMICOLON=59, APOSTROPHE=39, BACKSPACE=259,
)
HOME_Q = np.zeros(6)
READY_Q = np.array([0.0, 1.0, 1.0, -0.3, 0.0, 0.0])

HELP_LEFT = "SPACE\n1-6 / 7\n= / -\nArrows\nPgUp/PgDn\nEnd\nHome\n; / '\nBackspace"
HELP_RIGHT = "VIS <-> CONTROL\nselect joint / gripper\njog selected\nEE x / y\nEE z\ngripper toggle\nhome pose\nstep x0.5 / x2\nreset (sim)"


def ensure_mjpython() -> None:
    """On macOS, re-exec under mjpython (required by mujoco.viewer.launch_passive)."""
    if sys.platform != "darwin":
        return
    import mujoco.viewer as mv

    if mv._MJPYTHON is not None:
        return
    exe = os.path.join(os.path.dirname(sys.executable), "mjpython")
    if not os.path.exists(exe):
        raise SystemExit("macOS needs `mjpython` for the MuJoCo viewer, but it is not next to this python. Run: pixi run sim")
    print("[viewer] macOS: relaunching under mjpython ...", flush=True)
    os.execv(exe, [exe, "-m", "yam_sim.scripts.run_sim", *sys.argv[1:]])


class ArmViewer:
    VIS_RGBA = np.array([0.2, 0.8, 0.2, 0.35])
    CTRL_RGBA = np.array([0.9, 0.2, 0.2, 0.7])

    def __init__(
        self,
        robot: Any,
        arm: str = "yam",
        gripper: str = "linear_4310",
        ee_site: str = "grasp_site",
        jog_step: float = 0.05,
        ee_step: float = 0.01,
        max_speed: float = 1.0,
        control_rate: float = 100.0,
        render_rate: float = 60.0,
        start_in_control: bool = False,
    ):
        self.robot = robot
        info = robot.get_robot_info()
        self.is_sim = bool(info.get("sim", False))
        self.gi: Optional[int] = info.get("gripper_index")
        self.n = robot.num_dofs()
        if self.is_sim and hasattr(robot, "model"):
            self.model = copy.copy(robot.model)  # own copy: the physics thread edits actuator params live
        else:
            self.model, _ = load_scene(arm, gripper)
        self.data = mujoco.MjData(self.model)
        self.kin = Kinematics(model=copy.copy(self.model))
        self.ee_site = ee_site
        m = self.model
        self.mocap_id = m.body("target").mocapid[0]
        self.marker_geom = m.geom("target_geom").id
        self.arm_qadr = np.array([m.jnt_qposadr[m.joint(f"joint{i}").id] for i in range(1, 7)])
        self.finger_qadr = [m.jnt_qposadr[m.joint(j).id] for j in ("joint7", "joint8") if self._has_joint(j)]
        self.finger_range = m.jnt_range[m.joint("joint7").id] if self.finger_qadr else None

        self.jog_step = jog_step
        self.ee_step = ee_step
        self.control_dt = 1.0 / control_rate
        self.render_dt = 1.0 / render_rate
        self.max_speed = max_speed
        self.mode = "VIS"
        self.selected = 0
        self.keys: "queue.Queue[int]" = queue.Queue()
        self.stop = threading.Event()
        self.viewer = None
        self.target: Optional[RateLimitedTarget] = None
        self._last_mocap = (np.zeros(3), np.array([1.0, 0, 0, 0]))
        self._status = ""
        self._ik_ok = True
        self._start_in_control = start_in_control

    def _has_joint(self, name: str) -> bool:
        return mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name) >= 0

    # ---------------------------------------------------------------- mirroring / pose helpers
    def _mirror(self) -> None:
        if self.is_sim and hasattr(self.robot, "copy_state_to"):
            self.robot.copy_state_to(self.data)
        else:
            q = self.robot.get_joint_pos()
            self.data.qpos[self.arm_qadr] = q[:6]
            if self.gi is not None and self.finger_qadr:
                lo, hi = self.finger_range
                for a in self.finger_qadr:
                    self.data.qpos[a] = lo + float(np.clip(q[self.gi], 0, 1)) * (hi - lo)
        mujoco.mj_forward(self.model, self.data)

    def _ee_pose(self, q_arm: np.ndarray) -> np.ndarray:
        return self.kin.fk(q_arm, self.ee_site)

    def _set_mocap(self, T: np.ndarray) -> None:
        quat = np.empty(4)
        mujoco.mju_mat2Quat(quat, T[:3, :3].reshape(-1))
        self.data.mocap_pos[self.mocap_id] = T[:3, 3]
        self.data.mocap_quat[self.mocap_id] = quat
        self._last_mocap = (T[:3, 3].copy(), quat.copy())

    def _mocap_pose(self) -> np.ndarray:
        T = np.eye(4)
        R = np.empty(9)
        mujoco.mju_quat2Mat(R, self.data.mocap_quat[self.mocap_id])
        T[:3, :3] = R.reshape(3, 3)
        T[:3, 3] = self.data.mocap_pos[self.mocap_id]
        return T

    # ---------------------------------------------------------------- modes
    def _enter_control(self) -> None:
        if hasattr(self.robot, "set_external_forces"):
            self.robot.set_external_forces(None)
        q = np.asarray(self.robot.get_joint_pos(), dtype=float)
        self.target = RateLimitedTarget(q, max_speed=self.max_speed, gripper_index=self.gi)
        self.robot.command_joint_pos(q)
        self._set_mocap(self._ee_pose(q[:6]))
        self.model.geom_rgba[self.marker_geom] = self.CTRL_RGBA
        self.mode = "CONTROL"
        print("[viewer] CONTROL mode: drag the red marker (double-click, ctrl+right-drag) or use the keys", flush=True)

    def _enter_vis(self) -> None:
        if hasattr(self.robot, "enter_gravity_comp_idle"):
            self.robot.enter_gravity_comp_idle()
        self.model.geom_rgba[self.marker_geom] = self.VIS_RGBA
        self.mode = "VIS"
        self.target = None
        print("[viewer] VIS mode: gravity comp, mirroring the robot", flush=True)

    # ---------------------------------------------------------------- input handling (control thread)
    def _handle_key(self, key: int) -> None:
        K = KEY
        if key == K["SPACE"]:
            self._enter_control() if self.mode == "VIS" else self._enter_vis()
            return
        if ord("1") <= key <= ord("7"):
            j = key - ord("1")
            if j < self.n:
                self.selected = j
            return
        if key in (K["SEMICOLON"], K["APOSTROPHE"]):
            f = 0.5 if key == K["SEMICOLON"] else 2.0
            self.jog_step = float(np.clip(self.jog_step * f, 0.005, 0.5))
            self.ee_step = float(np.clip(self.ee_step * f, 0.001, 0.1))
            return
        if key == K["BACKSPACE"] and self.is_sim and hasattr(self.robot, "reset"):
            self.robot.reset()
            if self.mode == "CONTROL":
                self._enter_control()
            return
        if self.mode != "CONTROL" or self.target is None:
            if key in (K["EQUAL"], K["MINUS"], K["KP_ADD"], K["KP_SUB"], K["UP"], K["DOWN"], K["LEFT"], K["RIGHT"],
                       K["PAGE_UP"], K["PAGE_DOWN"], K["HOME"], K["END"], K["INSERT"]):
                self._status = "press SPACE for CONTROL mode first"
            return
        goal = self.target.goal
        if key in (K["EQUAL"], K["MINUS"], K["KP_ADD"], K["KP_SUB"]):
            sign = 1.0 if key in (K["EQUAL"], K["KP_ADD"]) else -1.0
            if self.selected == self.gi:
                goal[self.gi] = float(np.clip(goal[self.gi] + sign * 0.1, 0.0, 1.0))
            else:
                goal[self.selected] += sign * self.jog_step
                lim = self.robot.get_robot_info()["joint_limits"]
                goal[: 6] = np.clip(goal[:6], lim[:, 0], lim[:, 1])
            self._set_mocap(self._ee_pose(goal[:6]))
        elif key in (K["UP"], K["DOWN"], K["LEFT"], K["RIGHT"], K["PAGE_UP"], K["PAGE_DOWN"]):
            d = {K["UP"]: (0, 1), K["DOWN"]: (0, -1), K["LEFT"]: (1, 1), K["RIGHT"]: (1, -1), K["PAGE_UP"]: (2, 1), K["PAGE_DOWN"]: (2, -1)}[key]
            T = self._ee_pose(goal[:6])
            T[d[0], 3] += d[1] * self.ee_step
            self._solve_ik_to(T)
            self._set_mocap(T)
        elif key == K["END"] and self.gi is not None:
            goal[self.gi] = 0.0 if goal[self.gi] > 0.5 else 1.0
        elif key in (K["HOME"], K["INSERT"]):
            goal[:6] = HOME_Q if key == K["HOME"] else READY_Q
            self._set_mocap(self._ee_pose(goal[:6]))

    def _solve_ik_to(self, T: np.ndarray) -> None:
        ok, q = self.kin.ik(T, self.ee_site, init_q=self.target.goal[:6], restarts=0, max_iters=60)
        self._ik_ok = ok
        if ok:
            self.target.goal[:6] = q
        else:
            self._status = "IK did not converge (target out of reach?)"

    def _control_loop(self) -> None:
        while not self.stop.is_set():
            t0 = time.perf_counter()
            try:
                while True:
                    key = self.keys.get_nowait()
                    with self.viewer.lock():
                        self._handle_key(key)
            except queue.Empty:
                pass
            if self.mode == "CONTROL" and self.target is not None:
                with self.viewer.lock():
                    pos = self.data.mocap_pos[self.mocap_id].copy()
                    quat = self.data.mocap_quat[self.mocap_id].copy()
                    moved = not (np.allclose(pos, self._last_mocap[0], atol=1e-6) and np.allclose(quat, self._last_mocap[1], atol=1e-6))
                    T = self._mocap_pose()
                if moved:
                    self._last_mocap = (pos, quat)
                    self._solve_ik_to(T)
                self.robot.command_joint_pos(self.target.update(self.control_dt))
            elif self.is_sim and hasattr(self.robot, "set_external_forces"):
                with self.viewer.lock():
                    # Only pass on forces while the user is actually dragging, in case the viewer
                    # leaves a stale force in xfrc_applied after the drag ends.
                    active = bool(self.viewer.perturb.active)
                    xfrc = self.data.xfrc_applied.copy() if active else None
                self.robot.set_external_forces(xfrc)
            time.sleep(max(0.0, self.control_dt - (time.perf_counter() - t0)))

    # ---------------------------------------------------------------- overlay
    def _overlay(self) -> None:
        obs = self.robot.get_observations()
        q = np.asarray(obs["joint_pos"])
        tau = np.asarray(obs["joint_eff"])
        if self.target is not None:
            qd = self.target.cmd
        elif hasattr(self.robot, "get_commanded_pos"):
            qd = self.robot.get_commanded_pos()
        else:
            qd = np.full(self.n, np.nan)
        rows = []
        for j in range(6):
            mark = ">" if self.selected == j else " "
            rows.append(f"{mark}j{j + 1}  {q[j]:+7.3f}  {qd[j]:+7.3f}  {tau[j]:+7.2f}")
        if self.gi is not None:
            mark = ">" if self.selected == self.gi else " "
            rows.append(f"{mark}grip {obs['gripper_pos'][0]:6.3f}  {qd[self.gi]:6.3f}  {obs['gripper_eff'][0]:+7.2f}")
        info = self.robot.get_robot_info()
        wd = info.get("watchdog", {})
        hdr = f"{self.mode}  [{'SIM' if self.is_sim else 'REAL'}]  step {self.jog_step:.3f} rad / {self.ee_step * 1000:.0f} mm"
        if wd.get("tripped"):
            hdr += "  WATCHDOG: DAMPING"
        T = self._ee_pose(q[:6])
        left = hdr + "\n      q       q_des    tau(Nm)\n" + "\n".join(rows)
        left += f"\nEE  {T[0, 3]:+.3f} {T[1, 3]:+.3f} {T[2, 3]:+.3f} m"
        if self._status:
            left += "\n" + self._status
        self.viewer.set_texts(
            [
                (mujoco.mjtFontScale.mjFONTSCALE_150, mujoco.mjtGridPos.mjGRID_TOPLEFT, left, None),
                (mujoco.mjtFontScale.mjFONTSCALE_100, mujoco.mjtGridPos.mjGRID_BOTTOMLEFT, HELP_LEFT, HELP_RIGHT),
            ]
        )

    # ---------------------------------------------------------------- main loop
    def run(self, duration: Optional[float] = None) -> None:
        import mujoco.viewer

        self.model.geom_rgba[self.marker_geom] = self.VIS_RGBA
        self._mirror()
        self._set_mocap(self._ee_pose(self.robot.get_joint_pos()[:6]))
        with mujoco.viewer.launch_passive(self.model, self.data, key_callback=self.keys.put, show_left_ui=False, show_right_ui=False) as v:
            self.viewer = v
            v.cam.azimuth, v.cam.elevation, v.cam.distance = 150, -20, 1.4
            v.cam.lookat[:] = [0.15, 0, 0.2]
            v.opt.geomgroup[3] = 0  # hide finger collision pads
            ctrl = threading.Thread(target=self._control_loop, name="viewer_control", daemon=True)
            ctrl.start()
            if self._start_in_control:
                self.keys.put(KEY["SPACE"])
            print("[viewer] window open. SPACE toggles VIS/CONTROL. Close the window or Ctrl-C to quit.", flush=True)
            t_start = time.time()
            last_overlay = 0.0
            try:
                while v.is_running() and (duration is None or time.time() - t_start < duration):
                    with v.lock():
                        self._mirror()
                        if self.mode == "VIS":
                            self._set_mocap(self._ee_pose(np.asarray(self.robot.get_joint_pos())[:6]))
                        if time.time() - last_overlay > 0.1:
                            self._overlay()
                            last_overlay = time.time()
                    v.sync()
                    time.sleep(self.render_dt)
            except KeyboardInterrupt:
                pass
            finally:
                self.stop.set()
                ctrl.join(timeout=1.0)
        print("[viewer] closed", flush=True)
