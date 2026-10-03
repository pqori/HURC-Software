"""``YamSimRobot``: a physics simulation of the YAM arm with i2rt's ``Robot`` API.

Every method of i2rt's ``Robot`` protocol that ``MotorChainRobot`` (the real driver) implements
behaves the same way here, so code written against this class runs on the real arm unchanged
through ``i2rt.robots.get_robot.get_yam_robot``. Parts are adapted from i2rt @ 120c3c8 (MIT).

The command conventions are the same as on hardware:

* ``num_dofs()`` is 6 arm joints, plus 1 if there is a gripper.
* Positions are radians. The gripper is normalised to [0, 1] with 0 = closed and 1 = open.
  Gripper velocity is in the same normalised units per second, and gripper effort is motor torque
  (N m).
* ``command_joint_pos`` clips the arm joints to the MJCF ranges widened by 0.15 rad (like
  ``get_yam_robot``) and clips the gripper to [0, 1]. It then switches every motor to PD with the
  configured kp/kd (defaults from ``yam_v1.yml`` and the gripper YAML).
* ``zero_gravity_mode=True`` (the default, as in ``get_yam_robot``) starts with kp = 0, a small
  damping kd (``grav_comp_kd``) and gravity-compensation feedforward only, so the arm floats and
  can be pushed around. The first ``command_joint_pos`` turns position holding on.
* Gravity compensation is ``tau_ff = gravity_comp_factor * g(q)``. It is recomputed from the
  current state at every control tick and sent as the MIT-mode feedforward torque.

Physics:

* The motors' MIT-mode PD runs inside MuJoCo on every physics substep (1 kHz by default). The host
  side (gravity comp, gripper force limiter, the frame contents) updates at ``control_freq``
  (200 Hz by default).
* Torques saturate at each Damiao motor's limit (the peak torque by default).
* Joints have estimated rotor inertia (armature) and Coulomb friction (``frictionloss``, the
  ``coulomb_friction`` values from yam_v1.yml).
* The 400 ms CAN watchdog is modelled. See :class:`yam_sim.motor.Watchdog`.
"""

from __future__ import annotations

import logging
import threading
import time
import warnings
from typing import Any, Dict, Optional, Sequence, Union

import mujoco
import numpy as np

from yam_sim.assembly import ARM_JOINTS, GRIPPER_JOINT, build_scene, load_scene
from yam_sim.config import MOTOR_SPECS, load_arm_config, load_gripper_config
from yam_sim.motor import GripperForceLimiter, MotorCommand, Watchdog, quantize

logger = logging.getLogger(__name__)

_GRAVITY_TORQUE_SANITY_LIMIT = 25.0  # MotorChainRobot raises above this


class YamSimRobot:
    """Physics-simulated YAM arm with i2rt's ``Robot`` interface. See the module docstring."""

    def __init__(
        self,
        arm: str = "yam",
        gripper: str = "linear_4310",
        *,
        zero_gravity_mode: bool = True,
        control_freq: float = 200.0,
        timestep: float = 0.001,
        start_thread: bool = True,
        realtime: bool = True,
        kp: Optional[Sequence[float]] = None,
        kd: Optional[Sequence[float]] = None,
        gripper_kp: Optional[float] = None,
        gripper_kd: Optional[float] = None,
        grav_comp_kd: Optional[Sequence[float]] = None,
        use_gravity_comp: bool = True,
        gravity_comp_factor: Union[None, str, Sequence[float]] = None,
        use_coulomb_friction: bool = False,
        friction: Union[bool, Sequence[float]] = True,
        torque_limit: Union[str, float, Sequence[float]] = "peak",
        armature: Optional[Sequence[float]] = None,
        watchdog: bool = True,
        watchdog_timeout: float = 0.4,
        watchdog_feed: str = "bus",
        watchdog_damping_kd: Optional[Sequence[float]] = None,
        limit_gripper_force: float = 50.0,
        initial_qpos: Optional[Sequence[float]] = None,
        joint_limit_buffer: float = 0.15,
        quantize_feedback: bool = False,
        speed_torque_limit: bool = True,
        objects: Union[bool, str, Sequence[str], None] = False,
        self_collision: bool = False,
        camera: Optional[str] = "c920",
        tool: Optional[str] = None,
        keyboard: Optional[dict] = None,
    ) -> None:
        """
        Args:
            arm, gripper: model variant (only ``yam`` is vendored; grippers ``linear_4310``,
                ``crank_4310``, ``no_gripper``).
            zero_gravity_mode: start floating under gravity comp (True) or holding the initial pose (False).
            control_freq: host control-loop rate (Hz). Physics substeps = round(1 / (control_freq * timestep)).
            timestep: MuJoCo physics timestep (s).
            start_thread: run physics in a background thread in real time (like the real robot).
                With False, nothing moves until you call :meth:`step`, which makes runs deterministic.
            realtime: in thread mode, pace the simulation to wall-clock time (False = as fast as possible).
            kp, kd: arm PD gains (6 values). Default: yam_v1.yml.
            gripper_kp, gripper_kd: gripper motor gains. Default: gripper YAML (20, 0.5).
            grav_comp_kd: damping used in zero-gravity mode. Default: yam_v1.yml.
            use_gravity_comp: add the gravity feedforward (``use_gravity_comp`` in MotorChainRobot).
            gravity_comp_factor: per-arm-joint multiplier. Default 1.0 (the sim model is exact, as
                in i2rt's SimRobot). Pass ``"hardware"`` for the yam_v1.yml factors used on the real arm.
            use_coulomb_friction: add ``coulomb_friction * sign(dq)`` feedforward, as on hardware.
            friction: physical joint friction in the sim (True = yam_v1.yml coulomb_friction values).
            torque_limit: ``"peak"``, ``"rated"``, a number, or one value per motor.
            armature: reflected rotor inertia per arm joint (kg m^2), estimated by default.
            watchdog, watchdog_timeout, watchdog_feed: 400 ms CAN timeout model (see motor.Watchdog).
            watchdog_damping_kd: damping (N m s/rad) once tripped. Default: the joint's kd.
            limit_gripper_force: i2rt's gripper force limiter (N). Set <= 0 to disable. get_yam_robot uses 50.
            initial_qpos: start pose in command space (6 or num_dofs values). Default: zeros, gripper closed.
            joint_limit_buffer: command clipping is the MJCF range +/- this (get_yam_robot uses 0.15).
            quantize_feedback: round feedback through MIT-mode 16/12-bit encodings.
            speed_torque_limit: model the torque-speed curve: the torque available in the
                direction of motion falls linearly from the limit at standstill to zero at
                ``MotorSpec.no_load_speed``.
            objects: ``"cube"`` (or True) adds a graspable cube, ``"keyboard"`` a pressable keyboard,
                ``"cube,keyboard"`` both.
            self_collision: let the arm's links collide with each other.
            camera: wrist webcam on the gripper (``"c920"`` default, ``"c270"``, ``"none"``). Its
                mass is part of the model, so the sim's gravity comp includes it. On hardware,
                tell i2rt about it too (see the README, "Wrist camera and keyboard").
            tool: ``"stylus"`` adds a key-pressing stylus held by the gripper (site ``stylus_tip``).
            keyboard: options for :class:`yam_sim.keyboard.Keyboard` (layout, pos, yaw, ...).
        """
        self._arm_name = arm
        self._gripper_name = gripper
        self._arm_cfg = load_arm_config(arm)
        self._grip_cfg = load_gripper_config(gripper, arm)
        scene_kwargs = dict(
            torque_limit=torque_limit,
            armature=armature,
            friction=friction,
            timestep=timestep,
            objects=objects,
            self_collision=self_collision,
            camera=camera,
            tool=tool,
            keyboard=keyboard,
        )
        self._model, self._scene = load_scene(arm, gripper, **scene_kwargs)
        self._scene_kwargs = scene_kwargs
        self._xml_path: Optional[str] = None
        self._data = mujoco.MjData(self._model)
        self._kdl_data = mujoco.MjData(self._model)

        m = self._model
        self._n_arm = len(self._scene.arm_joints)
        self._has_gripper = self._scene.gripper_joint is not None
        self._gripper_index: Optional[int] = self._n_arm if self._has_gripper else None
        self._n = self._n_arm + (1 if self._has_gripper else 0)

        self._arm_qadr = np.array([m.jnt_qposadr[m.joint(j).id] for j in ARM_JOINTS[: self._n_arm]])
        self._arm_dadr = np.array([m.jnt_dofadr[m.joint(j).id] for j in ARM_JOINTS[: self._n_arm]])
        self._act_ids = np.array(
            [m.actuator(f"{j}_motor").id for j in ARM_JOINTS[: self._n_arm]]
            + ([m.actuator("gripper_motor").id] if self._has_gripper else [])
        )
        if self._has_gripper:
            gj = m.joint(GRIPPER_JOINT)
            self._g_qadr = int(m.jnt_qposadr[gj.id])
            self._g_dadr = int(m.jnt_dofadr[gj.id])
            self._g_tendon = int(m.tendon("gripper_drive").id)
            self._g_slide_lo, self._g_slide_hi = (float(v) for v in m.jnt_range[gj.id])
            self._g_gear = float(m.actuator_gear[self._act_ids[-1], 0])
            self._g_stroke = float(self._scene.gripper_motor_stroke)
            self._g_raw_limits = np.array([0.0, self._g_stroke])  # [closed, open] in motor rad
        # The coupled finger (joint8) and its equality constraint, used when teleporting the gripper.
        self._eq_pairs = []
        for i in range(m.neq):
            if m.eq_type[i] == mujoco.mjtEq.mjEQ_JOINT:
                self._eq_pairs.append(
                    (int(m.jnt_qposadr[m.eq_obj1id[i]]), int(m.jnt_qposadr[m.eq_obj2id[i]]), m.eq_data[i, :5].copy())
                )

        # --- gains and config (ordered like MotorChainRobot: arm joints, then gripper) -------------
        def _vec(v, default, n):
            return np.array(default if v is None else v, dtype=float).reshape(n).copy()

        arm_kp = _vec(kp, self._arm_cfg.kp, self._n_arm)
        arm_kd = _vec(kd, self._arm_cfg.kd, self._n_arm)
        self._kp = arm_kp
        self._kd = arm_kd
        self._grav_comp_kd = _vec(grav_comp_kd, self._arm_cfg.grav_comp_kd, self._n_arm)
        self._coulomb_friction = self._arm_cfg.coulomb_friction.copy()
        if isinstance(gravity_comp_factor, str):
            if gravity_comp_factor != "hardware":
                raise ValueError("gravity_comp_factor must be None, 'hardware', or a sequence")
            gcf = self._arm_cfg.gravity_comp_factor.copy()
        else:
            gcf = _vec(gravity_comp_factor, np.ones(self._n_arm), self._n_arm)
        self.gravity_comp_factor = gcf
        if self._has_gripper:
            self._kp = np.append(self._kp, self._grip_cfg.motor_kp if gripper_kp is None else gripper_kp)
            self._kd = np.append(self._kd, self._grip_cfg.motor_kd if gripper_kd is None else gripper_kd)
            self._grav_comp_kd = np.append(self._grav_comp_kd, 0.0)
            self._coulomb_friction = np.append(self._coulomb_friction, 0.0)
            self.gravity_comp_factor = np.append(self.gravity_comp_factor, 1.0)
        self.use_gravity_comp = bool(use_gravity_comp)
        self.use_coulomb_friction = bool(use_coulomb_friction)

        specs = [MOTOR_SPECS[t] for t in self._scene.motor_types]
        self._specs = specs
        self._kp_max = np.array([s.kp_max for s in specs])
        self._kd_max = np.array([s.kd_max for s in specs])
        self._q_lo = -np.array([s.pmax for s in specs])
        self._v_lo = -np.array([s.vmax for s in specs])
        self._t_lo = -np.array([s.tmax for s in specs])
        self._quantize = bool(quantize_feedback)
        self._speed_limit = bool(speed_torque_limit)
        self._tau_max = self._scene.torque_limits.astype(float).copy()
        self._w0 = np.array([s.no_load_speed for s in specs])

        xml_ranges = self._scene.xml_joint_ranges.copy()
        self._joint_limits = xml_ranges.copy()
        self._joint_limits[:, 0] -= joint_limit_buffer
        self._joint_limits[:, 1] += joint_limit_buffer
        self._gripper_limits = np.array([0.0, 1.0]) if self._has_gripper else None

        self._watchdog = Watchdog(watchdog_timeout, enabled=watchdog, feed=watchdog_feed)
        self._wd_kd = _vec(watchdog_damping_kd, self._kd, self._n)
        self._bus_stall_until = -np.inf

        self._limit_gripper_force = float(limit_gripper_force) if self._has_gripper else -1.0
        self._gripper_limiter: Optional[GripperForceLimiter] = None
        if self._has_gripper and self._limit_gripper_force > 0 and self._grip_cfg.limiter:
            self._gripper_limiter = GripperForceLimiter(
                self._limit_gripper_force, self._grip_cfg.limiter, kp=float(self._kp[self._gripper_index])
            )

        self._control_freq = float(control_freq)
        self._control_dt = 1.0 / self._control_freq
        self._n_substeps = max(1, int(round(self._control_dt / self._model.opt.timestep)))
        self._realtime = realtime

        # --- runtime state ------------------------------------------------------------------------
        self._lock = threading.RLock()
        self._zero_gravity_mode_init = bool(zero_gravity_mode)
        self._commands = MotorCommand.zeros(self._n)
        self._last_motor_torques = np.zeros(self._n)
        self._last_gripper_command_qpos = self._g_stroke if self._has_gripper else 0.0
        self._warned: set = set()
        self._tick_count = 0
        self._rtf = 0.0
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None

        self.reset(initial_qpos)
        if start_thread:
            self.start_server()

    # ============================================================================================
    # Raw motor space <-> command space
    # ============================================================================================
    def _to_raw_pos(self, pos: np.ndarray) -> np.ndarray:
        raw = np.array(pos, dtype=float).copy()
        if self._has_gripper:
            raw[self._gripper_index] = raw[self._gripper_index] * self._g_stroke
        return raw

    def _to_raw_vel(self, vel: np.ndarray) -> np.ndarray:
        return self._to_raw_pos(vel)

    def _to_cmd_pos(self, raw: np.ndarray) -> np.ndarray:
        out = np.array(raw, dtype=float).copy()
        if self._has_gripper:
            out[self._gripper_index] = out[self._gripper_index] / self._g_stroke
        return out

    def _read_raw_state(self):
        d = self._data
        q = d.qpos[self._arm_qadr].copy()
        dq = d.qvel[self._arm_dadr].copy()
        eff = d.actuator_force[self._act_ids].copy()
        if self._has_gripper:
            # The motor angle follows the drive tendon (the mean finger travel).
            theta = (d.ten_length[self._g_tendon] - self._g_slide_lo) * self._g_gear
            dtheta = d.ten_velocity[self._g_tendon] * self._g_gear
            q = np.append(q, theta)
            dq = np.append(dq, dtheta)
        if self._quantize:
            q = quantize(q, self._q_lo, -self._q_lo, 16)
            dq = quantize(dq, self._v_lo, -self._v_lo, 12)
            eff = quantize(eff, self._t_lo, -self._t_lo, 12)
        return q, dq, eff

    def _set_qpos_from_cmd(self, q_cmd: np.ndarray) -> None:
        d = self._data
        d.qpos[self._arm_qadr] = q_cmd[: self._n_arm]
        if self._has_gripper and len(q_cmd) > self._n_arm:
            g = float(np.clip(q_cmd[self._gripper_index], 0.0, 1.0))
            d.qpos[self._g_qadr] = self._g_slide_lo + g * (self._g_slide_hi - self._g_slide_lo)
        for a1, a2, coef in self._eq_pairs:
            d.qpos[a2] = np.polyval(coef[::-1], d.qpos[a1])

    # ============================================================================================
    # Control tick (the "host + CAN bus" side) and physics
    # ============================================================================================
    def _gravity_torques(self) -> np.ndarray:
        kd_ = self._kdl_data
        kd_.qpos[:] = self._data.qpos
        kd_.qvel[:] = 0.0
        mujoco.mj_kinematics(self._model, kd_)
        mujoco.mj_comPos(self._model, kd_)
        out = np.zeros(self._model.nv)
        mujoco.mj_rne(self._model, kd_, 0, out)
        g = np.zeros(self._n)
        g[: self._n_arm] = out[self._arm_dadr]
        if np.max(np.abs(g)) > _GRAVITY_TORQUE_SANITY_LIMIT:
            self._warn_once("gravity", f"gravity torque {g.round(2)} exceeds 25 N m; the real driver would raise here")
        return g

    def _update_torque_limits(self, dq_raw: np.ndarray) -> None:
        if not self._speed_limit:
            return
        frac = np.clip(1.0 - np.abs(dq_raw) / self._w0, 0.0, 1.0)
        upper = np.where(dq_raw > 0, self._tau_max * frac, self._tau_max)
        lower = np.where(dq_raw < 0, -self._tau_max * frac, -self._tau_max)
        self._model.actuator_forcerange[self._act_ids, 0] = lower
        self._model.actuator_forcerange[self._act_ids, 1] = upper

    def _apply_frame(self, kp: np.ndarray, kd: np.ndarray, pos: np.ndarray, vel: np.ndarray, tau: np.ndarray) -> None:
        m, d = self._model, self._data
        ids = self._act_ids
        m.actuator_biasprm[ids, 1] = -kp
        m.actuator_biasprm[ids, 2] = -kd
        d.ctrl[ids] = kp * pos + kd * vel + tau

    def _control_tick(self) -> None:
        now = self._data.time
        bus_alive = now >= self._bus_stall_until
        if bus_alive and self._watchdog.feed_source == "bus":
            self._watchdog.feed(now)
        tripped = self._watchdog.check(now)
        if self._speed_limit:
            _, dq_now, _ = self._read_raw_state()
            self._update_torque_limits(dq_now)

        if tripped:
            zeros = np.zeros(self._n)
            self._apply_frame(zeros, self._wd_kd, zeros, zeros, zeros)
            self._last_motor_torques = zeros
        elif bus_alive:
            q, dq, eff = self._read_raw_state()
            frame = self._commands.copy()
            g = self._gravity_torques() if self.use_gravity_comp else np.zeros(self._n)
            fric = self._coulomb_friction * np.sign(dq) if self.use_coulomb_friction else 0.0
            tau = frame.torque + g * self.gravity_comp_factor + fric
            self._last_motor_torques = tau.copy()
            if self._has_gripper:
                gi = self._gripper_index
                if self._gripper_limiter is not None:
                    state = {
                        "target_qpos": frame.pos[gi],
                        "current_qpos": q[gi],
                        "current_qvel": dq[gi] / self._g_stroke,
                        "current_eff": eff[gi],
                        "current_normalized_qpos": q[gi] / self._g_stroke,
                        "target_normalized_qpos": frame.pos[gi] / self._g_stroke,
                        "last_command_qpos": self._last_gripper_command_qpos,
                    }
                    frame.pos[gi] = self._gripper_limiter.update(now, state)
                frame.pos[gi] = float(np.clip(frame.pos[gi], *self._g_raw_limits))
                self._last_gripper_command_qpos = frame.pos[gi]
            kp = np.clip(frame.kp, 0.0, self._kp_max)
            kd = np.clip(frame.kd, 0.0, self._kd_max)
            self._apply_frame(kp, kd, frame.pos, frame.vel, tau)
        # else: bus stalled, so the motors keep executing the last frame they received.

        for _ in range(self._n_substeps):
            mujoco.mj_step(self._model, self._data)
        self._tick_count += 1

    def step(self, n: int = 1) -> None:
        """Advance ``n`` control ticks (``n / control_freq`` seconds). Only valid without the thread."""
        if self._thread is not None and self._thread.is_alive():
            raise RuntimeError("step() is for synchronous use; this robot has a running physics thread")
        with self._lock:
            for _ in range(int(n)):
                self._control_tick()

    def step_for(self, seconds: float) -> None:
        """Advance the synchronous simulation by ``seconds`` of sim time."""
        self.step(int(round(seconds * self._control_freq)))

    def _loop(self) -> None:
        dt = self._control_dt
        t_wall0 = time.perf_counter()
        t_sim0 = self._data.time
        next_t = time.perf_counter()
        while not self._stop_event.is_set():
            with self._lock:
                self._control_tick()
                sim_elapsed = self._data.time - t_sim0
            wall_elapsed = time.perf_counter() - t_wall0
            if wall_elapsed > 1.0:
                self._rtf = sim_elapsed / wall_elapsed
                t_wall0, t_sim0 = time.perf_counter(), self._data.time
            if self._realtime:
                next_t += dt
                sleep = next_t - time.perf_counter()
                if sleep > 0:
                    time.sleep(sleep)
                elif sleep < -0.25:  # fell far behind (debugger, laptop sleep): resynchronise
                    next_t = time.perf_counter()
            else:
                time.sleep(0)  # yield the GIL

    def start_server(self) -> None:
        """Start the real-time physics thread (no-op if running). Name kept from i2rt's SimRobot."""
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._loop, name="yam_sim_physics", daemon=True)
        self._thread.start()

    # ============================================================================================
    # i2rt Robot protocol
    # ============================================================================================
    def num_dofs(self) -> int:
        return self._n

    def get_joint_pos(self) -> np.ndarray:
        with self._lock:
            q, _, _ = self._read_raw_state()
        return self._to_cmd_pos(q)

    def get_joint_state(self) -> Dict[str, np.ndarray]:
        with self._lock:
            q, dq, _ = self._read_raw_state()
        return {"pos": self._to_cmd_pos(q), "vel": self._to_cmd_pos(dq)}

    def _clip_command(self, pos: np.ndarray) -> np.ndarray:
        pos = np.array(pos, dtype=float).copy()
        if pos.shape != (self._n,):
            raise ValueError(f"expected {self._n} joint values (arm + gripper), got shape {pos.shape}")
        pos[: self._n_arm] = np.clip(pos[: self._n_arm], self._joint_limits[:, 0], self._joint_limits[:, 1])
        if self._has_gripper:
            pos[self._gripper_index] = np.clip(pos[self._gripper_index], 0.0, 1.0)
        return pos

    def _on_user_command(self) -> None:
        if self._watchdog.feed_source == "command":
            self._watchdog.feed(self._data.time)

    def command_joint_pos(self, joint_pos: np.ndarray) -> None:
        """PD-track ``joint_pos`` (rad, gripper in [0, 1]) with the configured kp/kd."""
        pos = self._clip_command(joint_pos)
        with self._lock:
            cmd = MotorCommand.zeros(self._n)
            cmd.pos = self._to_raw_pos(pos)
            cmd.kp = self._kp.copy()
            cmd.kd = self._kd.copy()
            self._commands = cmd
            self._on_user_command()

    def command_joint_state(self, joint_state: Dict[str, np.ndarray]) -> None:
        """Command ``{"pos", "vel", optional "kp", "kd"}``, as in MotorChainRobot.

        Unlike the real driver, ``vel`` may be omitted (defaults to zero).
        """
        pos = self._clip_command(joint_state["pos"])
        vel = np.asarray(joint_state.get("vel", np.zeros(self._n)), dtype=float)
        kp = np.asarray(joint_state.get("kp", self._kp), dtype=float)
        kd = np.asarray(joint_state.get("kd", self._kd), dtype=float)
        with self._lock:
            cmd = MotorCommand.zeros(self._n)
            cmd.pos = self._to_raw_pos(pos)
            cmd.vel = self._to_raw_vel(vel)
            cmd.kp = np.broadcast_to(kp, (self._n,)).copy()
            cmd.kd = np.broadcast_to(kd, (self._n,)).copy()
            self._commands = cmd
            self._on_user_command()

    def command_target_vel(self, joint_vel: np.ndarray) -> None:
        """Does nothing, exactly like the real driver.

        ``MotorChainRobot`` does not override the Robot protocol's empty ``command_target_vel``,
        so it is a no-op on hardware. To command velocities, use ``command_joint_state`` with a
        ``vel`` entry.
        """
        self._warn_once(
            "target_vel",
            "command_target_vel() is a no-op on the real i2rt MotorChainRobot, so the sim ignores it too. "
            "Use command_joint_state({'pos': ..., 'vel': ...}) instead.",
        )

    def get_observations(self) -> Dict[str, np.ndarray]:
        with self._lock:
            q, dq, eff = self._read_raw_state()
        pos, vel = self._to_cmd_pos(q), self._to_cmd_pos(dq)
        if self._gripper_index is None:
            return {"joint_pos": pos, "joint_vel": vel, "joint_eff": eff}
        gi = self._gripper_index
        return {
            "joint_pos": pos[:gi],
            "joint_vel": vel[:gi],
            "joint_eff": eff[:gi],
            "gripper_pos": np.array([pos[gi]]),
            "gripper_vel": np.array([vel[gi]]),
            "gripper_eff": np.array([eff[gi]]),
        }

    def get_robot_info(self) -> Dict[str, Any]:
        info: Dict[str, Any] = {
            # keys of MotorChainRobot.get_robot_info (arm/gripper types are strings, not i2rt enums)
            "arm_type": self._arm_name,
            "gripper_type": self._gripper_name,
            "kp": self._kp.copy(),
            "kd": self._kd.copy(),
            "grav_comp_kd": self._grav_comp_kd.copy(),
            "coulomb_friction": self._coulomb_friction.copy(),
            "use_coulomb_friction": self.use_coulomb_friction,
            "joint_limits": self._joint_limits.copy(),
            "gripper_limits": None if self._gripper_limits is None else self._gripper_limits.copy(),
            "gravity_comp_factor": self.gravity_comp_factor.copy(),
            "gripper_index": self._gripper_index,
            "enable_auto_recovery": False,
            # key of i2rt's SimRobot
            "sim": True,
            # sim-only extras
            "motor_types": list(self._scene.motor_types),
            "torque_limits": self._scene.torque_limits.copy(),
            "control_freq": self._control_freq,
            "timestep": float(self._model.opt.timestep),
            "n_substeps": self._n_substeps,
            "sim_time": float(self._data.time),
            "realtime_factor": self._rtf,
            "watchdog": {
                "enabled": self._watchdog.enabled,
                "timeout": self._watchdog.timeout,
                "feed": self._watchdog.feed_source,
                "tripped": self._watchdog.tripped,
                "trip_time": self._watchdog.trip_time,
            },
            "gripper_motor_stroke_rad": self._g_stroke if self._has_gripper else None,
        }
        if self._gripper_index is not None:
            info["limit_gripper_effort"] = self._limit_gripper_force
            info["gripper_clogged"] = bool(self._gripper_limiter.is_clogged) if self._gripper_limiter else False
        return info

    def get_robot_type(self) -> str:
        return "arm"

    def joint_pos_spec(self) -> Dict[str, Any]:
        return {"shape": (self._n,), "dtype": np.float32}

    def joint_state_spec(self) -> Dict[str, Any]:
        return {"pos": self.joint_pos_spec(), "vel": self.joint_pos_spec()}

    def reinit(self) -> None:
        """Re-enable the motors after a watchdog trip, like power-cycling or re-enabling them. Pose is kept."""
        with self._lock:
            self._watchdog.reset(self._data.time)
            self._bus_stall_until = -np.inf

    def close(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None

    def __enter__(self) -> "YamSimRobot":
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.close()

    # ============================================================================================
    # Other public MotorChainRobot methods
    # ============================================================================================
    def get_motor_torques(self) -> np.ndarray:
        """The last feedforward torque sent (gravity comp + friction comp), as in MotorChainRobot.

        This does not include the PD term. For the total torque each motor applied, use
        :meth:`get_applied_torques` or ``get_observations()["joint_eff"]``.
        """
        with self._lock:
            return self._last_motor_torques.copy()

    def zero_torque_mode(self) -> None:
        with self._lock:
            self._commands = MotorCommand.zeros(self._n)
            self._kp = np.zeros(self._n)
            self._kd = np.zeros(self._n)

    def update_kp_kd(self, kp: np.ndarray, kd: np.ndarray) -> None:
        kp, kd = np.asarray(kp, dtype=float), np.asarray(kd, dtype=float)
        assert kp.shape == self._kp.shape == kd.shape
        with self._lock:
            self._kp, self._kd = kp.copy(), kd.copy()

    def enter_gravity_comp_idle(self) -> None:
        with self._lock:
            cmd = MotorCommand.zeros(self._n)
            cmd.kd = self._grav_comp_kd.copy()
            self._commands = cmd

    def move_joints(self, target_joint_positions: np.ndarray, time_interval_s: float = 2.0) -> None:
        """Linearly interpolate to a target over ``time_interval_s`` (50 steps), as in MotorChainRobot."""
        current = self.get_joint_pos()
        target = np.asarray(target_joint_positions, dtype=float)
        assert len(current) == len(target)
        steps = 50
        for i in range(steps + 1):
            a = i / steps
            self.command_joint_pos((1 - a) * current + a * target)
            self._wait(time_interval_s / steps)

    # SimRobot compatibility (i2rt's MujocoControlInterface calls these when info["sim"] is True)
    def enable_gravity_comp(self) -> None:
        self.enter_gravity_comp_idle()

    def disable_gravity_comp(self) -> None:
        self.command_joint_pos(self.get_joint_pos())

    # ============================================================================================
    # Sim-only helpers
    # ============================================================================================
    def reset(self, q: Optional[Sequence[float]] = None, hold: Optional[bool] = None) -> None:
        """Teleport to ``q`` (command space; 6 values or num_dofs; default all zeros, gripper closed).

        Velocities are zeroed and the watchdog is cleared. With ``hold=True`` the motors PD-hold
        ``q``. With ``hold=False`` they go back to zero-gravity idle. The default is the
        ``zero_gravity_mode`` the robot was built with.
        """
        q_cmd = np.zeros(self._n)
        if q is not None:
            q = np.asarray(q, dtype=float)
            q_cmd[: len(q)] = q
        with self._lock:
            mujoco.mj_resetData(self._model, self._data)
            self._set_qpos_from_cmd(q_cmd)
            mujoco.mj_forward(self._model, self._data)
            self._watchdog.reset(self._data.time if self._watchdog.feed_source == "bus" else None)
            self._bus_stall_until = -np.inf
            if self._gripper_limiter is not None:
                self._gripper_limiter = GripperForceLimiter(
                    self._limit_gripper_force, self._grip_cfg.limiter, kp=float(self._kp[self._gripper_index])
                )
            self._last_gripper_command_qpos = self._g_stroke if self._has_gripper else 0.0
            hold = (not self._zero_gravity_mode_init) if hold is None else hold
            if hold:
                self.command_joint_pos(q_cmd)
            else:
                self.enter_gravity_comp_idle()
            # Sends the first frame to the actuators so forward() and get_observations() agree
            # with the commanded state before the first step.
            self._prime_actuators()

    def _prime_actuators(self) -> None:
        q, dq, _ = self._read_raw_state()
        g = self._gravity_torques() if self.use_gravity_comp else np.zeros(self._n)
        c = self._commands
        self._apply_frame(np.clip(c.kp, 0, self._kp_max), np.clip(c.kd, 0, self._kd_max), c.pos, c.vel, c.torque + g * self.gravity_comp_factor)
        mujoco.mj_forward(self._model, self._data)

    def stall_bus(self, seconds: float) -> None:
        """Simulate the host control loop freezing for ``seconds`` of sim time.

        While stalled, the motors keep executing the last frame they got. Once the stall is longer
        than the watchdog timeout, they drop into damping mode.
        """
        with self._lock:
            self._bus_stall_until = self._data.time + float(seconds)

    def set_external_forces(self, xfrc_applied: Optional[np.ndarray]) -> None:
        """Apply Cartesian forces/torques to bodies, as ``MjData.xfrc_applied`` (nbody x 6, world frame).

        The viewer uses this to pass on its mouse perturbations, so you can push the floating arm
        around in zero-gravity mode the way you would guide the real one by hand.
        """
        with self._lock:
            if xfrc_applied is None:
                self._data.xfrc_applied[:] = 0.0
            else:
                self._data.xfrc_applied[:] = xfrc_applied

    def get_applied_torques(self) -> np.ndarray:
        """Torque each motor is producing right now (N m, after saturation)."""
        with self._lock:
            return self._data.actuator_force[self._act_ids].copy()

    def get_commanded_pos(self) -> np.ndarray:
        """The current PD target in command space (gripper normalised)."""
        with self._lock:
            return self._to_cmd_pos(self._commands.pos)

    def get_command_gains(self) -> Dict[str, np.ndarray]:
        with self._lock:
            return {"kp": self._commands.kp.copy(), "kd": self._commands.kd.copy()}

    def _commands_are_idle(self) -> bool:
        return not np.any(self._commands.kp > 0)

    @property
    def watchdog_tripped(self) -> bool:
        return self._watchdog.tripped

    @property
    def sim_time(self) -> float:
        return float(self._data.time)

    @property
    def model(self) -> mujoco.MjModel:
        return self._model

    @property
    def data(self) -> mujoco.MjData:
        """The live MjData. Hold :attr:`lock` while you read it if the physics thread is running."""
        return self._data

    @property
    def lock(self) -> threading.RLock:
        return self._lock

    @property
    def scene_info(self):
        return self._scene

    @property
    def xml_path(self) -> str:
        """Path of the scene MJCF, written on first access (MotorChainRobot also has ``xml_path``)."""
        if self._xml_path is None:
            import hashlib
            import os
            import tempfile

            xml = build_scene(self._arm_name, self._gripper_name, **self._scene_kwargs)
            out_dir = os.path.join(tempfile.gettempdir(), "yam_sim")
            os.makedirs(out_dir, exist_ok=True)
            path = os.path.join(out_dir, f"{self._arm_name}_{self._gripper_name}_{hashlib.sha1(xml.encode()).hexdigest()[:10]}.xml")
            with open(path, "w") as f:
                f.write(xml)
            self._xml_path = path
        return self._xml_path

    def copy_state_to(self, data: mujoco.MjData) -> None:
        """Copy the full simulation state (qpos, qvel) into another MjData built from the same scene."""
        with self._lock:
            data.qpos[:] = self._data.qpos
            data.qvel[:] = self._data.qvel
            data.time = self._data.time

    def get_ee_pose(self, site: str = "grasp_site") -> np.ndarray:
        """4x4 pose of a site in the base/world frame at the current state."""
        with self._lock:
            sid = self._model.site(site).id
            T = np.eye(4)
            T[:3, :3] = self._data.site_xmat[sid].reshape(3, 3)
            T[:3, 3] = self._data.site_xpos[sid]
        return T

    def is_threaded(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def _wait(self, seconds: float) -> None:
        if self.is_threaded():
            time.sleep(seconds)
        else:
            self.step_for(seconds)

    def _warn_once(self, key: str, msg: str) -> None:
        if key not in self._warned:
            self._warned.add(key)
            warnings.warn(msg, stacklevel=3)
            logger.warning(msg)

    def __repr__(self) -> str:
        return f"YamSimRobot(arm={self._arm_name!r}, gripper={self._gripper_name!r}, dofs={self._n})"
