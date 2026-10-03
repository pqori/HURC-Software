"""ROS 2 bridge for the I2RT YAM arm: MuJoCo sim (yam_sim) or the real arm (i2rt).

It talks to the robot only through the i2rt ``Robot`` API that ``yam_sim.make_robot`` returns, so
the same node drives ``YamSimRobot`` (backend:=sim) or i2rt's ``MotorChainRobot``
(backend:=real). From the ROS side it looks like the ros2_control setup in
``yam_description/config/yam_controllers.yaml``. Topic and action names are the same:

  publishes
    /joint_states                      sensor_msgs/JointState  <prefix>joint1..6 (rad, rad/s, N m),
                                       <prefix>joint7/8 fingers (m, m/s, gripper motor torque N m)
    ~/target_joint_pos                 sensor_msgs/JointState  what the bridge is commanding right now
    /yam_wrist_camera/image_raw        sensor_msgs/Image       rgb8, frame <prefix>wrist_camera_optical_frame
    /yam_wrist_camera/image_raw/compressed  sensor_msgs/CompressedImage (JPEG)
    /yam_wrist_camera/camera_info      sensor_msgs/CameraInfo  (camera:=none disables all three)
    /yam_keyboard/pressed_keys         std_msgs/String         keys currently down (objects:=keyboard)
    /yam_keyboard/typed                std_msgs/String         one message per key press
    /yam_keyboard/markers              visualization_msgs/MarkerArray (latched)
    /tf_static                         <prefix>base -> <prefix>keyboard
  actions
    /yam_arm_controller/follow_joint_trajectory   control_msgs/FollowJointTrajectory
    /yam_gripper_controller/gripper_cmd           control_msgs/GripperCommand (position in m)
  subscribes
    /yam_arm_controller/joint_trajectory          trajectory_msgs/JointTrajectory (streaming, like JTC)
    /yam_gripper_controller/commands              std_msgs/Float64MultiArray, data[0] = finger opening (m)

Robot command space: 6 arm joints in rad, then gripper in [0, 1] (0 = closed). The finger joints
in ROS are in metres: joint7 = joint8 = gripper * stroke, with a stroke of 0.0475 m
(linear_4310) or 0.039574 m (crank_4310). These are the URDF / MJCF slide ranges.
"""

from __future__ import annotations

import signal
import threading
import time
from dataclasses import dataclass, field
from typing import Optional, Sequence

import numpy as np
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from builtin_interfaces.msg import Duration as DurationMsg
from control_msgs.action import FollowJointTrajectory, GripperCommand
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# Joint ranges from yam_description/urdf/yam_macro.urdf.xacro (= i2rt yam.urdf), in rad.
ARM_LIMITS = np.array([
    [-2.61799, 3.14159],
    [-8.88178e-16, 3.66519],
    [0.0, 3.14159],
    [-1.69297, 1.5708],
    [-1.5708, 1.5708],
    [-2.0944, 2.0944],
])
# Finger slide stroke per gripper (m), = URDF joint7 upper limit = MJCF joint7 range.
GRIPPER_STROKE_M = {"linear_4310": 0.0475, "crank_4310": 0.039574, "none": None}
# URDF/launch name -> yam_sim / i2rt name
GRIPPER_I2RT_NAME = {"linear_4310": "linear_4310", "crank_4310": "crank_4310", "none": "no_gripper"}

R = FollowJointTrajectory.Result
LIMIT_EPS = 1e-6


def _dur(d: DurationMsg) -> float:
    return float(d.sec) + float(d.nanosec) * 1e-9


def _to_dur(t: float) -> DurationMsg:
    t = max(0.0, float(t))
    sec = int(t)
    return DurationMsg(sec=sec, nanosec=int(round((t - sec) * 1e9)))


class TrajectoryError(ValueError):
    def __init__(self, code: int, msg: str):
        super().__init__(msg)
        self.code = code


@dataclass
class Trajectory:
    """A validated arm trajectory in robot joint order, sampled against a wall-clock start time."""

    times: np.ndarray            # (N,) seconds from start, strictly increasing, times[0] == 0
    pos: np.ndarray              # (N, 6)
    vel: Optional[np.ndarray]    # (N, 6) or None (piecewise linear)
    start_wall: float = 0.0
    goal_handle: object = None   # rclpy ServerGoalHandle when it came from the action
    done: threading.Event = field(default_factory=threading.Event)
    outcome: Optional[tuple] = None  # (code, message) set when the controller ends it early

    @property
    def duration(self) -> float:
        return float(self.times[-1])

    def sample(self, t: float):
        """Desired (pos, vel) at ``t`` seconds after start. Clamped to the ends."""
        times, pos = self.times, self.pos
        if t <= 0.0:
            return pos[0].copy(), np.zeros(6)
        if t >= times[-1]:
            return pos[-1].copy(), np.zeros(6)
        i = int(np.searchsorted(times, t, side="right")) - 1
        t0, t1 = times[i], times[i + 1]
        h = t1 - t0
        s = (t - t0) / h
        p0, p1 = pos[i], pos[i + 1]
        if self.vel is None:
            return p0 + s * (p1 - p0), (p1 - p0) / h
        v0, v1 = self.vel[i], self.vel[i + 1]
        # cubic Hermite (position + velocity continuous), as JTC does when velocities are given
        h00 = 2 * s**3 - 3 * s**2 + 1
        h10 = s**3 - 2 * s**2 + s
        h01 = -2 * s**3 + 3 * s**2
        h11 = s**3 - s**2
        p = h00 * p0 + h10 * h * v0 + h01 * p1 + h11 * h * v1
        dh00 = (6 * s**2 - 6 * s) / h
        dh10 = 3 * s**2 - 4 * s + 1
        dh01 = (-6 * s**2 + 6 * s) / h
        dh11 = 3 * s**2 - 2 * s
        v = dh00 * p0 + dh10 * v0 + dh01 * p1 + dh11 * v1
        return p, v


def build_trajectory(
    msg: JointTrajectory, arm_joint_names: Sequence[str], start_pos: np.ndarray
) -> Trajectory:
    """Validate a JointTrajectory, reorder its joints to robot order, and prepend the start pose.

    Raises TrajectoryError with a FollowJointTrajectory error code.
    """
    names = list(msg.joint_names)
    if len(names) != len(set(names)):
        raise TrajectoryError(R.INVALID_JOINTS, f"duplicate joint names in {names}")
    if set(names) != set(arm_joint_names):
        raise TrajectoryError(
            R.INVALID_JOINTS,
            f"trajectory must name exactly the 6 arm joints {list(arm_joint_names)}, got {names}",
        )
    if not msg.points:
        raise TrajectoryError(R.INVALID_GOAL, "trajectory has no points")
    order = [names.index(j) for j in arm_joint_names]
    times, pos, vel = [], [], []
    have_vel = all(len(p.velocities) == len(names) for p in msg.points)
    last_t = -1.0
    for k, p in enumerate(msg.points):
        if len(p.positions) != len(names):
            raise TrajectoryError(R.INVALID_GOAL, f"point {k} has {len(p.positions)} positions, expected {len(names)}")
        if len(p.velocities) not in (0, len(names)):
            raise TrajectoryError(R.INVALID_GOAL, f"point {k} has {len(p.velocities)} velocities")
        t = _dur(p.time_from_start)
        if t <= last_t:
            raise TrajectoryError(R.INVALID_GOAL, f"time_from_start must increase strictly (point {k}: {t:.3f} s)")
        last_t = t
        q = np.array([p.positions[i] for i in order], dtype=float)
        if not np.all(np.isfinite(q)):
            raise TrajectoryError(R.INVALID_GOAL, f"point {k} has non-finite positions")
        lo_bad = q < ARM_LIMITS[:, 0] - LIMIT_EPS
        hi_bad = q > ARM_LIMITS[:, 1] + LIMIT_EPS
        if np.any(lo_bad | hi_bad):
            j = int(np.argmax(lo_bad | hi_bad))
            raise TrajectoryError(
                R.INVALID_GOAL,
                f"point {k}: {arm_joint_names[j]} = {q[j]:.4f} rad is outside "
                f"[{ARM_LIMITS[j, 0]:.4f}, {ARM_LIMITS[j, 1]:.4f}]",
            )
        times.append(t)
        pos.append(q)
        if have_vel:
            vel.append(np.array([p.velocities[i] for i in order], dtype=float))
    start = np.clip(np.asarray(start_pos, dtype=float), ARM_LIMITS[:, 0], ARM_LIMITS[:, 1])
    if times[0] > 0.0:
        times.insert(0, 0.0)
        pos.insert(0, start)
        if have_vel:
            vel.insert(0, np.zeros(6))
    return Trajectory(
        times=np.array(times), pos=np.array(pos), vel=np.array(vel) if have_vel else None
    )


class YamBridge(Node):
    def __init__(self, robot=None):
        super().__init__("yam_sim_bridge")
        self.declare_parameter("prefix", "yam_")
        self.declare_parameter("gripper", "linear_4310")
        self.declare_parameter("rate", 50.0)            # /joint_states publish rate (Hz)
        self.declare_parameter("command_rate", 200.0)   # trajectory sampling / command rate (Hz)
        self.declare_parameter("backend", "sim")         # sim | real
        self.declare_parameter("channel", "can0")        # CAN interface for backend:=real
        self.declare_parameter("hold_on_start", True)    # PD-hold the start pose instead of floating
        self.declare_parameter("goal_tolerance", 0.02)   # rad, default per-joint goal tolerance
        self.declare_parameter("goal_time", 1.0)         # s allowed after the end to reach tolerance
        self.declare_parameter("gripper_goal_tolerance", 0.002)  # m
        self.declare_parameter("gripper_timeout", 5.0)   # s
        self.declare_parameter("sim_realtime", True)
        # wrist webcam + scene objects (see perception.py)
        self.declare_parameter("camera", "c920")           # c920 | c270 | none
        self.declare_parameter("camera_rate", 15.0)        # Hz
        self.declare_parameter("camera_resolution", "")    # "1280x720"; "" = the camera's default
        self.declare_parameter("camera_device", "0")       # backend:=real: OpenCV device index or path
        self.declare_parameter("objects", "")              # sim: "", "keyboard", "cube", "cube,keyboard"
        self.declare_parameter("tool", "none")             # sim: none | stylus
        self.declare_parameter("keyboard_rate", 50.0)      # Hz, /yam_keyboard/pressed_keys
        self.declare_parameter("publish_key_markers", True)
        self.declare_parameter("stats_period", 30.0)       # s between timing reports in the log (0 = off)

        gp = self.get_parameter
        self.prefix = gp("prefix").value
        self.gripper = gp("gripper").value
        if self.gripper == "no_gripper":
            self.gripper = "none"
        if self.gripper not in GRIPPER_STROKE_M:
            raise ValueError(f"gripper must be linear_4310, crank_4310 or none, got {self.gripper!r}")
        self.stroke = GRIPPER_STROKE_M[self.gripper]
        self.has_gripper = self.stroke is not None
        self.backend = gp("backend").value
        self.arm_joints = [f"{self.prefix}joint{i}" for i in range(1, 7)]
        self.camera = str(gp("camera").value or "none").lower()
        if self.camera in ("", "false"):
            self.camera = "none"
        self.objects = str(gp("objects").value or "")
        self.tool = str(gp("tool").value or "none").lower()
        self.finger_joints = [f"{self.prefix}joint7", f"{self.prefix}joint8"] if self.has_gripper else []

        if robot is None:
            from yam_sim import make_robot

            kw = {}
            if self.backend == "sim":
                kw["realtime"] = bool(gp("sim_realtime").value)
                kw["camera"] = self.camera
                kw["objects"] = self.objects
                kw["tool"] = None if self.tool in ("none", "") else self.tool
            self.get_logger().info(
                f"creating robot: backend={self.backend} gripper={GRIPPER_I2RT_NAME[self.gripper]}"
                + (f" channel={gp('channel').value}" if self.backend == "real" else
                   f" camera={self.camera} objects={self.objects or 'none'} tool={self.tool}")
            )
            robot = make_robot(
                self.backend, arm="yam", gripper=GRIPPER_I2RT_NAME[self.gripper],
                channel=gp("channel").value, **kw,
            )
        self.robot = robot
        n = int(robot.num_dofs())
        expected = 7 if self.has_gripper else 6
        if n != expected:
            raise RuntimeError(f"robot has {n} DoF but gripper={self.gripper} needs {expected}")
        self._check_stroke()

        # ---- commanded target (robot command space: rad, gripper [0,1]) --------------------------
        self._lock = threading.RLock()
        q0 = np.asarray(robot.get_joint_pos(), dtype=float)
        self._arm_cmd = np.clip(q0[:6], ARM_LIMITS[:, 0], ARM_LIMITS[:, 1])
        self._arm_vel_cmd = np.zeros(6)
        self._grip_cmd = float(np.clip(q0[6], 0.0, 1.0)) if self.has_gripper else 0.0
        self._traj: Optional[Trajectory] = None
        self._holding = bool(gp("hold_on_start").value)
        if self._holding:
            self._send_command()

        # ---- ROS interfaces -----------------------------------------------------------------------
        self._timer_group = MutuallyExclusiveCallbackGroup()
        self._action_group = ReentrantCallbackGroup()
        self.js_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.target_pub = self.create_publisher(JointState, "~/target_joint_pos", 10)
        self._command_period = 1.0 / float(gp("command_rate").value)
        self._tick_lock = threading.Lock()
        self._tick_times: list = []
        self.create_timer(self._command_period, self._control_tick, callback_group=self._timer_group)
        self.create_timer(1.0 / float(gp("rate").value), self._publish_state, callback_group=self._timer_group)

        self.arm_action = ActionServer(
            self, FollowJointTrajectory, "/yam_arm_controller/follow_joint_trajectory",
            execute_callback=self._execute_fjt, goal_callback=lambda _g: GoalResponse.ACCEPT,
            handle_accepted_callback=self._accept_fjt, cancel_callback=lambda _g: CancelResponse.ACCEPT,
            callback_group=self._action_group,
        )
        self.create_subscription(
            JointTrajectory, "/yam_arm_controller/joint_trajectory", self._on_traj_topic, 10,
            callback_group=self._action_group,
        )
        if self.has_gripper:
            self.gripper_action = ActionServer(
                self, GripperCommand, "/yam_gripper_controller/gripper_cmd",
                execute_callback=self._execute_gripper, cancel_callback=lambda _g: CancelResponse.ACCEPT,
                callback_group=self._action_group,
            )
            self.create_subscription(
                Float64MultiArray, "/yam_gripper_controller/commands", self._on_gripper_topic, 10,
                callback_group=self._action_group,
            )
        # ---- wrist camera + keyboard -----------------------------------------------------------------
        self.camera_streamer = None
        self.keyboard = None
        if self.camera != "none":
            from yam_sim_ros.perception import WristCameraStreamer

            self.camera_streamer = WristCameraStreamer(
                self, self.robot, self.backend, self.camera, float(gp("camera_rate").value),
                str(gp("camera_resolution").value), self.prefix, device=str(gp("camera_device").value),
            )
        if "keyboard" in self.objects.lower():
            if getattr(self.robot, "model", None) is None:
                self.get_logger().warn("objects:=keyboard only exists in the sim; no keyboard topics on this backend")
            else:
                from yam_sim_ros.perception import KeyboardPublisher

                self.keyboard = KeyboardPublisher(
                    self, self.robot, self.prefix, rate=float(gp("keyboard_rate").value),
                    publish_markers=bool(gp("publish_key_markers").value),
                    callback_group=MutuallyExclusiveCallbackGroup(),
                )
        stats_period = float(gp("stats_period").value)
        if stats_period > 0:
            self._stats_first = True
            self._stats_timer = self.create_timer(5.0, lambda: self._report_stats(stats_period),
                                                  callback_group=MutuallyExclusiveCallbackGroup())

        self.get_logger().info(
            f"YAM bridge up: {self.backend} backend, joints {self.arm_joints + self.finger_joints}, "
            f"holding={'yes' if self._holding else 'no (floating until the first command)'}"
        )

    # ============================================================================================
    def _check_stroke(self) -> None:
        """For the sim, cross-check the finger stroke against the MuJoCo joint7 range."""
        model = getattr(self.robot, "model", None)
        if model is None or not self.has_gripper:
            return
        try:
            lo, hi = (float(v) for v in model.jnt_range[model.joint("joint7").id])
        except Exception:
            return
        if abs((hi - lo) - self.stroke) > 1e-6:
            self.get_logger().warn(f"MuJoCo joint7 range {hi - lo} m != URDF stroke {self.stroke} m; using MuJoCo's")
            self.stroke = hi - lo

    def _send_command(self) -> None:
        """Push the current target to the robot (called with self._lock held or from __init__)."""
        if self.has_gripper:
            pos = np.append(self._arm_cmd, self._grip_cmd)
            vel = np.append(self._arm_vel_cmd, 0.0)
        else:
            pos, vel = self._arm_cmd.copy(), self._arm_vel_cmd.copy()
        self.robot.command_joint_state({"pos": pos, "vel": vel})

    def _measured(self):
        obs = self.robot.get_observations()
        return obs

    # ---- periodic ------------------------------------------------------------------------------
    def _report_stats(self, period: float) -> None:
        """Log the command loop's achieved rate / worst gap and the camera's render time."""
        if self._stats_first:  # first report after 5 s, then every `period`
            self._stats_first = False
            self._stats_timer.cancel()
            self._stats_timer = self.create_timer(period, lambda: self._report_stats(period),
                                                  callback_group=MutuallyExclusiveCallbackGroup())
        with self._tick_lock:
            ts, self._tick_times = self._tick_times, []
        parts = []
        if len(ts) > 2:
            dt = np.diff(ts)
            parts.append(
                f"command loop {1.0 / dt.mean():.1f} Hz (target {1.0 / self._command_period:.0f}), "
                f"p99 gap {np.percentile(dt, 99) * 1e3:.1f} ms, max gap {dt.max() * 1e3:.1f} ms"
            )
        if self.camera_streamer is not None:
            s = self.camera_streamer.take_stats()
            if s["render"]:
                r = np.array(s["render"])
                parts.append(f"camera render {r.mean():.1f} ms mean / {r.max():.1f} ms max over {len(r)} frames")
            if s["encode"]:
                parts.append(f"jpeg {np.mean(s['encode']):.1f} ms")
            parts.append(f"camera_info msgs {s['frames']}, late frames {s['late']}")
        if parts:
            self.get_logger().info("stats: " + "; ".join(parts))

    def _control_tick(self) -> None:
        with self._tick_lock:
            if len(self._tick_times) < 50000:  # drained by _report_stats
                self._tick_times.append(time.monotonic())
        with self._lock:
            traj = self._traj
            if traj is not None:
                t = time.monotonic() - traj.start_wall
                p, v = traj.sample(t)
                self._arm_cmd, self._arm_vel_cmd = p, v
                if t >= traj.duration:
                    self._arm_vel_cmd = np.zeros(6)
                    if traj.goal_handle is None:  # topic trajectories just finish
                        self._traj = None
                        traj.done.set()
                self._holding = True
            if self._holding:
                self._send_command()

    def _publish_state(self) -> None:
        obs = self.robot.get_observations()
        now = self.get_clock().now().to_msg()
        msg = JointState()
        msg.header.stamp = now
        msg.name = self.arm_joints + self.finger_joints
        pos = list(map(float, obs["joint_pos"][:6]))
        vel = list(map(float, obs["joint_vel"][:6]))
        eff = list(map(float, obs["joint_eff"][:6]))
        if self.has_gripper:
            g = float(obs["gripper_pos"][0]) * self.stroke
            gv = float(obs["gripper_vel"][0]) * self.stroke
            ge = float(obs["gripper_eff"][0])
            pos += [g, g]
            vel += [gv, gv]
            eff += [ge, ge]
        msg.position, msg.velocity, msg.effort = pos, vel, eff
        self.js_pub.publish(msg)

        with self._lock:
            arm, arm_v, grip = self._arm_cmd.copy(), self._arm_vel_cmd.copy(), self._grip_cmd
        tgt = JointState()
        tgt.header.stamp = now
        tgt.name = list(msg.name)
        tgt.position = list(map(float, arm)) + ([grip * self.stroke] * 2 if self.has_gripper else [])
        tgt.velocity = list(map(float, arm_v)) + ([0.0, 0.0] if self.has_gripper else [])
        self.target_pub.publish(tgt)

    # ---- arm: trajectory start / preempt ---------------------------------------------------------
    def _start_trajectory(self, traj: Trajectory, stamp=None) -> None:
        delay = 0.0
        if stamp is not None and (stamp.sec or stamp.nanosec):
            delay = (rclpy.time.Time.from_msg(stamp) - self.get_clock().now()).nanoseconds * 1e-9
            delay = max(0.0, delay)
        with self._lock:
            old = self._traj
            if old is not None and not old.done.is_set():
                old.outcome = (R.INVALID_GOAL, "preempted by a newer trajectory")
                old.done.set()
            traj.start_wall = time.monotonic() + delay
            self._traj = traj
            self._holding = True

    def _on_traj_topic(self, msg: JointTrajectory) -> None:
        with self._lock:
            start = self._arm_cmd.copy()
        if not msg.points:  # empty trajectory = stop, like JTC
            with self._lock:
                self._stop_and_hold()
            return
        try:
            traj = build_trajectory(msg, self.arm_joints, start)
        except TrajectoryError as e:
            self.get_logger().error(f"ignoring joint_trajectory: {e}")
            return
        self._start_trajectory(traj, msg.header.stamp)

    def _stop_and_hold(self) -> None:
        """Hold at the current commanded position (call with the lock held)."""
        old = self._traj
        self._traj = None
        self._arm_vel_cmd = np.zeros(6)
        if old is not None:
            old.done.set()

    # ---- arm: FollowJointTrajectory --------------------------------------------------------------
    def _accept_fjt(self, goal_handle) -> None:
        goal_handle.execute()

    def _tolerances(self, goal) -> np.ndarray:
        tol = np.full(6, float(self.get_parameter("goal_tolerance").value))
        for jt in goal.goal_tolerance:
            if jt.name in self.arm_joints:
                i = self.arm_joints.index(jt.name)
                if jt.position > 0:
                    tol[i] = jt.position
                elif jt.position < 0:
                    tol[i] = np.inf
        return tol

    def _path_tolerances(self, goal) -> np.ndarray:
        tol = np.full(6, np.inf)
        for jt in goal.path_tolerance:
            if jt.name in self.arm_joints and jt.position > 0:
                tol[self.arm_joints.index(jt.name)] = jt.position
        return tol

    def _execute_fjt(self, goal_handle):
        goal = goal_handle.request
        result = FollowJointTrajectory.Result()
        with self._lock:
            start = self._arm_cmd.copy()
        try:
            traj = build_trajectory(goal.trajectory, self.arm_joints, start)
        except TrajectoryError as e:
            self.get_logger().error(f"rejecting trajectory goal: {e}")
            result.error_code, result.error_string = e.code, str(e)
            goal_handle.abort()
            return result
        traj.goal_handle = goal_handle
        goal_tol = self._tolerances(goal)
        path_tol = self._path_tolerances(goal)
        goal_time = _dur(goal.goal_time_tolerance) or float(self.get_parameter("goal_time").value)
        self._start_trajectory(traj, goal.trajectory.header.stamp)
        self.get_logger().info(f"executing trajectory: {len(traj.times)} points, {traj.duration:.2f} s")

        fb = FollowJointTrajectory.Feedback()
        fb.joint_names = list(self.arm_joints)
        period = 0.05
        while rclpy.ok():
            if traj.done.is_set():  # preempted / stopped
                code, text = traj.outcome or (R.INVALID_GOAL, "trajectory stopped")
                result.error_code, result.error_string = code, text
                goal_handle.abort()
                return result
            if goal_handle.is_cancel_requested:
                with self._lock:
                    if self._traj is traj:
                        self._stop_and_hold()
                goal_handle.canceled()
                result.error_code, result.error_string = R.SUCCESSFUL, "canceled"
                return result
            t = time.monotonic() - traj.start_wall
            desired, dvel = traj.sample(t)
            obs = self.robot.get_observations()
            actual = np.asarray(obs["joint_pos"][:6], dtype=float)
            err = desired - actual
            fb.header.stamp = self.get_clock().now().to_msg()
            fb.desired = JointTrajectoryPoint(positions=list(map(float, desired)), velocities=list(map(float, dvel)), time_from_start=_to_dur(t))
            fb.actual = JointTrajectoryPoint(positions=list(map(float, actual)), velocities=list(map(float, obs["joint_vel"][:6])), time_from_start=_to_dur(t))
            fb.error = JointTrajectoryPoint(positions=list(map(float, err)), time_from_start=_to_dur(t))
            goal_handle.publish_feedback(fb)

            if 0.0 <= t <= traj.duration and np.any(np.abs(err) > path_tol):
                j = int(np.argmax(np.abs(err) - path_tol))
                with self._lock:
                    if self._traj is traj:
                        self._arm_cmd = actual.copy()
                        self._stop_and_hold()
                result.error_code = R.PATH_TOLERANCE_VIOLATED
                result.error_string = f"{self.arm_joints[j]} path error {err[j]:.4f} rad > {path_tol[j]:.4f}"
                goal_handle.abort()
                return result
            if t >= traj.duration:
                final_err = np.abs(traj.pos[-1] - actual)
                if np.all(final_err <= goal_tol):
                    with self._lock:
                        if self._traj is traj:
                            self._traj = None
                            traj.done.set()
                    result.error_code = R.SUCCESSFUL
                    result.error_string = f"reached goal, max error {final_err.max():.4f} rad"
                    goal_handle.succeed()
                    return result
                if t >= traj.duration + goal_time:
                    j = int(np.argmax(final_err - goal_tol))
                    with self._lock:
                        if self._traj is traj:
                            self._traj = None  # keep holding the final point
                            traj.done.set()
                    result.error_code = R.GOAL_TOLERANCE_VIOLATED
                    result.error_string = (
                        f"{self.arm_joints[j]} final error {final_err[j]:.4f} rad > {goal_tol[j]:.4f} "
                        f"after {goal_time:.2f} s"
                    )
                    goal_handle.abort()
                    return result
            time.sleep(period)
        goal_handle.abort()
        return result

    # ---- gripper ---------------------------------------------------------------------------------
    def _set_gripper_m(self, metres: float) -> float:
        g = float(np.clip(metres / self.stroke, 0.0, 1.0))
        with self._lock:
            self._grip_cmd = g
            self._holding = True
            self._send_command()
        return g

    def _on_gripper_topic(self, msg: Float64MultiArray) -> None:
        if len(msg.data) < 1:
            self.get_logger().error("gripper commands need one value (finger opening in m)")
            return
        self._set_gripper_m(float(msg.data[0]))

    def _execute_gripper(self, goal_handle):
        cmd = goal_handle.request.command
        target_m = float(np.clip(cmd.position, 0.0, self.stroke))
        self._set_gripper_m(target_m)
        if cmd.max_effort > 0:
            self.get_logger().info(
                "GripperCommand.max_effort is ignored; the force limit is the i2rt gripper limiter's "
                "(limit_gripper_force, 50 N by default)", once=True,
            )
        tol = float(self.get_parameter("gripper_goal_tolerance").value)
        timeout = float(self.get_parameter("gripper_timeout").value)
        t0 = time.monotonic()
        still_since = None
        res = GripperCommand.Result()
        fb = GripperCommand.Feedback()
        while rclpy.ok():
            obs = self.robot.get_observations()
            pos = float(obs["gripper_pos"][0]) * self.stroke
            vel = float(obs["gripper_vel"][0]) * self.stroke
            eff = float(obs["gripper_eff"][0])
            reached = abs(pos - target_m) <= tol
            elapsed = time.monotonic() - t0
            if abs(vel) < 1e-3 and elapsed > 0.3:
                still_since = still_since or time.monotonic()
            else:
                still_since = None
            stalled = (not reached) and still_since is not None and time.monotonic() - still_since > 0.5
            fb.position, fb.effort, fb.stalled, fb.reached_goal = pos, eff, stalled, reached
            goal_handle.publish_feedback(fb)
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                res.position, res.effort, res.stalled, res.reached_goal = pos, eff, stalled, reached
                return res
            if reached or stalled or elapsed > timeout:
                res.position, res.effort, res.stalled, res.reached_goal = pos, eff, stalled, reached
                # Like GripperActionController: stalled (e.g. holding an object) still counts as success.
                if reached or stalled:
                    goal_handle.succeed()
                else:
                    goal_handle.abort()
                return res
            time.sleep(0.02)
        goal_handle.abort()
        return res

    def close(self) -> None:
        if getattr(self, "camera_streamer", None) is not None:
            self.camera_streamer.close()
        try:
            self.robot.close()
        except Exception as e:  # pragma: no cover
            self.get_logger().warn(f"robot.close() failed: {e}")


def main(args=None) -> None:
    rclpy.init(args=args)
    try:
        node = YamBridge()
    except ImportError as e:  # yam_sim.RealBackendUnavailable: no i2rt on this machine
        rclpy.logging.get_logger("yam_sim_bridge").fatal(str(e))
        rclpy.shutdown()
        raise SystemExit(1)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        # A second SIGINT (ros2 launch forwards one) must not interrupt shutting the robot down.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
