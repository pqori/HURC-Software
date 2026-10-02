# yam_sim_ros

ROS 2 Humble bridge for the I2RT YAM arm. One node, `bridge_node`, owns a robot
object from `yam_sim.make_robot(...)`:

* `backend:=sim` (default): `YamSimRobot`, the MuJoCo physics sim in `../arm_sim`
* `backend:=real`: i2rt's `MotorChainRobot` on the real arm over CAN

Both implement i2rt's `Robot` API, so the same node drives either one. On the
ROS side it uses the topic and action names from the ros2_control setup in
`yam_description/config/yam_controllers.yaml`. MoveIt, the base station and
scripts therefore don't need to know whether they are talking to the sim, the
real arm, or (later) a Gazebo/ros2_control stack.

## Interfaces

Names below assume the default `prefix:=yam_`.

| Interface | Type | Notes |
| --- | --- | --- |
| `/joint_states` (pub) | `sensor_msgs/JointState` | `yam_joint1..6`: rad, rad/s, N·m (the motor torque the sim applies). `yam_joint7`, `yam_joint8`: finger opening in m and m/s. Effort for the fingers is the gripper motor torque (N·m). Published at `rate` (default 50 Hz). |
| `~/target_joint_pos` (pub) | `sensor_msgs/JointState` | What the bridge is commanding right now: the interpolated trajectory setpoint and its velocity, and the gripper target in m. Full name `/yam_sim_bridge/target_joint_pos`. |
| `/yam_arm_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | See "Trajectory execution" below. |
| `/yam_arm_controller/joint_trajectory` (sub) | `trajectory_msgs/JointTrajectory` | Streaming, as with a JTC. A new message replaces the running trajectory. An empty `points` list means stop and hold. |
| `/yam_gripper_controller/gripper_cmd` | `control_msgs/action/GripperCommand` | `command.position` = finger opening in m (0 = closed, 0.0475 = open for linear_4310, 0.039574 for crank_4310). Succeeds when within `gripper_goal_tolerance` (2 mm) or when stalled on an object. `max_effort` is ignored: the force is capped by the i2rt gripper force limiter, which is 50 N by default. |
| `/yam_gripper_controller/commands` (sub) | `std_msgs/Float64MultiArray` | `data[0]` = finger opening in m, as with a `JointGroupPositionController`. |

The `gripper_cmd` action is the interface MoveIt's `GripperCommand` controller
handler expects. `yam_controllers.yaml` currently defines the gripper as a
`JointGroupPositionController` (`/commands`). The bridge serves both.

### How this maps to the i2rt API

| ROS | i2rt `Robot` call |
| --- | --- |
| `/joint_states` | `get_observations()`: `joint_pos/vel/eff` (6) and `gripper_pos/vel/eff` (gripper ×`stroke`) |
| every command tick (`command_rate`, 200 Hz) | `command_joint_state({"pos": [q1..q6, g], "vel": [dq1..dq6, 0]})`, with `g = opening_m / stroke` in [0, 1] |
| startup (`hold_on_start:=true`) | `command_joint_state` at the measured pose, so the arm holds still. i2rt otherwise starts floating in zero-gravity mode. |
| shutdown | `close()` |

The default PD gains (kp = [80, 80, 80, 10, 10, 10], kd = [5, 5, 5, 1.5, 1.5, 1.5])
come from the i2rt configs and run inside the robot (motor-side PD in the sim,
on the Damiao motors in reality). The bridge streams position setpoints with a
velocity feedforward.

### Trajectory execution

* Joint names can come in any order. They must be exactly the six arm joints:
  any other set is aborted with `INVALID_JOINTS` (-2).
* Every point must be within the URDF joint ranges. Times must strictly
  increase, and positions/velocities must have the right length. Otherwise the
  goal is aborted with `INVALID_GOAL` (-1), and `error_string` names the
  offending joint.
* If the first point is not at t = 0, the trajectory starts from the current
  commanded pose. Points are interpolated with cubic Hermite splines when every
  point has velocities, and linearly otherwise. Accelerations are ignored.
  `header.stamp` in the future delays the start.
* Path tolerance (`path_tolerance`) is checked while the trajectory runs; on a
  violation the arm holds where it is and the goal ends with
  `PATH_TOLERANCE_VIOLATED` (-4).
* The goal tolerance is `goal_tolerance` per joint, or the `goal_tolerance`
  parameter (default 0.02 rad). It must be met within `goal_time_tolerance`
  (or the `goal_time` parameter, default 1 s) after the end. Success returns
  `SUCCESSFUL` (0); failure returns `GOAL_TOLERANCE_VIOLATED` (-5).
* A new goal (or a `/joint_trajectory` message) **preempts** the running goal.
  The old goal is aborted with `error_string: "preempted by a newer trajectory"`.
  A cancel request stops and holds at the current setpoint.
* Feedback (`desired` / `actual` / `error`) is published at 20 Hz.

### Parameters

`prefix` (`yam_`), `gripper` (`linear_4310` | `crank_4310` | `none`; must match
the URDF), `rate` (50), `command_rate` (200), `backend` (`sim` | `real`),
`channel` (`can0`), `hold_on_start` (true), `goal_tolerance` (0.02),
`goal_time` (1.0), `gripper_goal_tolerance` (0.002), `gripper_timeout` (5.0),
`sim_realtime` (true).

## Running it

With pixi, from this directory (`pixi.toml` is self-contained, and the sim is
installed editable from `../arm_sim`):

```bash
pixi install          # first time only
pixi run build        # colcon build of yam_description + yam_sim_ros at the repo root
pixi run bridge       # sim + bridge + robot_state_publisher, headless
pixi run view         # same, with RViz
pixi run test         # headless integration test
```

Plain ROS (any Humble install that also has `mujoco` and `pip install -e arm_sim`):

```bash
colcon build --symlink-install --packages-select yam_description yam_sim_ros
source install/setup.bash
ros2 launch yam_sim_ros sim_bridge.launch.py                 # args: gripper, prefix, backend, channel, rate, rviz
ros2 action send_goal /yam_arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [yam_joint1, yam_joint2, yam_joint3, yam_joint4, yam_joint5, yam_joint6],
    points: [{positions: [0.5, 1.0, 0.8, -0.3, 0.4, 0.6], time_from_start: {sec: 3}}]}}"
ros2 action send_goal /yam_gripper_controller/gripper_cmd control_msgs/action/GripperCommand "{command: {position: 0.03}}"
ros2 run tf2_ros tf2_echo yam_base yam_grasp
```

Set a non-zero `ROS_DOMAIN_ID` if other ROS graphs share the machine. The pixi
env sets `ROS_LOCALHOST_ONLY=1`.

**MuJoCo viewer:** the launch file has no viewer option.
`yam_sim.scripts.run_sim` (the MuJoCo viewer) creates its own robot in its own
process and can't attach to the one the bridge owns. `ArmViewer` also sends its
own commands (VIS mode = gravity-comp idle), which would fight the bridge. Use
RViz (`rviz:=true`) to watch the arm. Adding a viewer would need a mirror-only
passive viewer inside the bridge process, run under `mjpython` on macOS.

## Talking to it from MoveIt / the base station

A `yam_moveit_config` should use `moveit_simple_controller_manager` with:

```yaml
controller_names: [yam_arm_controller, yam_gripper_controller]
yam_arm_controller:
  type: FollowJointTrajectory
  action_ns: follow_joint_trajectory
  default: true
  joints: [yam_joint1, yam_joint2, yam_joint3, yam_joint4, yam_joint5, yam_joint6]
yam_gripper_controller:
  type: GripperCommand
  action_ns: gripper_cmd
  default: true
  joints: [yam_joint7]
```

These are the same names as a real ros2_control `JointTrajectoryController`,
so MoveIt doesn't care whether this bridge or ros2_control is underneath. The
base station can use the same actions and topics over the network. Note that
`ROS_LOCALHOST_ONLY=1` in the pixi env blocks that, so unset it on the robot.
MoveIt joint limits are in `yam_description/config/joint_limits.yaml`.

## Real arm (`backend:=real`)

```bash
sudo ip link set can0 up type can bitrate 1000000
ros2 launch yam_sim_ros sim_bridge.launch.py backend:=real channel:=can0
```

This needs I2RT's `i2rt` package (`pip install -e` of
github.com/i2rt-robotics/i2rt) on the Linux machine wired to the CAN adapter.
`make_robot("real", ...)` calls `i2rt.robots.get_robot.get_yam_robot`. Without
`i2rt` the node exits with yam_sim's install instructions. The bridge
interpolates every trajectory at 200 Hz, so it never sends position jumps, but
the first command still holds the pose the arm is in at startup. Keep the
e-stop at hand the first time. This path has not been tested on hardware.
