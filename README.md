# HURC Software

Software for the Harvard Undergraduate Robotics Club rover (University Rover
Challenge). The 2025–26 ROS 2 Humble rover stack is being brought in on the
`Gabriel` branch; this branch (`arm_simulation`) adds the simulator and ROS 2
packages for the **I2RT YAM arm** the team owns.

## YAM arm simulation

The arm is a 6-DoF I2RT YAM (Damiao DM4340 motors on joints 1–3, DM4310 on
joints 4–6 and the gripper, 1 Mbit/s CAN bus), normally driven by I2RT's
open-source [`i2rt`](https://github.com/i2rt-robotics/i2rt) Python package.
Everything here uses I2RT's official URDF/MJCF models and meshes (MIT licence,
attribution in each package).

| Directory | What it is | Runs on |
| --- | --- | --- |
| [`arm_sim/`](arm_sim/) | MuJoCo physics simulation of the arm exposing **the same `Robot` API as the real arm's i2rt driver**, plus an interactive viewer, keyboard teleop, slider panel, go-to and trajectory record/replay scripts, and a headless test suite. | Mac (Apple Silicon) and Linux, via `pixi`. No ROS needed. |
| [`yam_description/`](yam_description/) | ROS 2 Humble description package: `yam_arm` xacro macro (arm + selectable gripper + TCP/grasp frames), meshes, `ros2_control` macro and controller config, MoveIt joint limits, RViz view launch. | Any ROS 2 Humble workspace. |
| [`yam_sim_ros/`](yam_sim_ros/) | ROS 2 bridge that runs the MuJoCo sim and exposes it as `/joint_states` + `FollowJointTrajectory` / `GripperCommand` actions under the same controller names as `yam_description/config/yam_controllers.yaml`, so MoveIt or the base station can drive the sim exactly as they will drive the hardware. Ships a pixi manifest with a RoboStack ROS 2 env for Macs. | Mac and Linux, via `pixi`. |

### Quick start (Mac or Linux, no ROS)

```bash
cd arm_sim
pixi install            # one-time, downloads MuJoCo + Python 3.11
pixi run test           # headless test suite
pixi run sim            # interactive MuJoCo viewer (uses mjpython on macOS)
pixi run sliders        # tkinter joint/gripper sliders
pixi run teleop         # terminal keyboard teleop
```

### Quick start with ROS 2 (Mac or Linux)

```bash
cd yam_sim_ros
pixi install            # RoboStack ROS 2 Humble + MuJoCo env (~2.6 GB, mostly hard-linked from the pixi cache)
pixi run build          # colcon build of yam_description + yam_sim_ros at the repo root
pixi run test           # headless integration tests (FollowJointTrajectory, GripperCommand, TF vs MuJoCo FK)
pixi run bridge         # MuJoCo sim behind /joint_states + the yam_arm_controller / yam_gripper_controller actions
pixi run view           # same, plus RViz
```

Then from any ROS 2 shell on the same machine:

```bash
ros2 action send_goal /yam_arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [yam_joint1, yam_joint2, yam_joint3, yam_joint4, yam_joint5, yam_joint6],
                 points: [{positions: [0.5, 1.0, 0.8, -0.3, 0.4, 0.6], time_from_start: {sec: 3}}]}}"
```

### Sim-to-real

Every script in `arm_sim/` takes `--sim` (default) or `--channel can0`. On the
Linux host wired to the arm (with `i2rt` installed and the CAN interface up),
the same command drives the physical arm:

```bash
pixi run go-to -- --pos 0.3 0.0 0.2 --duration 3          # simulation
python -m yam_sim.scripts.go_to --channel can0 --pos 0.3 0.0 0.2 --duration 3   # real arm
```

See [`arm_sim/README.md`](arm_sim/README.md) for the API, the fidelity notes
(motor PD gains, torque limits, the 400 ms CAN watchdog) and the hardware
caveats, and [`yam_sim_ros/README.md`](yam_sim_ros/README.md) for the ROS 2
side.

### Attaching the arm to the rover model

Once `rover_description` is on `main`, include the `yam_arm` macro in
`rover.urdf.xacro` as shown in [`yam_description/README.md`](yam_description/README.md).
The mount pose on the chassis still needs to be measured.
