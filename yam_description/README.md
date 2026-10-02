# yam_description

ROS 2 Humble description package for the **I2RT YAM** 6-DoF arm the team owns
(Damiao DM4340 motors on joints 1–3, DM4310 on joints 4–6 and the gripper,
driven over CAN by I2RT's `i2rt` Python package).

It provides:

| File | What it is |
| --- | --- |
| `urdf/yam_macro.urdf.xacro` | `yam_arm` xacro macro: arm + selectable gripper + TCP/grasp frames |
| `urdf/yam.urdf.xacro` | Standalone arm on a `world` link (args `gripper`, `prefix`, `ros2_control`, `ros2_control_plugin`) |
| `urdf/yam.ros2_control.xacro` | `yam_ros2_control` macro (position command; position/velocity/effort state) |
| `config/yam_controllers.yaml` | `joint_state_broadcaster`, `yam_arm_controller` (JTC), `yam_gripper_controller` |
| `config/joint_limits.yaml` | MoveIt joint limits for a future `yam_moveit_config` |
| `launch/view_yam.launch.py` | robot_state_publisher + joint sliders + RViz |
| `meshes/yam/`, `meshes/grippers/<name>/` | STL meshes from i2rt |

## Viewing the arm

```bash
colcon build --packages-select yam_description
source install/setup.bash
ros2 launch yam_description view_yam.launch.py                       # linear_4310 gripper
ros2 launch yam_description view_yam.launch.py gripper:=crank_4310
ros2 launch yam_description view_yam.launch.py gripper:=none
ros2 launch yam_description view_yam.launch.py gui:=false            # headless: no RViz/sliders,
                                                                     # joint_state_publisher sends zeros
```

To get the plain URDF: `xacro urdf/yam.urdf.xacro gripper:=linear_4310 > yam.urdf`.

## Names

With the default `prefix:=yam_`:

* links: `yam_base`, `yam_link1` … `yam_link5`, `yam_gripper`, `yam_tip_left`,
  `yam_tip_right`, `yam_tcp`, `yam_grasp`
* joints: `yam_joint1` … `yam_joint6` (revolute), `yam_joint7` (finger,
  prismatic, actuated), `yam_joint8` (other finger, `<mimic>` of joint7),
  plus fixed `yam_base_joint`, `yam_tcp_joint`, `yam_grasp_joint`

With `prefix:=""` the names are exactly i2rt's (`base`, `link1`, `joint1`, …).
`config/*.yaml` and `rviz/yam.rviz` assume the `yam_` prefix.

## Adding the arm to the rover

`rover_description` is not yet on `main` (it lives on the `Gabriel` branch).
Once it is, add this to `rover_description/urdf/rover.urdf.xacro`, replacing
the old 5-DoF arm links/joints (`Arm_Base` … `Arm_Gripper`). Change the
`origin` to where the arm is bolted on the chassis:

```xml
<!-- I2RT YAM arm on the chassis -->
<xacro:include filename="$(find yam_description)/urdf/yam_macro.urdf.xacro"/>
<xacro:include filename="$(find yam_description)/urdf/yam.ros2_control.xacro"/>

<xacro:yam_arm prefix="yam_" parent="Chassis" gripper="linear_4310" fixed_base="true">
  <!-- pose of yam_base in the Chassis frame: placeholder, measure on the rover -->
  <origin xyz="0.25 0 0.12" rpy="0 0 0"/>
</xacro:yam_arm>

<xacro:if value="$(arg use_gazebo)">
  <xacro:yam_ros2_control name="YamArm" prefix="yam_" gripper="linear_4310"
                          plugin="gz_ros2_control/GazeboSimSystem"/>
</xacro:if>
```

Then add `<exec_depend>yam_description</exec_depend>` to
`rover_description/package.xml`, and merge `config/yam_controllers.yaml` into
the rover controller file (the rover's `gz_ros2_control` plugin loads one
parameter file). The rover's `Chassis` link is rotated −90° about Z relative to
`base_link`. Take that into account when choosing the mount `rpy` so that the
arm's +X (its "front" at home) points where you want. The rover snippet above
was checked with `xacro` + `check_urdf` against a stub `base_link`→`Chassis`
tree, not against the real rover URDF.

Macro parameters:

| Param | Default | Meaning |
| --- | --- | --- |
| `prefix` | `yam_` | prepended to every link/joint/material name |
| `parent` | `world` | link the arm base is attached to |
| `*origin` | (required block) | pose of `<prefix>base` in `parent` |
| `gripper` | `linear_4310` | `linear_4310`, `crank_4310` or `none` (anything else is a xacro error) |
| `fixed_base` | `true` | `true`: fixed joint `<prefix>base_joint` from `parent`. `false`: no attachment joint, `<prefix>base` is a root link (`parent`/`origin` ignored), e.g. for a free-floating sim spawn |

## Frame conventions

* `<prefix>base`: arm base frame, +Z up. At home (all joints 0) the arm points
  along +X.
* `<prefix>gripper`: the tool flange (child of joint6). It uses i2rt's
  "flange convention v1", the same convention as ROS optical frames:
  **+Z forward (approach axis), +X right, +Y down**. At home its
  position in `<prefix>base` is (0.110598, 0, 0.173501) and its rotation matrix
  rows are [0 0 1; −1 0 0; 0 −1 0]. This means gripper +Z = base +X,
  gripper +X = base −Y and gripper +Y = base −Z.
* `<prefix>tcp`: i2rt `tcp_site`. It sits at the flange origin.
* `<prefix>grasp`: i2rt `grasp_site`, the midpoint between the two finger tips'
  distal facets. It does not move as the fingers open. It is 0.14465 m along
  flange +Z for linear_4310, (−0.0001, 0.0446, 0.1468) for crank_4310, and at
  the flange origin for `none`.
  Both `tcp` and `grasp` are rotated +90° about the flange Z axis, exactly as
  the MJCF sites are, so their +X points down and +Y points left at home.
  Use `<prefix>gripper` if you want the optical-style axes.
* Fingers: `joint7` / `joint8` range [0, 0.0475] m (linear_4310) or
  [0, 0.039574] m (crank_4310) per finger, with **0 = closed** and max = fully
  open. This matches i2rt `sim_robot.py`, which maps gripper command 0 = closed
  and 1 = open onto that range. Each finger mesh reaches across the centreline,
  so the finger link origins pass each other as the gripper opens. That is
  expected.

## Where the numbers come from

* Arm (`base`, `link1`–`link5`, `joint1`–`joint6`, the `gripper` mount frame,
  and the linear_4310 mount inertial): copied verbatim from
  `i2rt/robot_models/arm/yam/v1/yam.urdf`. That includes origins, axes, joint
  ranges, masses, COMs, inertia tensors and visual offsets. A programmatic
  check found zero difference on every value, and identical forward kinematics
  over 200 random configurations.
* Grippers: the i2rt MJCFs
  (`robot_models/gripper/{linear_4310,crank_4310,no_gripper}/*.xml`),
  re-expressed in the mount frame the same way i2rt's `robots/utils.py`
  assembler merges a gripper into the arm. Finger frames, meshes and sites
  match the assembled i2rt MuJoCo model to < 1e-8. The linear_4310 finger
  frames in i2rt's Onshape `yam.urdf` differ from the MJCF ones by about
  2 mm. The MJCF values are used here so that ROS and the MuJoCo sim agree.
* `gripper:=none` uses a 1 g / 1e-7 kg·m² placeholder on the flange. i2rt
  itself uses 1e-6 kg, which some physics engines reject.
* `<limit effort>`: motor peak torques, 27 N·m (DM4340, joints 1–3) and
  7 N·m (DM4310, joints 4–6). `velocity`: conservative planning values,
  π rad/s (joints 1–3) and 2π rad/s (joints 4–6). The finger limits
  (50 N, 0.1 m/s) are nominal placeholders, not from a datasheet. MoveIt
  acceleration limits in `config/joint_limits.yaml` are also conservative
  guesses.
* Real-arm PD gains used by the i2rt driver: kp = [80, 80, 80, 10, 10, 10],
  kd = [5, 5, 5, 1.5, 1.5, 1.5]. The ros2_control config here only sends
  position setpoints.
* Collision geometry: i2rt ships none, so the visual meshes are reused. The
  arm meshes are the original i2rt STLs. `link2.stl` and `link3.stl` are about
  1.5 MB (30k triangles) each. The gripper meshes are i2rt's 8000-triangle
  versions. The whole `meshes/` folder is about 6 MB. Swap in convex hulls or
  primitives if Gazebo collision checking becomes slow.

## ros2_control

`yam.urdf.xacro ros2_control:=true ros2_control_plugin:=<plugin>` adds the
`<ros2_control>` block. It is not yet wired into Gazebo: there is no
`gz_ros2_control` `<gazebo>` system plugin tag and no launch file for it.
`joint8` is declared with the `mimic`/`multiplier` params, which
`gz_ros2_control` and `mock_components/GenericSystem` use to drive mimic
joints. It was tested with `mock_components/GenericSystem`, `ros2_control_node`
and `config/yam_controllers.yaml`: all three controllers activated, a
`FollowJointTrajectory` goal reached its target, and a gripper command of
0.03 m moved both `yam_joint7` and `yam_joint8`.

## Attribution

Meshes, kinematics and inertial data come from
[i2rt-robotics/i2rt](https://github.com/i2rt-robotics/i2rt) at commit
`120c3c81400171174604e503943f8d1ebc891058`, © I2RT Robotics, MIT licence. A copy
of the licence is in [`LICENSE.i2rt`](LICENSE.i2rt).
