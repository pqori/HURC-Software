# YAM arm simulator (`arm_sim/`)

This is a MuJoCo physics simulation of the I2RT **YAM** 6-DoF arm, the arm the HURC rover team owns.
It exposes the same control API as the `Robot` that I2RT's
[`i2rt`](https://github.com/i2rt-robotics/i2rt) driver returns from `get_yam_robot(...)`. You can
write and test arm code on a Mac, then run **the same script** on the real arm by adding
`--channel can0`.

| Ready pose | Grasping the test cube |
| --- | --- |
| ![ready](docs/img/yam_ready.png) | ![grasp](docs/img/yam_grasp_cube.png) |

Unlike i2rt's own `SimRobot`, which mostly teleports the joints, this is a dynamic simulation. Each
motor is modelled as a Damiao MIT-mode PD controller with torque limits, gravity compensation, joint
friction, rotor inertia, the 400 ms CAN watchdog, and i2rt's gripper force limiter.

## Setup

You need [pixi](https://pixi.sh) (`~/.pixi/bin/pixi`). Everything comes from conda-forge, plus this
package installed in editable mode. Supported platforms are `osx-arm64` and `linux-64`, with Python 3.11.

```bash
cd arm_sim
pixi install          # one time
pixi run test         # headless test suite
pixi run sim          # interactive 3-D viewer
```

**macOS needs `mjpython`.** MuJoCo's interactive (passive) viewer has to own the Cocoa main thread,
so on macOS it only works under the `mjpython` launcher, which ships with the conda-forge `mujoco`
package. `pixi run sim` already uses `mjpython`. If you start the viewer with plain `python`, it
relaunches itself under `mjpython`. Headless use doesn't need `mjpython`: the API, tests,
`mujoco.Renderer` and the other scripts all run under plain `python`. On Linux, plain `python` works
for everything.

## Tasks

| Command | What it does |
| --- | --- |
| `pixi run sim [--gripper crank_4310] [--objects] [--control]` | Interactive viewer (see controls below) |
| `pixi run teleop` | Terminal keyboard teleop with no GUI (needs a real terminal) |
| `pixi run go-to --joints 0 1 1 -0.3 0 0 --grip 1` | Smooth move to a joint vector (`--deg` for degrees) |
| `pixi run go-to --pos 0.35 0 0.25 [--rpy 0 1.57 0]` | Smooth move to a `grasp_site` pose using IK |
| `pixi run play demo traj.csv` / `pixi run play replay traj.csv --speed 0.5` | Write a demo trajectory / replay a CSV or JSON trajectory |
| `pixi run play record traj.csv --duration 10 --zero-gravity` | Record joint positions (on hardware, move the arm by hand) |
| `pixi run sliders` | Tkinter slider panel (joints in degrees, gripper 0–1, Home/Ready/Open/Close/Float) |
| `pixi run smoke` | Headless tracking test. Prints per-joint RMS/max error and peak torque. Exits 1 on failure |
| `pixi run render` | Offscreen renders of a few poses into `docs/img/` |
| `pixi run test` | pytest suite |

Every script takes `--sim` (the default) or `--channel can0` (real arm), plus `--arm`,
`--gripper linear_4310|crank_4310|no_gripper`, `--zero-gravity`, and `--objects` (sim only, adds a 4 cm cube).

## Viewer controls (`pixi run sim`)

Modes work like i2rt's `control_with_mujoco`. **SPACE** toggles between them:

* **VIS** (the start mode): gravity-comp idle (kp = 0). The arm floats and the green marker follows
  the end effector. In the sim you can push the arm by double-clicking a link and then
  ctrl + right-dragging (forwarding these forces is implemented but has not been tested by hand).
* **CONTROL**: the arm PD-tracks a target and the marker turns red. Double-click the marker, then
  **ctrl + right-drag** to translate it or **ctrl + left-drag** to rotate it. IK follows the marker.

| Key | Action |
| --- | --- |
| `SPACE` | VIS ↔ CONTROL |
| `1`–`6` / `7` | select joint / gripper |
| `=` / `-` (keypad `+`/`-`) | jog the selected joint (gripper ±0.1) |
| ↑ / ↓, ← / →, PgUp / PgDn | nudge the end effector ±x, ±y, ±z (base frame) |
| `End` | toggle the gripper open/closed |
| `Home` | glide to home (all zeros) |
| `;` / `'` | halve / double the jog and nudge step |
| `Backspace` | reset the simulation (sim only) |

On a Mac keyboard, PgUp/PgDn are fn+↑/↓ and Home/End are fn+←/→. Most letter keys are already
MuJoCo's own visualisation toggles, so the bindings above avoid them. The overlay shows the mode, q,
q_des and torque for each joint, the end-effector position, and a help box. Every viewer motion is
rate-limited (`--max-speed`, default 1 rad/s).

Terminal teleop (`pixi run teleop`): `1`–`7` select, `+`/`-` jog, `w/s a/d r/f` move the end
effector in x/y/z, `o/c/g` open/close/toggle the gripper, `h`/`y` go home/ready, `space` toggles
HOLD↔FLOAT, `[`/`]` change the step, `p` prints the observation, and `q` quits.

## Python API

It mirrors the snippet in i2rt's README:

```python
import numpy as np
from yam_sim import make_robot

robot = make_robot("sim", arm="yam", gripper="linear_4310")   # sim on any machine
# robot = make_robot("real", channel="can0")                  # real arm: calls i2rt get_yam_robot()

q = robot.get_joint_pos()          # shape (7,): 6 joints [rad] + gripper [0 = closed, 1 = open]
robot.command_joint_pos(np.array([0, 1.0, 1.0, -0.3, 0, 0, 1.0]))
obs = robot.get_observations()     # joint_pos/vel/eff (6,), gripper_pos/vel/eff (1,)
robot.close()
```

On the real host you can also skip `yam_sim` and call i2rt directly. The calls after construction are the same:

```python
from i2rt.robots.get_robot import get_yam_robot
from i2rt.robots.utils import ArmType, GripperType
robot = get_yam_robot(channel="can0", arm_type=ArmType.YAM, gripper_type=GripperType.LINEAR_4310)
```

`YamSimRobot` implements the i2rt `Robot` protocol (duck-typed, with no `i2rt`/`dm_env` import):
`num_dofs`, `get_joint_pos`, `get_joint_state`, `command_joint_pos`, `command_joint_state`
(`pos`, `vel`, optional `kp`/`kd`), `command_target_vel`, `get_observations`, `get_robot_info`,
`reinit`, `close`. It also has the extra public `MotorChainRobot` methods: `get_motor_torques`,
`zero_torque_mode`, `update_kp_kd`, `enter_gravity_comp_idle`, `move_joints`, `xml_path`.

It follows the same conventions as hardware:

* `zero_gravity_mode=True` (the default) means gravity comp plus a small damping, with no
  position hold. The first `command_joint_pos` turns on PD holding.
* Arm commands are clipped to the MJCF range ±0.15 rad, and the gripper is clipped to [0, 1].
* The observation keys match i2rt's `SimRobot`/`MotorChainRobot`.

There are also sim-only helpers:

* Synchronous, deterministic stepping: `YamSimRobot(start_thread=False)` with `step(n)` and `step_for(seconds)`.
* `reset(q, hold=)`, `stall_bus(seconds)`, `get_applied_torques()`, `get_ee_pose(site)`,
  `get_commanded_pos()`, `set_external_forces()`, `watchdog_tripped`, `model`, `data`, `lock`.

`yam_sim.kinematics.Kinematics` provides `fk(q, frame)` for any site or body, a 6×6
`jacobian(q, frame)`, and `ik(T or xyz, frame, init_q)`. The IK is damped least squares with
joint-limit clamping and random restarts. It needs no `mink`.
`yam_sim.assembly.build_scene(arm, gripper)` returns the full scene MJCF, and `build_scene_file`
writes it to a path.

## Sim to real

Run on the Linux machine (e.g. the Jetson) that is wired to the arm's CAN adapter:

```bash
git clone https://github.com/i2rt-robotics/i2rt.git && cd i2rt && pip install -e .   # once
sudo ip link set can0 up type can bitrate 1000000      # 1 Mbit/s; i2rt: sh scripts/reset_all_can.sh if stuck
ls -l /sys/class/net/can*                              # check the adapter is there
cd HURC-Software/arm_sim && pip install -e .           # or use pixi on linux-64
python -m yam_sim.scripts.go_to --channel can0 --joints 0 1 1 -0.3 0 0
python -m yam_sim.scripts.teleop_keyboard --channel can0
python -m yam_sim.scripts.run_sim --channel can0       # viewer mirrors and drives the real arm
```

Without i2rt, `make_robot("real")` raises `RealBackendUnavailable` with these install instructions.
Things to know on hardware:

* **400 ms watchdog.** From the factory, each motor drops into damping mode if it gets no CAN frame
  for 400 ms. i2rt's driver thread re-sends the last command all the time, so this only trips if
  your process hangs or dies. The arm then sinks slowly instead of holding. The sim models this
  (`stall_bus()` to try it; `watchdog_feed="command"` for a stricter mode). You can turn the
  timeout off with i2rt's `motor_config_tool/set_timeout.py`, but if you do, always start with
  `zero_gravity_mode=False`.
* **Gripper calibration.** linear_4310 and crank_4310 have `gripper_limits: null`. When the real
  robot is created, i2rt calibrates the gripper by driving it to both ends. Start with the gripper
  free to move, preferably closed, and keep fingers and objects out of it. The sim needs no
  calibration and maps [0, 1] straight onto the finger stroke.
* **Big moves are dangerous** with kp = 80 on joints 1–3. The scripts always interpolate (min-jerk or
  a speed limit). Do the same in your own code, and test it in the sim first.
* The real driver stops with an error if a joint ends up more than 0.25 rad outside its MJCF range
  (bad zero offset) or if computed gravity torque exceeds 25 N·m. The sim only warns about the second.

## Fidelity: what is modelled and what isn't

**Modelled**

* Kinematics and inertias come from i2rt's YAM v1 MJCF. The gripper is merged onto the `gripper`
  mount exactly as i2rt's `combine_arm_and_gripper_xml` does it. A test checks FK at home against
  the "Home configuration M" in the model README to 1e-4.
* Each motor runs the MIT-mode law `tau = kp(q_des−q) + kd(dq_des−dq) + tau_ff`. MuJoCo evaluates
  it on every 1 ms physics substep, with kd integrated implicitly. The host-side work runs at 200 Hz
  (`control_freq`): gravity comp `g(q)·gravity_comp_factor`, the optional Coulomb friction
  feedforward, and the frame update.
* Gains come from `yam_v1.yml`: kp = [80, 80, 80, 10, 10, 10], kd = [5, 5, 5, 1.5, 1.5, 1.5],
  grav_comp_kd = [0.1, 0.1, 0.1, 0.3, 0.05, 0.05]. The gripper uses kp 20 and kd 0.5 in motor
  radians. You can override all of them.
* Torque limits per motor: DM4340 on joints 1–3 is limited to 27 N·m (peak; ~9 rated), and DM4310 on
  joints 4–6 and the gripper to 7 N·m (peak; ~3 rated). Use `torque_limit="rated"`, a number, or a
  list to change them. A linear torque–speed derate goes to zero at an approximate no-load speed
  (DM4340 8 rad/s, DM4310 20 rad/s).
* Joint Coulomb friction uses `yam_v1.yml`'s `coulomb_friction` as MuJoCo `frictionloss`.
  Rotor inertia is modelled as joint `armature`.
* The gripper is a DM4310 driving both fingers through a linear transmission
  (6.57 rad = 47.5 mm per finger), with i2rt's force limiter (50 N, as `get_yam_robot` uses).
  The fingertips have box collision pads, so the cube can actually be grasped.
* The 400 ms watchdog with latching damping mode. Optional MIT-mode 16/12-bit feedback quantisation.

**Not modelled, or only approximate**

* **Armature and no-load speed are estimates**, not measurements. Measured rotor inertia, torque-speed
  curves, thermal limits, current/voltage limits, backlash and gearbox compliance are all missing.
* The sim's gravity comp uses factor 1.0 by default because its model is exact, the same choice
  i2rt's `SimRobot` makes. The real arm uses [1.0, 1.1, 1.1, 1.2, 1.0, 1.0] to make up for model
  error. Pass `gravity_comp_factor="hardware"` to use those.
* crank_4310 is treated as a linear transmission. Its real crank kinematics are nonlinear.
* Arm self-collision is off by default (`self_collision=True` turns it on). Collision uses the convex
  hulls of the visual meshes.
* CAN latency, bus errors, motor error codes and auto-recovery are not modelled. Temperatures are
  not reported.
* The teaching handle and other arm variants (yam_pro, yam_ultra, big_yam) are not vendored. To add
  one, copy its model and config into `yam_sim/models/` (see `config.py`).

### Deviations from the i2rt API

* `command_target_vel` is a **no-op with a warning**, matching the real `MotorChainRobot`, which
  never implements it. i2rt's `SimRobot` does implement it, but that would teach habits that fail on
  hardware. Use `command_joint_state({"pos", "vel"})` instead.
* `get_robot_info()["arm_type"]`/`["gripper_type"]` are strings, not i2rt enums. The dict also has
  extra sim keys (`sim`, `torque_limits`, `watchdog`, ...). `gripper_limits` is `[0, 1]`, as in `SimRobot`.
* `command_joint_state` accepts a missing `vel` (the real driver requires it). Commands never modify
  the caller's array (the real driver clips it in place).
* `get_motor_torques()` returns the feedforward torque (gravity + friction comp), as on hardware.
  The total applied torque is `get_applied_torques()` or `joint_eff`.

## Layout

```
arm_sim/
  pixi.toml, pixi.lock, pyproject.toml
  yam_sim/
    assembly.py     scene builder (arm+gripper merge, motors, floor, IK target, finger pads)
    config.py       YAML loading, Damiao motor specs
    motor.py        MIT frame, watchdog, quantisation, gripper force limiter
    robot.py        YamSimRobot
    factory.py      make_robot("sim"|"real"), shared CLI flags
    kinematics.py   FK / Jacobian / IK
    motion.py       min-jerk moves, rate limiting, pacing
    viewer.py       interactive viewer
    scripts/        run_sim, teleop_keyboard, go_to, play_trajectory, slider_panel, smoke_test, render_poses
    models/         vendored i2rt models + NOTICE.md
  tests/            pytest (headless)
  docs/img/         renders
```

## Attribution

The meshes, MJCF/URDF and YAML configs in `yam_sim/models/` are copied unmodified from
[i2rt-robotics/i2rt](https://github.com/i2rt-robotics/i2rt) at commit
`120c3c81400171174604e503943f8d1ebc891058` (MIT License, © I2RT Robotics). Parts of `assembly.py`,
`motor.py` and `robot.py` are adapted from the same commit. See
[`yam_sim/models/NOTICE.md`](yam_sim/models/NOTICE.md) for the full license text.
