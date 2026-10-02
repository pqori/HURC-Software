### Development Setup
When you first clone the repo, you need to build the base docker image. This will install ROS, the ZED SDK, the Teensy build system, and all of the current package dependencies:
```bash
make image
```
This command takes a while, so you should only re-run it whenever new dependencies or setup commands are added to the project that would be annoying to re-run often. You can also run `make update` from inside the container to install/update any ros package dependencies without rebuilding the whole image (but these will not persist across creating new containers).

Once the image is built, you can run the following command to open a shell in the development environment:
```bash
make ros
```
Any changes made to the container's filesystem will be saved until you run either `make stop` or `make restart`.

Once the container is running, you can access any GUI apps by opening http://localhost:8080/vnc.html in a browser.

### Building
Inside the container or on the jetson, you can use the following commands to build and use ros packages:
```bash
make build # or build-package-name
source install/local_setup.bash
```

### Running the simulator natively on macOS (pixi)
The Gazebo simulation can also run without Docker, natively on an Apple Silicon Mac (and on x86-64 Linux), using [pixi](https://pixi.sh) and [RoboStack](https://robostack.github.io) builds of ROS 2 Humble. Install pixi once (`curl -fsSL https://pixi.sh/install.sh | sh`), then from the repo root:
```bash
pixi install          # one-time; downloads ROS 2 Humble, Gazebo Fortress, Nav2, MoveIt (several GB) into .pixi/
pixi run build        # builds rover_description, rover_navigation, rover_moveit_config
pixi run sim          # Gazebo server + rviz2 + ros2_control controllers
pixi run sim-headless # same, but no GUI at all (good for testing/CI)
pixi run teleop       # in a second terminal: drive with the keyboard
pixi run clean        # remove build/ install/ log/
```
Run any other ROS command inside the environment with `pixi shell` followed by `source install/setup.bash`. The launch file also takes `rviz:=false`, `headless:=true` and `world:=<sdf>` arguments, e.g. `pixi run sim rviz:=false`.

Useful topics while the sim runs: drive with `geometry_msgs/Twist` on `/rover_drive_controller/cmd_vel_unstamped` (wheel odometry on `/rover_drive_controller/odom`), and move the arm with `std_msgs/Float64MultiArray` on `/rover_arm_controller/commands` (joint order: Arm_Base_Rotate, Arm_Shoulder, Arm_Elbow_Tilt, Arm_Elbow_Rotate, Arm_Wrist_Tilt, Arm_Wrist_Rotate).

What works natively on a Mac: Gazebo (Fortress) physics with ros2_control (drive and arm controllers), rviz2, keyboard teleop, and the Nav2, MoveIt 2 and robot_localization (EKF) packages are installed in the environment.

What does not, so use Docker (or the Jetson) for these: the ZED SDK and `zed_*` packages (they need CUDA), RTAB-Map (not packaged for macOS), and the Teensy/Arduino firmware toolchain. `pixi run build` therefore only builds the three simulation packages; the Docker build is unchanged and still builds everything.

Gazebo GUI on macOS: Gazebo Fortress's own GUI cannot run on macOS (the `ign gazebo` front end only accepts `-s` there, and its 3D view needs the Cocoa main thread; see [gz-sim#44](https://github.com/gazebosim/gz-sim/issues/44)). On macOS the launch file therefore always starts the Gazebo server alone (`-s`) and you watch the rover in rviz2. On Linux, `pixi run sim` opens the normal Gazebo GUI as before.
