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
The Gazebo simulation can also run without Docker, natively on an Apple Silicon Mac (and on x86-64 Linux), using [pixi](https://pixi.sh) and [RoboStack](https://robostack.github.io) builds of ROS 2 Jazzy with Gazebo Harmonic. Install pixi once (`curl -fsSL https://pixi.sh/install.sh | sh`), then from the repo root:
```bash
pixi install          # one-time; downloads ROS 2 Jazzy, Gazebo Harmonic, Nav2, MoveIt (several GB) into .pixi/
pixi run build        # builds rover_description, rover_navigation, rover_moveit_config
pixi run sim          # Gazebo server + rviz2 + ros2_control controllers
pixi run sim-headless # same, but no GUI at all (good for testing/CI)
pixi run gz-gui       # in a second terminal: the Gazebo GUI for the running sim
pixi run teleop       # in a second terminal: drive with the keyboard
pixi run clean        # remove build/ install/ log/
```
Run any other ROS command inside the environment with `pixi shell` followed by `source install/setup.bash`. The launch file also takes `rviz:=false`, `headless:=true` and `world:=<sdf>` arguments, e.g. `pixi run sim rviz:=false`. The environment keeps ROS 2 (`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`) and Gazebo transport (`GZ_IP=127.0.0.1`) on this machine.

Useful topics while the sim runs: drive with `geometry_msgs/TwistStamped` on `/rover_drive_controller/cmd_vel` (wheel odometry on `/rover_drive_controller/odom`), and move the arm with `std_msgs/Float64MultiArray` on `/rover_arm_controller/commands` (joint order: Arm_Base_Rotate, Arm_Shoulder, Arm_Elbow_Tilt, Arm_Elbow_Rotate, Arm_Wrist_Tilt, Arm_Wrist_Rotate). For example:
```bash
ros2 topic pub -r 10 /rover_drive_controller/cmd_vel geometry_msgs/msg/TwistStamped "{header: {frame_id: base_link}, twist: {linear: {x: 0.3}}}"
```
On Jazzy the drive controller no longer accepts plain `Twist` (the old `cmd_vel_unstamped` topic is gone); `pixi run teleop` already publishes stamped messages. A zero header stamp is fine: the controller fills in the current time.

What works natively on a Mac: Gazebo (Harmonic) physics with ros2_control (drive and arm controllers), rviz2, the Gazebo GUI as a separate window, keyboard teleop, and the Nav2, MoveIt 2 and robot_localization (EKF) packages are installed in the environment.

What does not, so use Docker (or the Jetson) for these: the ZED SDK and `zed_*` packages (they need CUDA), RTAB-Map (not packaged for macOS), and the Teensy/Arduino firmware toolchain. `pixi run build` therefore only builds the three simulation packages; the Docker build still builds everything.

Gazebo GUI on macOS: Gazebo cannot run its server and GUI in one process on macOS (the GUI needs the Cocoa main thread; see [gz-sim#44](https://github.com/gazebosim/gz-sim/issues/44)), so on macOS the launch file always starts the Gazebo server alone (`-s`) and rviz2 is the default viewer. To get the Gazebo GUI too, leave `pixi run sim` running and run `pixi run gz-gui` (which runs `gz sim -g` with the rover's mesh path set) in a second terminal; it connects to the running server. On Linux, `pixi run sim` opens the normal Gazebo GUI directly.
