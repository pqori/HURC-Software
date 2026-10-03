# rover_sim

Gazebo Harmonic simulation of the rover for the Nav2 obstacle-avoidance demo:
an obstacle world, a simulated ZED 2i (left/right RGB, depth, point cloud, IMU)
bridged to the real zed_wrapper topic names, controller parameters that match
the model's wheel geometry, and one launch file.

```bash
ros2 launch rover_sim sim.launch.py headless:=true rviz:=false
# args: world:=obstacles.sdf (default; a name in rover_sim/worlds, a path, or a
#       Gazebo world such as empty.sdf), headless, rviz, spawn_x/y/z/yaw
#       (default 0, 0, 0.6, 0), controllers_file
```

The launch file does what `rover_description/launch/gazebo.launch.py` does
(Gazebo always server-only on macOS, gz_ros2_control plugin path workaround,
one spawner for joint_state_broadcaster, rover_drive_controller and
rover_arm_controller). On top of that it expands the URDF with
`controllers_file:=config/rover_controllers_sim.yaml` and runs the sensor
bridge from `config/bridge.yaml`. On macOS, open the Gazebo GUI with
`pixi run gz-gui` in a second terminal.

## Sensor topics (ROS side)

| Topic | Type | frame_id | Rate / size |
|---|---|---|---|
| /Front_Zed/zed_node/left/image_rect_color | sensor_msgs/Image (rgb8) | Front_Zed_left_camera_optical_frame | 15 Hz, 640x360 |
| /Front_Zed/zed_node/left/camera_info | sensor_msgs/CameraInfo | Front_Zed_left_camera_optical_frame | 15 Hz |
| /Front_Zed/zed_node/right/image_rect_color | sensor_msgs/Image (rgb8) | Front_Zed_right_camera_optical_frame | 15 Hz, 640x360, 0.12 m baseline |
| /Front_Zed/zed_node/right/camera_info | sensor_msgs/CameraInfo | Front_Zed_right_camera_optical_frame | 15 Hz |
| /Front_Zed/zed_node/depth/depth_registered | sensor_msgs/Image (32FC1, m) | Front_Zed_left_camera_frame | 10 Hz, 320x180 |
| /Front_Zed/zed_node/depth/camera_info | sensor_msgs/CameraInfo | Front_Zed_left_camera_frame | 10 Hz |
| /Front_Zed/zed_node/point_cloud/cloud_registered | sensor_msgs/PointCloud2 (x,y,z,rgb) | Front_Zed_left_camera_frame | 10 Hz, 320x180 |
| /Front_Zed/zed_node/imu/data | sensor_msgs/Imu | Front_Zed_imu_link | 100 Hz |
| /clock | rosgraph_msgs/Clock | | |

All cameras: horizontal FOV 1.57 rad, clip 0.3 to 20 m. The depth image and
the point cloud are stamped with the x-forward `Front_Zed_left_camera_frame`,
not the optical frame, because Gazebo Harmonic emits the rgbd point cloud in
the sensor's body convention (x forward, y left, z up). Depth and the cloud run
at half resolution: a 640x360 cloud is 5.5 MB per message, and Fast DDS on a
laptop delivered it at only about 4.5 Hz. The sensors are defined in
`rover_description/urdf/rover.urdf.xacro` (inside `xacro:if use_gazebo`).
`Front_Zed_imu_link` exists only in the sim URDF; on the real rover,
zed_wrapper publishes that frame itself.

Note: `ros2 topic hz` (Python) undercounts the 640x360 images and the cloud
(under 2 Hz) because deserializing large messages in Python is slow. C++
subscribers such as rosbag2 or Nav2 receive the full rates.

## Controller overlay

`config/rover_controllers_sim.yaml` is a copy of
`rover_description/config/rover_controllers.yaml` with the wheel geometry
measured from the model:

- `wheel_radius: 0.096`: the wheel STL meshes span +/-0.096 m.
- `wheel_separation: 0.75`: the wheel tread centres sit at y = +/-0.3748 m
  in base_link.

The hardware yaml (0.2 / 0.8) is unchanged.

## World: worlds/obstacles.sdf

The world has a 30 x 30 m ground plane, a sun, and the Physics, UserCommands,
SceneBroadcaster, Sensors (ogre2), Contact and Imu systems. Start A = (0, 0)
and goal B = (8, 0, yaw 0). The straight line A->B is blocked by box_line_1
and cyl_line_2. The north gap between box_line_1 and wall_north is only about
1 m wide. The clear detour runs south, (0,0) -> (1.5,-2.2) -> (6.5,-2.2) ->
(8,0), and keeps at least 1.57 m from every obstacle edge. Every obstacle edge
is at least 2.0 m from A and from B. All obstacles are static and have both
collision and visual geometry. Heights are full heights; boxes sit on the
ground.

| name | shape (m) | height (m) | center (x, y) | yaw (rad) | edge dist to A | edge dist to B |
|---|---|---|---|---|---|---|
| box_line_1 | box 1 x 1.2 | 0.8 | (3, 0.3) | 0 | 2.50 | 4.50 |
| cyl_line_2 | cylinder r 0.5 | 1 | (5.5, 0) | 0 | 5.00 | 2.00 |
| wall_north | box 3 x 0.2 | 0.8 | (4.25, 2) | 0 | 3.34 | 2.94 |
| box_sw | box 0.8 x 0.8 | 0.6 | (2, -4.3) | 0 | 4.22 | 6.82 |
| cyl_se | cylinder r 0.3 | 0.7 | (6.5, -4.2) | 0 | 7.44 | 4.16 |
| box_ne | box 1 x 1 | 1.2 | (7, 3) | 0.5 | 7.08 | 2.61 |
| cyl_nw | cylinder r 0.4 | 0.9 | (1, 3) | 0 | 2.76 | 7.22 |
| box_far_se | box 0.6 x 0.6 | 0.5 | (9.5, -2.5) | 0.3 | 9.41 | 2.56 |
| box_behind | box 0.8 x 0.8 | 1 | (-2.5, 1.5) | 0 | 2.37 | 10.16 |
| cyl_far_ne | cylinder r 0.25 | 0.6 | (10, 2) | 0 | 9.95 | 2.58 |

The rover spawns at A with z = 0.6 and settles at z = 0.529 (base_link).
