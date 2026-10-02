# HURC-Software: sim-readiness audit (branch `Gabriel`, commit f44c4aa)

Scope: read-only. Line numbers for `rover_description/*` are from `HEAD`, because another agent is editing `gazebo.launch.py` and `rover.urdf.xacro` in the working tree right now. Install, log and build directories, and `* copy.py` files, were skipped. Param files were checked with Humble's rcl parser (`rclpy.init(['--ros-args','--params-file',f])` from the repo's `.pixi` env).

---

## 0. Problems that stop the stack on real hardware too (fix these first)

| # | Problem | Evidence |
|---|---|---|
| B1 | `localization.yaml` is missing commas, so rcl throws *"Sequence should be of same type ... line 23"*. **ekf_local_node, navsat_transform_node and ekf_global_node all fail to start.** | rover_navigation/config/localization.yaml:23-24 |
| B2 | These param files have no `<node>: ros__parameters:` wrapper. rcl throws *"Cannot have a value before ros__parameters"*, so a regular `Node` crashes, and a composable node (the ZED) quietly ignores the file. | pointcloud_tools/config/pointcloud_params.yaml:1 (depth_to_scan and grid_builder crash), detection_pose_estimator/config/estimator_params.yaml:1 (pose_estimator crashes), zed2i_launch/params/zed2i.yaml:1 (whole ZED override ignored), slam_launch/config/{navigation_integration,hybrid_mapping,rtabmap_params}.yaml, object_detection/config/{detector_params,zed_detection_params}.yaml |
| B3 | In `rover_navigation`, ekf_global has no output remap, so both EKFs publish `/odometry/filtered`, and ekf_global also consumes it as odom0. That is a feedback loop. | location.launch.py:63-74, localization.yaml:59 |
| B4 | In `loc_fusion`, ekf_global remaps `odometry/filtered` to `/odometry/global`. ROS 2 matches remaps on the fully-qualified name, so its odom0 subscription (`/odometry/filtered`) is redirected too, and it subscribes to its own output. | loc_fusion.launch.py:55-57, ekf.yaml:39 |
| B5 | navsat_transform's remap keys don't match what the node uses. The Humble node subscribes `imu`, `gps/fix` and `odometry/filtered`, and publishes `odometry/gps` and `gps/filtered` (confirmed from the strings in `librl_lib.dylib`). Remaps `imu/data` (location.launch.py:58), `odom` and `filtered` (loc_fusion.launch.py:46-47) do nothing, so loc_fusion's ekf_global `odom1: /odom/gps` (ekf.yaml:46) never receives anything. | as cited |

---

## 1. Topic contract

The ZED wrapper builds its topics as `~/` + suffix (zed_camera_component_main.cpp:1891-1926, zed_camera_component_video_depth.cpp:63-98). Camera info comes from image_transport camera publishers as `<dir>/camera_info`.

### 1a. `/Front_Zed/zed_node/*`: consumers only. **No launch file in the repo produces this.**
| topic | type | published by | consumed by | file:line |
|---|---|---|---|---|
| /Front_Zed/zed_node/odom | nav_msgs/Odometry | ZED, only if `camera_name:=Front_Zed` | ekf_local_node odom0 (relative name) | rover_navigation/config/localization.yaml:13 |
| /Front_Zed/zed_node/imu/data | sensor_msgs/Imu | same | ekf_local_node imu0; navsat (remap does nothing, B5) | localization.yaml:27; location.launch.py:58 |
| /Front_Zed/zed_node/point_cloud/cloud_registered | sensor_msgs/PointCloud2 | same | Nav2 local_costmap obstacle_layer | rover_navigation/config/navigation.yaml:57 |

### 1b. `/zed2i/zed2i_camera/*`: **what the repo's ZED launch actually publishes** (zed2i_driver.launch.py:109-111)
| topic | type | published by | consumed by | file:line |
|---|---|---|---|---|
| …/left/image_rect_color, …/left/camera_info, …/depth/depth_registered | Image / CameraInfo / Image | ZED (zed2i_launch) | aruco `standalone_detector_ros.py` (script defaults) | aruco_detector/scripts/standalone_detector_ros.py:122-124 |
| …/left/image_rect_color | Image | ZED | ONNX `standalone_continuous_object_detector_ros.py` | object_detection/scripts/standalone_continuous_object_detector_ros.py:207 |
| …/point_cloud/cloud_registered | PointCloud2 | ZED | depth_to_scan, grid_builder (both crash, B2; node defaults are the same topic / `/rtabmap/cloud_map`) | pointcloud_params.yaml:9,18; depth_to_scan.py:28; grid_builder.py:25 |
| …/imu/data, …/odom, …/obj_det/objects | Imu / Odometry / zed_msgs/ObjectsStamped | ZED | **nobody** | n/a |

### 1c. `/zed2i/*` (no `zed2i_camera` segment): **no publisher exists**
The docstring's "REMAPPED TOPICS (for compatibility)" (zed2i_driver.launch.py:51-68) were never implemented. The launch has no remappings (:103-114).
| topic | type | consumed by | file:line |
|---|---|---|---|
| /zed2i/left/image_rect_color | Image | aruco_detector; ONNX object_detection; rtabmap; perception_guardian; calibration_validator; health monitor; system_status | aruco config/detector_params.yaml:16, node.py:164; object_detection.launch.py:28, node.py:38, config/detector_params.yaml:11; slam.launch.py:75; perception_guardian.py:166; calibration_validator.py:171; perception_health_monitor.py:63; system_status.py:129 |
| /zed2i/left/camera_info | CameraInfo | aruco; rtabmap; calibration_validator | detector_params.yaml:17, node.py:165; slam.launch.py:76; calibration_validator.py:173 |
| /zed2i/right/image_rect_color, /zed2i/right/camera_info | Image / CameraInfo | calibration_validator | calibration_validator.py:172,174 |
| /zed2i/depth/depth_registered | Image | aruco (ApproxTimeSync with rgb); rtabmap | detector_params.yaml:18, node.py:166; slam.launch.py:77 |
| /zed2i/depth/camera_info | CameraInfo | rtabmap | slam.launch.py:78 |
| /zed2i/point_cloud/cloud_registered | PointCloud2 | pose_estimator; sensor_costmap_publisher | estimator_params.yaml:2, node.py:94; navigation_integration.yaml:34, sensor_costmap_publisher.py:22 |
| /zed2i/imu/data | Imu | loc_fusion ekf_local imu0; loc_fusion navsat (`imu` remap is correct here); guardian; calib; status | loc_fusion/config/ekf.yaml:20; loc_fusion.launch.py:44; perception_guardian.py:167; calibration_validator.py:175; system_status.py:130 |
| /zed2i/odom | Odometry | loc_fusion ekf_local odom0 | ekf.yaml:13 |
| /zed2i/obj_det/objects | zed_msgs/ObjectsStamped | zed_object_bridge | zed_integrated_detection.launch.py:88; zed_bridge.py:33 |

### 1d. `/zed/zed_node/*`: produced by `zed_integrated_detection.launch.py`, which doesn't pass `camera_name` (:66-78, default `zed` at zed_camera.launch.py:356). **Nothing consumes it.**

### 1e. Non-camera topics
| topic | type | published by | consumed by | file:line |
|---|---|---|---|---|
| /gps/fix | NavSatFix | ublox `gnss_driver` | navsat (both stacks), gnss_health_monitor, gnss_validator, zed_gnss_fusion, map_server | gnss.launch.py:36; location.launch.py:28,59; loc_fusion.launch.py:45; gnss_health_monitor.py:30; gnss_validator.py:22; fusion_params.yaml:8; map_server.py:27 |
| /gnss/fix | NavSatFix | **nobody** (wrong name) | monitoring/recording configs | monitoring_config.yaml:17; recording_config.yaml:23,64 |
| /rover_drive_controller/odom | Odometry | diff_drive_controller | ekf_local_node odom1 | rover_controllers.yaml:8-9; localization.yaml:20 |
| /odometry/filtered | Odometry | ekf_local (both stacks) **and** rover_navigation ekf_global (B3) | ekf_global odom0, navsat (default name), velocity_smoother `odom_topic`, rtabmap, guardian, localization_switcher, navigation_recovery | localization.yaml:59; ekf.yaml:39; navigation.yaml:131; slam.launch.py:79; perception_guardian.py:168; localization_switcher.py:53; navigation_recovery.py:51 |
| /odometry/gps | Odometry | navsat_transform (default name) | rover_navigation ekf_global odom1 | localization.yaml:66 |
| /odom/gps | Odometry | **nobody** (B5) | loc_fusion ekf_global odom1 | ekf.yaml:46 |
| /cmd_vel | Twist | controller_server (remap), behavior_server | velocity_smoother (default `cmd_vel`) | navigation.launch.py:22 |
| /rover_drive_controller/cmd_vel_unstamped | Twist | velocity_smoother `cmd_vel_smoothed` remap | diff_drive_controller (`use_stamped_vel: false`) | navigation.launch.py:55; rover_controllers.yaml:30 |
| /cmd_vel_fallback | Twist | navigation_recovery | nobody | navigation_recovery.py:60 |
| /scan | LaserScan | depth_to_scan **and** sensor_costmap_publisher (two publishers) | **nothing in Nav2** (the only obstacle source is the point cloud, navigation.yaml:54) | pointcloud_params.yaml:10; depth_to_scan.py:29; sensor_costmap_publisher.py:23 |
| /local_map | OccupancyGrid | grid_builder | nobody (recording only) | pointcloud_params.yaml:19; grid_builder.py:26; recording_config.yaml:31 |
| /aruco_detections | vision_msgs/Detection3DArray | aruco_detector (relative `aruco_detections`) | sensor_fusion (OK); **SearchPattern subscribes as `visualization_msgs/Marker`** (type mismatch); monitor_detections listens on `/aruco/detections` (name mismatch) | node.py:119-122; sensor_fusion.py:53,83; rover_navigation/src/SearchPattern.py:75; aruco scripts/monitor_detections.py:25 |
| /detected_objects | vision_msgs/Detection2DArray | ONNX object_detection | pose_estimator, select_object_service, sensor_fusion | node.py:39,85; pose_estimator node.py:92; select_object_service.py:43 |

---

## 2. What the real ZED driver publishes under each launch path

Rule (zed_camera.launch.py:149-160, 315-321, 370-373): `namespace = camera_name` unless a namespace is given; node = `node_name` (default `zed_node`). The topic prefix is `/<camera_name>/<node_name>/`. The frame prefix is `<camera_name>_` (zed_camera_component_main.cpp:1311-1312, 1821-1836).

| launch path | camera_name / model / node_name | topic prefix | frame prefix | silently unsubscribed consumers |
|---|---|---|---|---|
| P1 `zed2i_launch/zed2i_driver.launch.py` (also pulled in by loc_fusion.launch.py:28, slam.launch.py:37, aruco_with_zed.launch.py:34, perception_complete → slam) | zed2i / zed2i / zed2i_camera (:109-111) | `/zed2i/zed2i_camera/` | `zed2i_` | everything in 1a (Nav2 costmap, rover_navigation EKF) and everything in 1c (aruco, ONNX, rtabmap, pose_estimator, loc_fusion EKF + navsat, monitors) |
| P2 `object_detection/zed_integrated_detection.launch.py` (perception_complete.launch.py:149-153, on by default via `use_zed_detection`, :84) | zed (default) / zed2i / zed_node | `/zed/zed_node/` | `zed_` | everything in 1a, 1b and 1c, plus zed_bridge (`/zed2i/obj_det/objects`). When P1 is also running, **two drivers open the same camera**. Its `config_common_path` / `config_od_path` / `object_detection.*` args (:68-77) aren't declared by zed_camera.launch.py, so they are ignored. |
| P3 bare `ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i` | zed / zed2i / zed_node | `/zed/zed_node/` | `zed_` | same as P2 |
| P4 `camera_name:=Front_Zed` (needed by 1a, but no file sets it) | Front_Zed / zed2i / zed_node | `/Front_Zed/zed_node/` | `Front_Zed_` | everything in 1b and 1c |

Other facts about P1:
- The override YAML is ignored (B2). Even if it loaded, `pos_tracking.publish_tf: false` (zed2i.yaml:16) would lose to the launch-argument dict, which is appended last (zed_camera.launch.py:284-303, `publish_tf` default `true` at :403-407).
- So the driver **publishes `odom→zed2i_camera_link` and `map→odom`** (publish_map_tf default true, :408-412), and a second robot_state_publisher publishes the `zed2i_*` URDF (publish_urdf default true, :234-248, 398-402).

**Recommendation: use `/Front_Zed/zed_node/` (camera_name `Front_Zed`, default node_name `zed_node`).** The wrapper ties frame IDs to `camera_name`, and the rover URDF instantiates the macro as `Front_Zed` on `base_link` (rover.urdf.xacro:629-639). So `Front_Zed` is the only name whose image, cloud and IMU `frame_id`s connect to `base_link` through the rover's own TF tree, without a second URDF publisher. The autonomy stack that must run unchanged in sim (Nav2 costmap and the rover_navigation EKF) already uses this prefix. It also keeps the wrapper's documented default node name, so the launch changes only one argument. Gazebo sensors and the bridge can publish to exactly these names. Launch the driver with `publish_urdf:=false publish_tf:=false publish_map_tf:=false publish_imu_tf:=true` so that robot_state_publisher and the EKFs own TF.

**Edits to adopt it** (replace `/zed2i/…` or `/zed2i/zed2i_camera/…` with `/Front_Zed/zed_node/…`, and `zed2i_*` frames with `Front_Zed_*`):
- Driver: zed2i_launch/launch/zed2i_driver.launch.py:110-111 (camera_name `Front_Zed`, drop node_name, add the 4 publish_* args); zed2i_launch/params/zed2i.yaml:1 (wrap in `/**: ros__parameters:`); docstring :12-84.
- object_detection: zed_integrated_detection.launch.py:66 (add `camera_name`, or remove the second driver), :88; zed_bridge.py:33; config/detector_params.yaml:11; launch/object_detection.launch.py:28; object_detection/node.py:38; zed_detector.py:152 (frame); scripts/standalone_continuous_object_detector_ros.py:207.
- aruco_detector: config/detector_params.yaml:16-18, 24 (frame → `Front_Zed_left_camera_optical_frame`); aruco_detector/node.py:164-166, 169; zed_interface.py:103; scripts/standalone_detector_ros.py:122-124.
- detection_pose_estimator: config/estimator_params.yaml:2; node.py:94.
- pointcloud_tools: config/pointcloud_params.yaml:9, 18; depth_to_scan.py:28.
- loc_fusion: config/ekf.yaml:13, 20; launch/loc_fusion.launch.py:44.
- slam_launch: launch/slam.launch.py:75-78; config/navigation_integration.yaml:9, 34, 86; config/monitoring_config.yaml:7, 12; config/recording_config.yaml:15-22, 57, 62-63; navigation/sensor_costmap_publisher.py:22; navigation/frame_coordinator.py:25; monitoring/perception_guardian.py:166-167; monitoring/calibration_validator.py:171-175; monitoring/perception_health_monitor.py:63; monitoring/system_status.py:129-130.
- rover_navigation: localization.yaml:13, 27 (add a leading `/`); location.launch.py:58 (remap key `imu/data` → `imu`).

---

## 3. Sim time and lifecycle (zed/ and uros/ excluded)

| file:line | setting | exposed as launch arg? |
|---|---|---|
| rover_navigation/launch/location.launch.py:43 | ekf_local_node `use_sim_time: False` | No (file declares no args) |
| rover_navigation/launch/location.launch.py:56 | navsat_transform_node `use_sim_time: False` | No |
| rover_navigation/launch/location.launch.py:73 | ekf_global_node `use_sim_time: False` | No |
| rover_navigation/launch/navigation.launch.py:64 | lifecycle_manager_navigation `use_sim_time: False` | No (file declares no args) |
| rover_navigation/launch/navigation.launch.py:65 | `autostart: True` | No |
| rover_navigation/launch/navigation.launch.py:66-72 | `node_names`: planner_server, controller_server, bt_navigator, behavior_server, velocity_smoother | No |
| rover_navigation/config/navigation.yaml:106 | planner_server `use_sim_time: false` | No. The other Nav2 servers (navigation.launch.py:21,30,38,46,54) get no value and use the default, false. |
| rover_navigation/config/navigation.yaml:121 | bt_navigator `# use_sim_time: false` (commented out) | n/a |
| rover_description/launch/gazebo.launch.py:45 (HEAD) | ros_gz_bridge `use_sim_time: True` | No (correct for sim) |
| rover_description/launch/gazebo.launch.py:67 (HEAD) | robot_state_publisher `use_sim_time: True` | No |
| rover_description/launch/gazebo.launch.py:112 (HEAD) | rviz2 `use_sim_time: True` | No |
| rover_description/launch/moveit_rviz.launch.py:35 | `# {"use_sim_time": True}` (commented out) | n/a |
| perception/slam_launch/launch/perception_complete.launch.py:514 | `'use_sim_time': 'false'`, passed to `rover_description/launch/display.launch.py` (:511), **a file that doesn't exist** | n/a |

- **9 active hard-codes** (6 false, 3 true) plus 2 commented out. Every perception launch (aruco, object_detection, pose_estimator, pointcloud_tools, loc_fusion, gnss, slam, zed_gps_integration) sets nothing and runs on wall time.
- Vendored ZED: `use_sim_time` is a declared argument (zed_camera.launch.py:444-448, default false), but zed2i_driver.launch.py doesn't forward it. The ZED robot_state_publisher takes its `use_sim_time` from `publish_svo_clock` (:244).
- Lifecycle: the only lifecycle manager is the Nav2 one above. aruco imports `LifecycleNode` but the class is a plain `Node` (aruco_detector/node.py:16,42).
- hw_control.launch.py has no `use_sim_time`. In Gazebo, gz_ros2_control's controller manager sets it on its own.

---

## 4. ZED SDK / CUDA coupling

**A. Needs the ZED SDK (`pyzed.sl`). Cannot run in Gazebo (16 files):**
| file:line | what it does |
|---|---|
| object_detection/object_detection/zed_detector.py:15,60 | `zed_object_detector` ROS node. Opens the camera with `sl.Camera()`, runs ZED native 3D object detection, publishes `/zed_detections_3d` (:38,102) |
| zed_gps_integration/zed_gps_integration/zed_gnss_fusion.py:17,99,151 | `zed_gnss_fusion` ROS node. SDK VIO plus `sl.Fusion` GNSS fusion. Opens the camera itself (it fights the wrapper for it). Publishes `~/fused_odom`, `~/geo_pose` and TF `map→base_link` (:315-316,367-374) |
| aruco_detector/aruco_detector/zed_interface.py:41,46 | `ZEDSDKInterface` class (lazy import). The ROS node uses only `ROSBridgeAdaptor` from this file (node.py:30), so the node itself is fine |
| aruco_detector/scripts/standalone_detector_3d.py:12,208 | standalone SDK ArUco with 3D pose |
| loc_fusion/scripts/standalone_locfusion.py:31,236 | standalone SDK localization demo |
| object_detection/scripts/standalone_object_detector.py:39 and standalone_continuous_object_detector.py:39 | standalone SDK grab plus ONNX (CUDA→CPU at :548 / :550) |
| {aruco_detector,loc_fusion,object_detection}/scripts/cv_viewer/{tracking_viewer.py:5,utils.py:3}, ogl_viewer/viewer.py:14 | SDK visualisers (9 files) |

Also blocked in sim, though they don't import the SDK: zed_bridge.py:15,33 (needs `zed_msgs/ObjectsStamped`, which only the ZED OD module produces); slam.launch.py:53 and perception_complete.launch.py:468,488 (include package `zed_integration`, which **doesn't exist in the repo**); slam.launch.py:69-70 (runs `rtabmap.launch.py` as a Node executable, which is invalid). `$(find zed_wrapper)` in the URDF (rover.urdf.xacro:628) needs zed_wrapper, which build-depends on `zed_components`, which needs the SDK and CUDA (zed_wrapper/package.xml:14,18). Its meshes come from `zed_msgs` (zed_macro.urdf.xacro:121).

**B. ROS topics only. Runs in Gazebo:**
- aruco_detector/aruco_detector/{node.py, core.py}, scripts/{standalone_detector_ros.py, monitor_detections.py, generate_markers.py, process.py}; cam_cal.py uses `cv2.VideoCapture(0)` (:5)
- object_detection/object_detection/node.py: onnxruntime tries `CUDAExecutionProvider` and falls back to CPU (:22,66-72), so it runs on CPU. Same for scripts/standalone_continuous_object_detector_ros.py:13,103. Also select_object_service.py, scripts/monitor_detections.py.
- detection_pose_estimator/node.py; pointcloud_tools/{depth_to_scan,grid_builder}.py; gnss_launch/* ; zed_gps_integration/map_server.py; all of slam_launch/slam_launch/**; rover_navigation/src/SearchPattern.py

---

## 5. Frames

**Camera frames.** The macro is instantiated as `name="Front_Zed" model="zed2i"`, joint `Front_Zed_joint`, parent `base_link`, origin `xyz="0.55 0 -0.18" rpy="0 0 0"` (rover.urdf.xacro:629-639, HEAD). It sits **inside `<xacro:unless value="$(arg use_gazebo)">` (:627-640), so in Gazebo there are no camera frames at all.**
- From the URDF (zed_macro.urdf.xacro:113-199): `Front_Zed_camera_link` → `Front_Zed_camera_center` → `Front_Zed_left_camera_frame` → `Front_Zed_left_camera_optical_frame`, plus the matching `Front_Zed_right_camera_frame` → `Front_Zed_right_camera_optical_frame`. The mag, baro and temp links only exist for `model=='zed2'` (:202-203), so not for zed2i.
- From the driver: IMU msgs use `Front_Zed_imu_link` (zed_camera_component_main.cpp:1828, 4712). That frame is **not in the URDF**. The driver only broadcasts it (`left_camera_frame→imu_link`, :4033,4045-4046) when `publish_imu_tf` is true, and the default is false (zed_camera.launch.py:413-417).
- Driver message frames: cloud and depth use `*_left_camera_frame` / `*_left_camera_optical_frame` (:1833-1836); odom header is `odom`, child `*_camera_link` (:5325-5326).
- Under P1 every one of these has the prefix `zed2i_` instead.

**Frames the rest of the stack references:**
| consumer | frames | file:line |
|---|---|---|
| rover_navigation EKFs | map / odom / base_link; local world=odom, global world=map; `publish_tf: true` on both; two_d_mode true | localization.yaml:5-11, 51-57 |
| loc_fusion EKFs | same names, `publish_tf: true` on both, two_d_mode **false** | ekf.yaml:5-11, 32-37 |
| navsat_transform | GPS `frame_id` is `gps_link` (gnss.launch.py:25) or `base_link` (location.launch.py:17); `broadcast_utm_transform: false` | navsat_transform.yaml:9 |
| frame_coordinator | publishes **static** `base_link→zed2i_left_camera_frame` (0.2,0,0.3) and `base_link→gnss_link` | frame_coordinator.py:71-96; navigation_integration.yaml:6-10 |
| pointcloud_tools | depth_to_scan relabels the cloud as `base_link` without a TF lookup; grid_builder relabels as `map` | pointcloud_params.yaml:2,13; depth_to_scan.py:81-83; grid_builder.py:87 |
| Nav2 | global: map / base_link; local: odom / base_link | navigation.yaml:4-5, 31-32 |
| diff_drive_controller | base `base_link`, odom default `odom` (line commented out), `enable_odom_tf: false` | rover_controllers.yaml:33-34, 41 |
| aruco | output frame hard-coded to `zed2i_left_camera_optical_frame`; the image header frame is ignored | detector_params.yaml:24; node.py:168-169, 282-284 |
| pose_estimator / select_object | `map` via TF from the cloud frame / `base_link` and `map` | estimator_params.yaml:4; node.py:234-236; select_object_service.py:262,293 |

**Mismatches and double-publication risks:**
1. The URDF uses `Front_Zed_*`, but the driver (P1) and the perception configs use `zed2i_*`. Under P1 the camera frames reach `base_link` only through the ZED's own `odom→zed2i_camera_link` VIO TF, not through the 0.55 m mount.
2. If you just set `camera_name:=Front_Zed` and leave defaults, `Front_Zed_camera_link` gets **two parents**: `base_link` from rover robot_state_publisher, and `odom` from ZED `publish_tf` (zed_camera.launch.py:403-407). A second robot_state_publisher also republishes the same links (:234-248).
3. **`map→odom` is published twice**: by the ZED (`publish_map_tf` default true, :408-412) and by ekf_global (localization.yaml:51 / ekf.yaml:33).
4. **`base_link` gets a second parent** when zed_gnss_fusion runs, because `publish_tf: true` makes it publish `map→base_link` (fusion_params.yaml:17; zed_gnss_fusion.py:315-316,367-374).
5. **`zed2i_left_camera_frame` gets two parents** when frame_coordinator runs alongside P1 (frame_coordinator.py:71-80 vs the ZED robot_state_publisher).
6. The IMU frame `*_imu_link` has no TF, so robot_localization can't rotate imu0 into base_link and drops it.
7. The GPS frames `gps_link` and `gnss_link` don't match each other, and neither is in the URDF.
8. ZED odom's child frame is `*_camera_link`, not `base_link`. ekf_local fuses absolute x/y/yaw from it and from wheel odom together (localization.yaml:14-25), with a 0.55 m lever arm.
9. Diff drive `enable_odom_tf: false` is correct. If you add Gazebo's OdometryPublisher in sim, **don't bridge its TF**, or `odom→base_link` gets published twice.

---

## 6. Prioritised step-2 checklist

1. **Make the stack parse.** Add the commas at localization.yaml:23-24. Wrap pointcloud_params.yaml, estimator_params.yaml and zed2i.yaml in `<node>: ros__parameters:` (or `/**:`). Remap the dual-EKF outputs: ekf_local `odometry/filtered→/odometry/local`, ekf_global `→/odometry/global`, navsat `odometry/filtered→/odometry/global`, then point ekf_global odom0 at `/odometry/local`. Fix the navsat remap key to `imu`. Files: rover_navigation/config/localization.yaml, rover_navigation/launch/location.launch.py, and loc_fusion/launch/loc_fusion.launch.py plus config/ekf.yaml (B4/B5).
2. **Adopt `/Front_Zed/zed_node/` and `Front_Zed_*` frames** using the edit list in section 2.
3. **Add a `use_sim_time` launch arg** (default `false`) and pass it to every node. Replace location.launch.py:43,56,73 and navigation.launch.py:64. Delete navigation.yaml:106 and add the param to navigation.launch.py:21,30,38,46,54. Add the same arg to aruco_detector.launch.py, object_detection.launch.py, pose_estimator.launch.py and pointcloud_tools.launch.py. Without sim time in sim, aruco's ApproximateTimeSynchronizer and every TF lookup break.
4. **URDF for sim** (rover_description/urdf/rover.urdf.xacro:616-640, the file the other agent is editing):
   - Move the ZED macro out of `xacro:unless use_gazebo`, using a macro that doesn't need `zed_wrapper` or `zed_msgs`.
   - Add a fixed `Front_Zed_imu_link` and a `gps_link`.
   - Add these sensors (Fortress; set `<ignition_frame_id>`, which the installed gz-sensors6 accepts, as does `gz_frame_id`):
     - `<gazebo reference="Front_Zed_left_camera_frame"><sensor type="rgbd_camera">` with `<topic>Front_Zed/zed_node/rgbd</topic>`. Gazebo publishes `…/rgbd/image`, `…/rgbd/depth_image`, `…/rgbd/points` and `…/rgbd/camera_info`. Set the frame to `Front_Zed_left_camera_frame`, which is what the real ZED cloud uses (zed_camera_component_main.cpp:1836). Check the cloud axes in RViz once.
     - `<sensor type="imu">` on `Front_Zed_imu_link`, topic `Front_Zed/zed_node/imu/data`.
     - `<sensor type="navsat">` on `gps_link`, topic `gps/fix`.
   - System plugins: `ignition-gazebo-sensors-system` (`ignition::gazebo::systems::Sensors`, `<render_engine>ogre2`), `ignition-gazebo-imu-system` (`…::Imu`), `ignition-gazebo-navsat-system` (`…::NavSat`), and optionally `ignition-gazebo-odometry-publisher-system` (`…::OdometryPublisher`) as a stand-in for ZED VIO on `Front_Zed/zed_node/odom`. All the `.dylib`s are present in `.pixi/envs/default/lib`.
   - NavSat needs `<spherical_coordinates>` in the world, and gazebo.launch.py:37 loads `empty.sdf`, which has none. Ship a world file, and put ArUco marker models in it.
5. **Bridge.** Extend gazebo.launch.py:40-46, preferably with a `config_file` YAML mapping `gz_topic_name` to `ros_topic_name`. The installed ros_gz_bridge 0.244.24 accepts both `ignition.msgs.*` and `gz.msgs.*` (checked in the binary); use `ignition.msgs.*` for Fortress:
   | ROS topic | ROS type | Gazebo type | Gazebo source |
   |---|---|---|---|
   | /clock | rosgraph_msgs/msg/Clock | ignition.msgs.Clock | world |
   | /Front_Zed/zed_node/left/image_rect_color (also rgb/…) | sensor_msgs/msg/Image | ignition.msgs.Image | …/rgbd/image |
   | /Front_Zed/zed_node/left/camera_info (also depth/camera_info) | sensor_msgs/msg/CameraInfo | ignition.msgs.CameraInfo | …/rgbd/camera_info |
   | /Front_Zed/zed_node/depth/depth_registered | sensor_msgs/msg/Image (32FC1, metres) | ignition.msgs.Image | …/rgbd/depth_image |
   | /Front_Zed/zed_node/point_cloud/cloud_registered | sensor_msgs/msg/PointCloud2 | ignition.msgs.PointCloudPacked | …/rgbd/points |
   | /Front_Zed/zed_node/imu/data | sensor_msgs/msg/Imu | ignition.msgs.IMU | imu sensor |
   | /gps/fix | sensor_msgs/msg/NavSatFix | ignition.msgs.NavSat | navsat sensor |
   | /Front_Zed/zed_node/odom (optional) | nav_msgs/msg/Odometry | ignition.msgs.Odometry | OdometryPublisher |

   Direction is GZ→ROS (`[`) for all rows. Don't bridge `Pose_V`/TF. Wheel odometry and `cmd_vel` already go through gz_ros2_control's diff_drive_controller (`/rover_drive_controller/odom`, `/rover_drive_controller/cmd_vel_unstamped`).
6. **TF ownership on hardware.** Pass `publish_urdf:=false publish_tf:=false publish_map_tf:=false publish_imu_tf:=true` in zed2i_driver.launch.py. Disable frame_coordinator's static TFs and zed_gnss_fusion's `publish_tf` whenever the EKFs run.
7. **Wire up the detection consumers.** Fix SearchPattern.py:75 to `vision_msgs/Detection3DArray` on `/aruco_detections`. Set the aruco `output_frame` to `Front_Zed_left_camera_optical_frame`. ONNX needs nothing beyond the topic rename, since it falls back to CPU.
8. **Leave out of the sim launch:** zed_object_detector, zed_gnss_fusion, zed_bridge, zed_integrated_detection, and the `zed_integration` includes in slam/perception_complete (section 4A).
