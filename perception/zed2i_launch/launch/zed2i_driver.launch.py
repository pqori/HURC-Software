#!/usr/bin/env python3
"""
Launches the Stereolabs zed_wrapper node with parameters tuned for the ZED 2i.
==========================================================================

The ZED 2i wrapper publishes 100+ topics across multiple namespaces. Key topics include:

┌─────────────────────────────────────────────────────┬────────────────────────────┐
│ TOPIC                                               │ MESSAGE TYPE               │
├─────────────────────────────────────────────────────┼────────────────────────────┤
│ CAMERA IMAGES & INFO                                │                            │
│ /zed2i/zed2i_camera/left/image_rect_color           │ sensor_msgs/Image          │
│ /zed2i/zed2i_camera/left/camera_info                │ sensor_msgs/CameraInfo     │
│ /zed2i/zed2i_camera/right/image_rect_color          │ sensor_msgs/Image          │
│ /zed2i/zed2i_camera/right/camera_info               │ sensor_msgs/CameraInfo     │
│ /zed2i/zed2i_camera/rgb/image_rect_color            │ sensor_msgs/Image          │
│ /zed2i/zed2i_camera/rgb/camera_info                 │ sensor_msgs/CameraInfo     │
│                                                     │                            │
│ DEPTH & POINT CLOUD                                 │                            │
│ /zed2i/zed2i_camera/depth/depth_registered          │ sensor_msgs/Image          │
│ /zed2i/zed2i_camera/depth/camera_info               │ sensor_msgs/CameraInfo     │
│ /zed2i/zed2i_camera/confidence/confidence_map       │ sensor_msgs/Image          │
│ /zed2i/zed2i_camera/point_cloud/cloud_registered    │ sensor_msgs/PointCloud2    │
│ /zed2i/zed2i_camera/disparity/disparity_image       │ stereo_msgs/DisparityImage │
│                                                     │                            │
│ IMU & SENSORS                                       │                            │
│ /zed2i/zed2i_camera/imu/data                        │ sensor_msgs/Imu            │
│ /zed2i/zed2i_camera/imu/data_raw                    │ sensor_msgs/Imu            │
│ /zed2i/zed2i_camera/imu/mag                         │ sensor_msgs/MagneticField  │
│ /zed2i/zed2i_camera/temperature/imu                 │ sensor_msgs/Temperature    │
│ /zed2i/zed2i_camera/temperature/left                │ sensor_msgs/Temperature    │
│ /zed2i/zed2i_camera/temperature/right               │ sensor_msgs/Temperature    │
│ /zed2i/zed2i_camera/atm_press                       │ sensor_msgs/FluidPressure  │
│                                                     │                            │
│ ODOMETRY & POSE                                     │                            │
│ /zed2i/zed2i_camera/odom                            │ nav_msgs/Odometry          │
│ /zed2i/zed2i_camera/pose                            │ geometry_msgs/PoseStamped  │
│ /zed2i/zed2i_camera/pose_with_covariance            │ geometry_msgs/PoseWithCov… │
│ /zed2i/zed2i_camera/path_map                        │ nav_msgs/Path              │
│ /zed2i/zed2i_camera/path_odom                       │ nav_msgs/Path              │
│                                                     │                            │
│ STATUS & DIAGNOSTICS                                │                            │
│ /zed2i/zed2i_camera/status/health                   │ diagnostic_msgs/Diagnostic │
│ /zed2i/zed2i_camera/status/heartbeat                │ std_msgs/Bool              │
│                                                     │                            │
│ ADDITIONAL TOPICS                                   │                            │
│ /zed2i/zed2i_camera/plane                           │ geometry_msgs/Plane        │
│ /zed2i/zed2i_camera/plane_marker                    │ visualization_msgs/Marker  │
│ /zed2i/zed2i_camera/left_cam_imu_transform          │ geometry_msgs/Transform    │
│                                                     │                            │
│ REMAPPED TOPICS (for compatibility)                 │                            │
│ /zed2i/left/image_rect_color                        │ sensor_msgs/Image          │
│ /zed2i/left/camera_info                             │ sensor_msgs/CameraInfo     │
│ /zed2i/right/image_rect_color                       │ sensor_msgs/Image          │
│ /zed2i/right/camera_info                            │ sensor_msgs/CameraInfo     │
│ /zed2i/depth/depth_registered                       │ sensor_msgs/Image          │
│ /zed2i/depth/camera_info                            │ sensor_msgs/CameraInfo     │
│ /zed2i/confidence/confidence_map                    │ sensor_msgs/Image          │
│ /zed2i/point_cloud/cloud_registered                 │ sensor_msgs/PointCloud2    │
│ /zed2i/imu/data                                     │ sensor_msgs/Imu            │
│ /zed2i/imu/data_raw                                 │ sensor_msgs/Imu            │
│ /zed2i/imu/mag                                      │ sensor_msgs/MagneticField  │
│ /zed2i/odom                                         │ nav_msgs/Odometry          │
│ /zed2i/pose                                         │ geometry_msgs/PoseStamped  │
│ /zed2i/pose_with_covariance                         │ geometry_msgs/PoseWithCov… │
│ /zed2i/rgb/camera_info                              │ sensor_msgs/CameraInfo     │
│ /zed2i/temperature/imu                              │ sensor_msgs/Temperature    │
│ /zed2i/diagnostics                                  │ diagnostic_msgs/Diagnostic │
└─────────────────────────────────────────────────────┴────────────────────────────┘

Note: All image topics also publish compressed versions (e.g., /compressed, /compressedDepth, /theora)

Key streams for rover navigation and perception:
  • /zed2i/zed2i_camera/odom                    — 6-DoF visual-inertial odometry
  • /zed2i/zed2i_camera/imu/data                — IMU data at 100Hz for sensor fusion
  • /zed2i/zed2i_camera/point_cloud/cloud_registered — XYZRGBA point cloud for mapping
  • /zed2i/zed2i_camera/left/image_rect_color   — Rectified color images for vision
  • /zed2i/zed2i_camera/depth/depth_registered  — Depth images for obstacle detection

TF Frame Tree:
  map → odom → zed2i_camera_link → zed2i_camera_center → zed2i_left_camera_frame
                                                      → zed2i_right_camera_frame
                                                      → zed2i_imu_link

Configuration: Edit params/zed2i.yaml to customize topic rates and enable/disable features.
Testing: Run ./test_zed2i.sh to launch and verify all topics are publishing correctly.
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Path to our custom parameters
    params_file = os.path.join(
        get_package_share_directory('zed2i_launch'),
        'params', 'zed2i.yaml')

    # Include the main ZED launch file with our custom parameters
    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('zed_wrapper'),
            '/launch/zed_camera.launch.py'
        ]),
        launch_arguments={
            'camera_model': 'zed2i',
            'camera_name': 'zed2i',
            'node_name': 'zed2i_camera',
            'ros_params_override_path': params_file,
        }.items()
    )

    return LaunchDescription([zed_launch])
