#!/usr/bin/env python3
"""
Full localization launch - GPS + ZED fusion
  ▸ GPS driver             → /gps/fix
  ▸ ZED 2i                 → /zed2i/odom, /zed2i/imu/data
  ▸ ekf_local              → /odometry/filtered (odom→base_link)
  ▸ navsat_transform       → /odom/gps (GPS in odom frame)
  ▸ ekf_global             → /odometry/global (map→odom)
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def share(pkg, rel):
    return os.path.join(get_package_share_directory(pkg), rel)

def generate_launch_description():

    gnss_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            share('gnss_launch', 'launch/gnss.launch.py'))
    )

    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            share('zed2i_launch', 'launch/zed2i_driver.launch.py'))
    )

    ekf_local = Node(
        package='robot_localization', executable='ekf_node',
        name='ekf_local', output='screen',
        parameters=[share('loc_fusion', 'config/ekf.yaml')]
    )

    navsat = Node(
        package='robot_localization', executable='navsat_transform_node',
        name='navsat_transform', output='screen',
        parameters=[share('loc_fusion', 'config/navsat_transform.yaml')],
        remappings=[
            ('imu',      '/zed2i/imu/data'),
            ('gps/fix',  '/gps/fix'),
            ('odom',     '/odometry/filtered'),
            ('filtered', '/odom/gps')
        ]
    )

    ekf_global = Node(
        package='robot_localization', executable='ekf_node',
        name='ekf_global', output='screen',
        parameters=[share('loc_fusion', 'config/ekf.yaml')],
        remappings=[
            ('odometry/filtered', '/odometry/global')
        ]
    )

    return LaunchDescription([
        gnss_launch,
        zed_launch,
        ekf_local,
        navsat,
        ekf_global
    ])
