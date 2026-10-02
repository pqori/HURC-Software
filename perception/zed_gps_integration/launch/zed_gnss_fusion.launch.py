#!/usr/bin/env python3
"""
ZED GNSS Fusion Launch

Launches complete system:
- ZED camera with GNSS fusion
- Map server bridge for web visualization
- Optional: GNSS driver (if not already running)
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description for ZED GNSS fusion system"""

    # Package directories
    pkg_share = FindPackageShare('zed_gps_integration')

    # Configuration files
    fusion_config = PathJoinSubstitution(
        [pkg_share, 'config', 'fusion_params.yaml'])
    map_config = PathJoinSubstitution(
        [pkg_share, 'config', 'map_server_params.yaml'])

    # Launch arguments
    declare_launch_gnss = DeclareLaunchArgument(
        'launch_gnss',
        default_value='false',
        description='Launch GNSS driver (set false if already running)'
    )

    declare_enable_map = DeclareLaunchArgument(
        'enable_map',
        default_value='true',
        description='Enable web map visualization'
    )

    declare_camera_sn = DeclareLaunchArgument(
        'camera_sn',
        default_value='0',
        description='ZED camera serial number (0 for auto-detect)'
    )

    declare_gnss_topic = DeclareLaunchArgument(
        'gnss_topic',
        default_value='/gps/fix',
        description='GNSS NavSatFix topic'
    )

    # Optional: Launch GNSS driver
    gnss_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gnss_launch'),
                'launch',
                'gnss.launch.py'
            ])
        ]),
        condition=IfCondition(LaunchConfiguration('launch_gnss'))
    )

    # Main fusion node
    fusion_node = Node(
        package='zed_gps_integration',
        executable='zed_gnss_fusion',
        name='zed_gnss_fusion',
        output='screen',
        parameters=[
            fusion_config,
            {
                'camera_sn': LaunchConfiguration('camera_sn'),
                'gnss_topic': LaunchConfiguration('gnss_topic'),
            }
        ],
        emulate_tty=True
    )

    # Map server bridge
    map_server_node = Node(
        package='zed_gps_integration',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            map_config,
            {
                'raw_gnss_topic': LaunchConfiguration('gnss_topic'),
            }
        ],
        condition=IfCondition(LaunchConfiguration('enable_map')),
        emulate_tty=True
    )

    # Informational messages
    info_start = LogInfo(
        msg=[
            '\n',
            '=' * 70, '\n',
            '  ZED GNSS FUSION - Global Localization System\n',
            '=' * 70, '\n',
            '  1. Wait for GPS fix (check: ros2 topic echo /gps/fix)\n',
            '  2. Move camera 10-15m forward + turn 30-45 degrees\n',
            '  3. Watch for "Calibration complete" message\n',
            '  4. View live map: cd ~/zed_map_data && python3 -m http.server 8000\n',
            '     Then open: http://localhost:8000/\n',
            '=' * 70, '\n'
        ]
    )

    return LaunchDescription([
        # Arguments
        declare_launch_gnss,
        declare_enable_map,
        declare_camera_sn,
        declare_gnss_topic,

        # Info
        info_start,

        # Nodes
        gnss_launch,
        fusion_node,
        map_server_node,
    ])
