#!/usr/bin/env python3
"""
URC Perception Development Mode
==============================

Lightweight launch file for development and testing.
Allows selective component activation for debugging.

Usage:
    # Test only SLAM
    ros2 launch slam_launch perception_dev.launch.py

    # Test SLAM + Object Detection
    ros2 launch slam_launch perception_dev.launch.py enable_object_detection:=true

    # Test specific components
    ros2 launch slam_launch perception_dev.launch.py \
        enable_object_detection:=true \
        enable_aruco_detection:=false \
        debug_mode:=true
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def get_package_path(package: str, *paths: str) -> str:
    """Get path within a ROS2 package"""
    return os.path.join(get_package_share_directory(package), *paths)


def generate_launch_description():
    """Generate development mode launch description"""
    
    return LaunchDescription([
        # Development-focused arguments (defaults for testing)
        DeclareLaunchArgument(
            'enable_object_detection', default_value='false',
            description='Enable object detection for testing'
        ),
        DeclareLaunchArgument(
            'enable_aruco_detection', default_value='false',
            description='Enable ArUco detection for testing'  
        ),
        DeclareLaunchArgument(
            'enable_pose_estimation', default_value='false',
            description='Enable pose estimation for testing'
        ),
        DeclareLaunchArgument(
            'debug_mode', default_value='true',
            description='Enable debug logging (default: true for dev mode)'
        ),
        
        LogInfo(msg="🔧 Starting URC Perception Development Mode..."),
        
        # Use the complete perception launch with dev-friendly defaults
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                get_package_path('slam_launch', 'launch', 'perception_complete.launch.py')
            ),
            launch_arguments={
                'enable_object_detection': LaunchConfiguration('enable_object_detection'),
                'enable_aruco_detection': LaunchConfiguration('enable_aruco_detection'), 
                'enable_pose_estimation': LaunchConfiguration('enable_pose_estimation'),
                'enable_recording': 'false',  # No recording in dev mode
                'enable_monitoring': 'true',   # Always monitor in dev
                'debug_mode': LaunchConfiguration('debug_mode')
            }.items()
        )
    ])
