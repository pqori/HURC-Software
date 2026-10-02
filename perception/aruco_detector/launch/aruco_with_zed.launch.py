#!/usr/bin/env python3
"""
Combined launch file: ZED 2i camera + ArUco detector

Launches both the ZED camera driver and ArUco detector node,
then you can monitor detections with the provided script.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Generate launch description for ZED + ArUco."""
    
    # Get package directories
    aruco_pkg = get_package_share_directory('aruco_detector')
    aruco_params = os.path.join(aruco_pkg, 'config', 'detector_params.yaml')
    
    # Launch arguments
    use_composable_arg = DeclareLaunchArgument(
        'use_composable',
        default_value='false',
        description='Use composable node (set true for production)'
    )
    
    # Include ZED 2i launch file
    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('zed2i_launch'),
            '/launch/zed2i_driver.launch.py'
        ])
    )
    
    # ArUco detector node (standalone - easier for testing)
    # Use Python directly since it's a Python node
    from launch.actions import ExecuteProcess
    aruco_node = ExecuteProcess(
        cmd=[
            'python3',
            PathJoinSubstitution([
                FindPackageShare('aruco_detector'),
                'bin',
                'aruco_detector_node'
            ])
        ],
        output='screen',
        additional_env={
            'PYTHONUNBUFFERED': '1',
            'ROS_LOCALHOST_ONLY': '1'
        },
        name='aruco_detector'
    )
    
    return LaunchDescription([
        use_composable_arg,
        zed_launch,
        aruco_node,
    ])

