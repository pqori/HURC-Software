#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    cfg = os.path.join(
        get_package_share_directory('pointcloud_tools'),
        'config', 'pointcloud_params.yaml')

    depth_to_scan = Node(
        package='pointcloud_tools', executable='depth_to_scan',
        name='depth_to_scan', parameters=[cfg],
        output='screen')

    grid_builder = Node(
        package='pointcloud_tools', executable='grid_builder',
        name='grid_builder', parameters=[cfg],
        output='screen')

    return LaunchDescription([depth_to_scan, grid_builder])
