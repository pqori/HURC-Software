#!/usr/bin/env python3
"""
Launch file for ArUco detector (composable node).

Supports both standalone and composable container modes.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node, ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Generate launch description for ArUco detector."""
    
    pkg_share = get_package_share_directory('aruco_detector')
    default_params = os.path.join(pkg_share, 'config', 'detector_params.yaml')
    
    # Launch arguments
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='Path to detector parameters YAML file'
    )
    
    use_composable_arg = DeclareLaunchArgument(
        'use_composable',
        default_value='true',
        description='Use composable node in isolated container'
    )
    
    container_name_arg = DeclareLaunchArgument(
        'container_name',
        default_value='aruco_container',
        description='Name of component container'
    )
    
    use_intra_process_arg = DeclareLaunchArgument(
        'use_intra_process',
        default_value='true',
        description='Enable intra-process communication'
    )
    
    # Composable node description
    composable_node = ComposableNode(
        package='aruco_detector',
        plugin='aruco_detector.node::ArucoDetectorNode',
        name='aruco_detector',
        parameters=[LaunchConfiguration('params_file')],
        extra_arguments=[{'use_intra_process_comms': LaunchConfiguration('use_intra_process')}]
    )
    
    # Option 1: Composable node in isolated container
    container = ComposableNodeContainer(
        condition=IfCondition(LaunchConfiguration('use_composable')),
        name=LaunchConfiguration('container_name'),
        namespace='',
        package='rclcpp_components',
        executable='component_container_isolated',
        composable_node_descriptions=[composable_node],
        output='screen',
        emulate_tty=True,
    )
    
    # Option 2: Standalone node (fallback)
    standalone_node = Node(
        condition=UnlessCondition(LaunchConfiguration('use_composable')),
        package='aruco_detector',
        executable='aruco_detector_node',
        name='aruco_detector',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
        emulate_tty=True,
    )
    
    return LaunchDescription([
        params_file_arg,
        use_composable_arg,
        container_name_arg,
        use_intra_process_arg,
        container,
        standalone_node,
    ])
