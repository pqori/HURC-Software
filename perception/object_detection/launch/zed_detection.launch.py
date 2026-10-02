#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """Launch ZED native object detection"""
    
    # Get package directory
    pkg = 'object_detection'
    pkg_share = get_package_share_directory(pkg)
    
    # Configuration file
    config_file = os.path.join(pkg_share, 'config', 'zed_detection_params.yaml')
    
    # Launch arguments
    args = [
        DeclareLaunchArgument(
            'detection_model',
            default_value='MULTI_CLASS_BOX_ACCURATE',
            description='ZED detection model (MULTI_CLASS_BOX_ACCURATE, MULTI_CLASS_BOX_FAST, etc.)'
        ),
        DeclareLaunchArgument(
            'enable_tracking',
            default_value='true',
            description='Enable object tracking with persistent IDs'
        ),
        DeclareLaunchArgument(
            'detection_confidence',
            default_value='0.5',
            description='Minimum detection confidence threshold'
        ),
        DeclareLaunchArgument(
            'max_range',
            default_value='20.0',
            description='Maximum detection range in meters'
        ),
        DeclareLaunchArgument(
            'publish_rate',
            default_value='10.0',
            description='Detection publishing rate in Hz'
        )
    ]
    
    # ZED Object Detection Node
    zed_detector_node = Node(
        package=pkg,
        executable='zed_object_detector',
        name='zed_object_detector',
        output='screen',
        parameters=[
            config_file,
            {
                'detection_model': LaunchConfiguration('detection_model'),
                'enable_tracking': LaunchConfiguration('enable_tracking'),
                'detection_confidence': LaunchConfiguration('detection_confidence'),
                'max_range': LaunchConfiguration('max_range'),
                'publish_rate': LaunchConfiguration('publish_rate'),
            }
        ],
        remappings=[
            # Add any topic remappings here if needed
        ]
    )
    
    # Object Selection Service (updated for ZED 3D)
    selection_service_node = Node(
        package=pkg,
        executable='select_object_service',
        name='object_selection_service',
        output='screen',
        parameters=[{
            'use_zed_3d': True  # Prefer ZED 3D detections
        }]
    )
    
    return LaunchDescription([
        *args,
        zed_detector_node,
        selection_service_node
    ])
