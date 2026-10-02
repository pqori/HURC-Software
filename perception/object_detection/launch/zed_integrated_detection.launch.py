#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    """Launch ZED ROS2 wrapper with object detection + our bridge"""
    
    # Get package directories
    zed_wrapper_share = get_package_share_directory('zed_wrapper')
    object_detection_share = get_package_share_directory('object_detection')
    
    # Configuration files
    zed_config = os.path.join(zed_wrapper_share, 'config', 'zed2i.yaml')
    od_config = os.path.join(zed_wrapper_share, 'config', 'object_detection.yaml')
    
    # Launch arguments
    args = [
        DeclareLaunchArgument(
            'camera_model',
            default_value='zed2i',
            description='ZED camera model (zed, zed2, zed2i, zedm, etc.)'
        ),
        DeclareLaunchArgument(
            'detection_model', 
            default_value='MULTI_CLASS_BOX_ACCURATE',
            description='ZED detection model'
        ),
        DeclareLaunchArgument(
            'enable_tracking',
            default_value='true',
            description='Enable object tracking'
        ),
        DeclareLaunchArgument(
            'max_range',
            default_value='20.0', 
            description='Maximum detection range in meters'
        ),
        DeclareLaunchArgument(
            'min_confidence',
            default_value='50.0',
            description='Minimum detection confidence (0-100)'
        ),
        DeclareLaunchArgument(
            'use_custom_model',
            default_value='false',
            description='Use custom ONNX model for URC objects'
        ),
        DeclareLaunchArgument(
            'custom_model_path',
            default_value='',
            description='Path to custom ONNX model file'
        )
    ]
    
    # ZED Camera Launch (with object detection enabled)
    zed_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(zed_wrapper_share, 'launch', 'zed_camera.launch.py')
        ]),
        launch_arguments={
            'camera_model': LaunchConfiguration('camera_model'),
            'config_common_path': zed_config,
            'config_od_path': od_config,
            # Object detection parameters
            'object_detection.od_enabled': 'true',
            'object_detection.detection_model': LaunchConfiguration('detection_model'),
            'object_detection.enable_tracking': LaunchConfiguration('enable_tracking'),
            'object_detection.max_range': LaunchConfiguration('max_range'),
            'object_detection.confidence_threshold': LaunchConfiguration('min_confidence'),
            # Custom model parameters (if enabled)
            'object_detection.custom_onnx_file': LaunchConfiguration('custom_model_path'),
        }.items()
    )
    
    # ZED Object Detection Bridge
    zed_bridge_node = Node(
        package='object_detection',
        executable='zed_object_bridge',
        name='zed_object_bridge',
        output='screen',
        parameters=[{
            'zed_objects_topic': '/zed2i/obj_det/objects',
            'output_topic': '/zed_detections_3d',
            'min_confidence': 0.5  # Convert from 0-100 to 0-1 scale
        }],
        remappings=[
            # Remap topics if needed
        ]
    )
    
    # Object Selection Service (updated for ZED 3D)
    selection_service_node = Node(
        package='object_detection',
        executable='select_object_service',
        name='object_selection_service',
        output='screen',
        parameters=[{
            'use_zed_3d': True  # Prefer ZED 3D detections
        }]
    )
    
    return LaunchDescription([
        *args,
        zed_camera_launch,
        zed_bridge_node,
        selection_service_node
    ])
