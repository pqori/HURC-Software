#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def share(pkg: str, relpath: str) -> str:
    return os.path.join(get_package_share_directory(pkg), relpath)


def generate_launch_description():
    """Enhanced SLAM with hybrid ZED + RTAB-Map mapping"""
    
    # Launch arguments
    args = [
        DeclareLaunchArgument(
            'mapping_mode', default_value='hybrid_fusion',
            description='Mapping mode: hybrid_fusion, zed_primary, rtabmap_only, auto_select'
        ),
        DeclareLaunchArgument(
            'enable_zed_mapping', default_value='true',
            description='Enable ZED GPU spatial mapping'
        ),
        DeclareLaunchArgument(
            'enable_rtabmap', default_value='true', 
            description='Enable RTAB-Map SLAM'
        )
    ]

    # Core perception (always active)
    zed_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            share('zed2i_launch', 'launch/zed2i_driver.launch.py'))
    )

    fusion = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            share('loc_fusion', 'launch/loc_fusion.launch.py'))
    )

    # ZED Spatial Mapping (conditional)
    zed_mapping = GroupAction(
        condition=IfCondition(LaunchConfiguration('enable_zed_mapping')),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    share('zed_integration', 'launch/spatial_mapping.launch.py')
                ),
                launch_arguments={
                    'mapping_resolution': '0.05',
                    'mapping_range': '10.0',
                    'publish_occupancy_grid': 'false'  # Coordinator handles grid
                }.items()
            )
        ]
    )

    # RTAB-Map SLAM (conditional) 
    rtabmap = GroupAction(
        condition=IfCondition(LaunchConfiguration('enable_rtabmap')),
        actions=[
            Node(
                package='rtabmap_launch',
                executable='rtabmap.launch.py',
                name='rtabmap',
                output='screen',
                parameters=[share('slam_launch', 'config/rtabmap_params.yaml')],
                remappings=[
                    ('rgb/image', '/zed2i/left/image_rect_color'),
                    ('rgb/camera_info', '/zed2i/left/camera_info'),
                    ('depth/image', '/zed2i/depth/depth_registered'),
                    ('depth/camera_info', '/zed2i/depth/camera_info'),
                    ('odom', '/odometry/filtered'),
                    ('grid_map', '/rtabmap/grid_map')  # Remap for coordinator
                ]
            )
        ]
    )

    # Mapping Coordinator (hybrid intelligence)
    mapping_coordinator = Node(
        package='slam_launch',
        executable='mapping_coordinator',
        name='mapping_coordinator',
        output='screen',
        parameters=[
            share('slam_launch', 'config/hybrid_mapping.yaml'),
            {
                'mapping_mode': LaunchConfiguration('mapping_mode')
            }
        ]
    )

    return LaunchDescription([
        *args,
        zed_driver,
        fusion,
        zed_mapping,
        rtabmap,
        mapping_coordinator
    ])
