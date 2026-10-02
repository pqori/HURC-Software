#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg = 'detection_pose_estimator'
    cfg = os.path.join(
        get_package_share_directory(pkg),
        'config', 'estimator_params.yaml'
    )

    return LaunchDescription([
        Node(
            package=pkg,
            executable='pose_estimator',
            name='pose_estimator',
            output='screen',
            parameters=[cfg]
        ),
    ])
