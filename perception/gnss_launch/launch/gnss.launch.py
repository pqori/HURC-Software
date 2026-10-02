#!/usr/bin/env python3
"""
GNSS System Launch
Launches ZED-F9P GPS driver with health monitoring
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    gnss_src = os.path.dirname(os.path.abspath(__file__))

    # GPS Driver
    gnss_driver = Node(
        package='ublox_gps',
        executable='ublox_gps_node',
        name='gnss_driver',
        output='screen',
        parameters=[{
            'device': '/dev/ttyACM0',
            'frame_id': 'gps_link',
            'baudrate': 38400,
            'rate': 1.0,
            'nav_rate': 1,
            'enable_ppp': False,
            'tmode3': 0,
            'debug': 1,
            'dgnss_mode': 3,  # RTK Fixed mode
            'configure_gps': False,  # Don't reconfigure GPS
        }],
        remappings=[
            ('fix', '/gps/fix'),
            ('fix_velocity', '/gps/velocity')
        ]
    )

    # Health Monitor (start after driver is up)
    health_monitor = TimerAction(
        period=3.0,
        actions=[ExecuteProcess(
            cmd=['python3', f'{gnss_src}/gnss_health_monitor.py'],
            output='screen',
            name='gnss_health_monitor'
        )]
    )

    # Validator (start after driver is up)
    validator = TimerAction(
        period=3.0,
        actions=[ExecuteProcess(
            cmd=['python3', f'{gnss_src}/gnss_validator.py'],
            output='screen',
            name='gnss_validator'
        )]
    )

    return LaunchDescription([
        gnss_driver,
        health_monitor,
        validator,
    ])
