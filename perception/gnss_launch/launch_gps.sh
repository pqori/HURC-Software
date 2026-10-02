#!/bin/bash
# Simple GPS Launch Script

source /opt/ros/humble/setup.bash
source ~/workspaces/ros2-ublox-zedf9p/install/setup.bash

ros2 launch ~/workspaces/rover/install/gnss_launch/share/gnss_launch/launch/gnss.launch.py "$@"

