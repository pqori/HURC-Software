#!/bin/bash
# Comprehensive test: ZED camera + ArUco detector + monitoring

cd /home/rover/workspaces/rover

echo "=========================================="
echo "ArUco Full System Test"
echo "=========================================="
echo "This will:"
echo "  1. Clean up old processes"
echo "  2. Launch ZED 2i camera"
echo "  3. Launch ArUco detector"
echo "  4. Monitor detections"
echo ""
echo "Make sure you have ArUco markers visible!"
echo "=========================================="
echo ""

# Clean up any existing ZED/ArUco processes
echo "Cleaning up old processes..."
pkill -f "zed2i_driver" 2>/dev/null
pkill -f "aruco_detector" 2>/dev/null
pkill -f "standalone_detector" 2>/dev/null
pkill -f "zed_container" 2>/dev/null
sleep 2
echo "✓ Cleanup complete"
echo ""

# Fix network config
unset CYCLONEDDS_URI
export ROS_LOCALHOST_ONLY=1
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{name}]: {message}"
export RCUTILS_COLORIZED_OUTPUT=1

# Source workspace
source install/setup.bash

echo "Building aruco_detector (if needed)..."
colcon build --packages-select aruco_detector --symlink-install > /tmp/build.log 2>&1
if [ $? -eq 0 ]; then
    echo "✓ Build successful"
    source install/setup.bash
else
    echo "✗ Build failed, check /tmp/build.log"
    exit 1
fi

echo ""
echo "=========================================="
echo "Launching ZED camera..."
echo "=========================================="
echo ""

# Launch ZED camera
ros2 launch zed2i_launch zed2i_driver.launch.py > /tmp/zed_launch.log 2>&1 &
ZED_PID=$!

echo "ZED camera PID: $ZED_PID"
echo "Waiting 5 seconds for ZED initialization..."
sleep 5

# Check if still running
if ! ps -p $ZED_PID > /dev/null; then
    echo "✗ ZED launch failed! Check /tmp/zed_launch.log"
    cat /tmp/zed_launch.log
    exit 1
fi

echo "✓ ZED camera running"

echo ""
echo "=========================================="
echo "Launching ArUco detector..."
echo "=========================================="
echo ""

# Launch ArUco detector
python3 install/aruco_detector/bin/aruco_detector_node \
    --ros-args \
    -p rgb_topic:=/zed2i/zed2i_camera/left/image_rect_color \
    -p cam_info_topic:=/zed2i/zed2i_camera/left/camera_info \
    -p depth_topic:=/zed2i/zed2i_camera/depth/depth_registered \
    -p use_depth:=true \
    -p marker_size_m:=0.15 \
    -r aruco_detections:=/aruco/detections \
    -r aruco_markers:=/aruco/markers \
    > /tmp/aruco.log 2>&1 &
ARUCO_PID=$!

echo "ArUco detector PID: $ARUCO_PID"
echo "Waiting 3 seconds for ArUco initialization..."
sleep 3

# Check if still running
if ! ps -p $ARUCO_PID > /dev/null; then
    echo "✗ ArUco launch failed! Check /tmp/aruco.log"
    cat /tmp/aruco.log
    kill $ZED_PID 2>/dev/null
    exit 1
fi

echo "✓ System initialized"
echo ""
echo "=========================================="
echo "Checking topics..."
echo "=========================================="

# List topics
ros2 topic list 2>/dev/null | grep -E "(zed|aruco)" | head -10

echo ""
echo "=========================================="
echo "Starting detection monitor..."
echo "=========================================="
echo "Hold ArUco markers (ID 0-9) in front of camera"
echo "Press Ctrl+C when done"
echo "=========================================="
echo ""

# Run monitor
python3 src/perception/aruco_detector/scripts/monitor_detections.py

echo ""
echo "=========================================="
echo "Cleaning up..."
echo "=========================================="

# Kill both processes
echo "Stopping ArUco detector..."
kill $ARUCO_PID 2>/dev/null
wait $ARUCO_PID 2>/dev/null

echo "Stopping ZED camera..."
kill $ZED_PID 2>/dev/null
wait $ZED_PID 2>/dev/null

echo "✓ Test complete"
echo ""
echo "Logs saved to:"
echo "  /tmp/zed_launch.log - ZED camera output"
echo "  /tmp/aruco.log      - ArUco detector output"
echo "  /tmp/build.log      - Build output"
echo ""

