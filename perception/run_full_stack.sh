#!/bin/bash
# Launch full perception stack

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}╔════════════════════════════════════════════════════════╗${NC}"
echo -e "${BLUE}║  LAUNCHING FULL PERCEPTION STACK                       ║${NC}"
echo -e "${BLUE}╚════════════════════════════════════════════════════════╝${NC}"
echo ""

# Cleanup
pkill -9 -f "ublox_gps|zed|gnss|robot_localization" 2>/dev/null
sleep 2

# Export paths
export LD_LIBRARY_PATH=/usr/local/zed/lib:/usr/local/cuda/lib64:$LD_LIBRARY_PATH

# 1. GPS
echo -e "${YELLOW}[1/4] Starting GPS...${NC}"
cd ~/workspaces/rover/src/perception/gnss_launch
./launch_gps.sh > /tmp/gps_stack.log 2>&1 &
sleep 5
echo -e "${GREEN}  ✓ GPS running${NC}"

# 2. ZED
echo -e "${YELLOW}[2/4] Starting ZED Camera (20 sec)...${NC}"
source /opt/ros/humble/setup.bash
source ~/workspaces/rover/install/setup.bash
ros2 launch zed2i_launch zed2i_driver.launch.py > /tmp/zed_stack.log 2>&1 &
sleep 20
echo -e "${GREEN}  ✓ ZED running${NC}"

# 3. Localization
echo -e "${YELLOW}[3/4] Starting Localization Fusion...${NC}"
source /opt/ros/humble/setup.bash
source ~/workspaces/rover/install/setup.bash

# Launch gnss_launch components first
cd ~/workspaces/rover/src/perception/gnss_launch/launch
python3 gnss_health_monitor.py &
python3 gnss_validator.py &

# Launch localization directly
ros2 run robot_localization ekf_node --ros-args \
    --params-file ~/workspaces/rover/src/perception/loc_fusion/config/ekf.yaml \
    -r __node:=ekf_local > /tmp/ekf_local.log 2>&1 &

ros2 run robot_localization navsat_transform_node --ros-args \
    --params-file ~/workspaces/rover/src/perception/loc_fusion/config/navsat_transform.yaml \
    -r imu:=/zed2i/zed2i_camera/imu/data \
    -r gps/fix:=/gps/fix \
    -r odom:=/odometry/filtered \
    -r filtered:=/odom/gps > /tmp/navsat.log 2>&1 &

ros2 run robot_localization ekf_node --ros-args \
    --params-file ~/workspaces/rover/src/perception/loc_fusion/config/ekf.yaml \
    -r __node:=ekf_global \
    -r odometry/filtered:=/odometry/global > /tmp/ekf_global.log 2>&1 &

sleep 5
echo -e "${GREEN}  ✓ Localization running${NC}"

# 4. Pointcloud Tools for Nav2
echo -e "${YELLOW}[4/4] Starting Point Cloud Tools...${NC}"
python3 ~/workspaces/rover/src/perception/pointcloud_tools/pointcloud_tools/depth_to_scan.py > /tmp/depth_to_scan.log 2>&1 &
python3 ~/workspaces/rover/src/perception/pointcloud_tools/pointcloud_tools/grid_builder.py > /tmp/grid_builder.log 2>&1 &
echo -e "${GREEN}  ✓ depth_to_scan running (/scan topic)${NC}"
echo -e "${GREEN}  ✓ grid_builder running (/local_map topic)${NC}"

sleep 2

echo ""
echo -e "${BLUE}══════════════════════════════════════════════════════${NC}"
echo ""
echo -e "${GREEN}✅ ALL COMPONENTS RUNNING!${NC}"
echo ""
echo "Check status:"
echo "  ros2 node list"
echo "  ros2 topic list | grep -E '(gps|zed2i|odometry)'"
echo ""
echo "Test data:"
echo "  ros2 topic hz /gps/fix"
echo "  ros2 topic hz /zed2i/zed2i_camera/odom"
echo "  ros2 topic hz /odometry/filtered"
echo ""
echo "Logs:"
echo "  tail -f /tmp/gps_stack.log"
echo "  tail -f /tmp/zed_stack.log"
echo "  tail -f /tmp/ekf_local.log"
echo ""

