#!/bin/bash
# Test full perception → localization → navigation pipeline

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}╔════════════════════════════════════════════════════════╗${NC}"
echo -e "${BLUE}║  FULL NAVIGATION PIPELINE TEST                         ║${NC}"
echo -e "${BLUE}╚════════════════════════════════════════════════════════╝${NC}"
echo ""

# Source workspaces
source /opt/ros/jazzy/setup.bash 2>/dev/null
source ~/workspaces/ros2-ublox-zedf9p/install/setup.bash 2>/dev/null
source ~/workspaces/rover/install/setup.bash 2>/dev/null

# Kill any existing processes
echo "🧹 Cleaning up old processes..."
pkill -9 -f "ublox_gps" 2>/dev/null || true
pkill -9 -f "zed" 2>/dev/null || true
pkill -9 -f "gnss" 2>/dev/null || true
pkill -9 -f "robot_localization" 2>/dev/null || true
sleep 2

# ============================================================================
# STEP 1: Launch GPS
# ============================================================================
echo -e "${BLUE}[1/4] Launching GPS...${NC}"
cd ~/workspaces/rover/src/perception/gnss_launch
./launch_gps.sh > /tmp/gps.log 2>&1 &
GPS_PID=$!
sleep 5

# Check GPS topics
echo "  Checking GPS topics..."
if ros2 topic list 2>/dev/null | grep -q "/gps/fix"; then
    echo -e "${GREEN}  ✓ /gps/fix${NC}"
else
    echo -e "${RED}  ✗ /gps/fix NOT FOUND${NC}"
    exit 1
fi

if ros2 topic list 2>/dev/null | grep -q "/gnss/health_status"; then
    echo -e "${GREEN}  ✓ /gnss/health_status${NC}"
else
    echo -e "${YELLOW}  ⚠ /gnss/health_status${NC}"
fi
echo ""

# ============================================================================
# STEP 2: Launch ZED Camera
# ============================================================================
echo -e "${BLUE}[2/4] Launching ZED Camera (20 seconds to initialize)...${NC}"
ros2 launch zed2i_launch zed2i_driver.launch.py > /tmp/zed.log 2>&1 &
ZED_PID=$!
sleep 20

# Check ZED topics
echo "  Checking ZED topics..."
ZED_TOPICS=(
    "/zed2i/zed2i_camera/odom"
    "/zed2i/zed2i_camera/imu/data"
    "/zed2i/zed2i_camera/left/image_rect_color"
    "/zed2i/zed2i_camera/depth/depth_registered"
    "/zed2i/zed2i_camera/point_cloud/cloud_registered"
)

ZED_OK=true
for topic in "${ZED_TOPICS[@]}"; do
    if ros2 topic list 2>/dev/null | grep -q "^$topic$"; then
        echo -e "${GREEN}  ✓ $topic${NC}"
    else
        echo -e "${RED}  ✗ $topic${NC}"
        ZED_OK=false
    fi
done

if [ "$ZED_OK" = false ]; then
    echo -e "${RED}ZED camera not fully initialized${NC}"
    echo "Check log: tail /tmp/zed.log"
    exit 1
fi
echo ""

# ============================================================================
# STEP 3: Check Data Flow
# ============================================================================
echo -e "${BLUE}[3/4] Verifying Data Flow...${NC}"

echo "  Testing GPS data rate..."
timeout 3 ros2 topic hz /gps/fix 2>&1 | grep "average rate" && \
    echo -e "${GREEN}  ✓ GPS publishing${NC}" || \
    echo -e "${YELLOW}  ⚠ GPS no data (expected indoors)${NC}"

echo "  Testing ZED odometry rate..."
timeout 3 ros2 topic hz /zed2i/zed2i_camera/odom 2>&1 | grep "average rate" > /dev/null && \
    echo -e "${GREEN}  ✓ ZED odometry publishing${NC}" || \
    echo -e "${RED}  ✗ ZED odometry not publishing${NC}"

echo "  Testing ZED IMU rate..."
timeout 3 ros2 topic hz /zed2i/zed2i_camera/imu/data 2>&1 | grep "average rate" > /dev/null && \
    echo -e "${GREEN}  ✓ ZED IMU publishing${NC}" || \
    echo -e "${RED}  ✗ ZED IMU not publishing${NC}"

echo "  Testing ZED point cloud rate..."
timeout 3 ros2 topic hz /zed2i/zed2i_camera/point_cloud/cloud_registered 2>&1 | grep "average rate" > /dev/null && \
    echo -e "${GREEN}  ✓ ZED point cloud publishing${NC}" || \
    echo -e "${RED}  ✗ ZED point cloud not publishing${NC}"

echo ""

# ============================================================================
# STEP 4: Launch Localization Fusion
# ============================================================================
echo -e "${BLUE}[4/4] Testing Localization Fusion...${NC}"
ros2 launch loc_fusion loc_fusion.launch.py > /tmp/loc_fusion.log 2>&1 &
LOC_PID=$!
sleep 8

# Check localization topics
echo "  Checking localization topics..."
LOC_TOPICS=(
    "/odometry/filtered"
    "/odometry/global"
    "/odom/gps"
)

for topic in "${LOC_TOPICS[@]}"; do
    if ros2 topic list 2>/dev/null | grep -q "^$topic$"; then
        echo -e "${GREEN}  ✓ $topic${NC}"
    else
        echo -e "${YELLOW}  ⚠ $topic (may need GPS fix)${NC}"
    fi
done

echo ""
echo "  Testing localization data flow..."
timeout 3 ros2 topic hz /odometry/filtered 2>&1 | grep "average rate" > /dev/null && \
    echo -e "${GREEN}  ✓ Local odometry publishing${NC}" || \
    echo -e "${RED}  ✗ Local odometry not publishing${NC}"

echo ""

# ============================================================================
# SUMMARY
# ============================================================================
echo -e "${BLUE}╔════════════════════════════════════════════════════════╗${NC}"
echo -e "${BLUE}║  TOPIC MAP FOR NAVIGATION                              ║${NC}"
echo -e "${BLUE}╚════════════════════════════════════════════════════════╝${NC}"
echo ""
echo -e "${YELLOW}INPUTS (Sensors):${NC}"
echo "  • /gps/fix                                  → GPS position"
echo "  • /zed2i/zed2i_camera/odom                  → Visual odometry"
echo "  • /zed2i/zed2i_camera/imu/data              → IMU data"
echo "  • /zed2i/zed2i_camera/point_cloud/cloud_registered → 3D map"
echo "  • /zed2i/zed2i_camera/depth/depth_registered → Depth for obstacles"
echo ""
echo -e "${YELLOW}LOCALIZATION (robot_localization):${NC}"
echo "  • /odometry/filtered                        → Smooth local odom (ZED+IMU)"
echo "  • /odom/gps                                 → GPS in odom frame"
echo "  • /odometry/global                          → GPS-corrected global odom"
echo ""
echo -e "${YELLOW}FOR NAV2 (Navigation):${NC}"
echo "  • /odometry/filtered                        → Local planner input"
echo "  • /odometry/global                          → Global planner input"
echo "  • /scan                                     → LaserScan (from depth_to_scan)"
echo "  • /local_map                                → Local costmap (from grid_builder)"
echo ""
echo -e "${YELLOW}MISSING TOPICS (need pointcloud_tools):${NC}"
if ros2 topic list 2>/dev/null | grep -q "/scan"; then
    echo -e "${GREEN}  ✓ /scan${NC}"
else
    echo -e "${RED}  ✗ /scan - Run: ros2 launch pointcloud_tools pointcloud_tools.launch.py${NC}"
fi

if ros2 topic list 2>/dev/null | grep -q "/local_map"; then
    echo -e "${GREEN}  ✓ /local_map${NC}"
else
    echo -e "${RED}  ✗ /local_map - Run: ros2 launch pointcloud_tools pointcloud_tools.launch.py${NC}"
fi

echo ""
echo -e "${BLUE}════════════════════════════════════════════════════════${NC}"
echo ""
echo -e "${GREEN}All processes are running in background.${NC}"
echo ""
echo "To stop everything:"
echo "  pkill -f 'ublox_gps|zed|gnss|robot_localization'"
echo ""
echo "To launch full perception stack:"
echo "  ./launch_perception.sh"
echo ""
echo "Logs:"
echo "  GPS:           tail -f /tmp/gps.log"
echo "  ZED:           tail -f /tmp/zed.log"
echo "  Localization:  tail -f /tmp/loc_fusion.log"

