#!/bin/bash
# ZED 2i Camera Topic Verification Script
# Usage: 
#   ./test_zed2i.sh           - Launch camera and test (full test)
#   ./test_zed2i.sh --no-launch - Test only (camera must be running)

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Parse command line arguments
LAUNCH_CAMERA=true
if [ "$1" = "--no-launch" ]; then
    LAUNCH_CAMERA=false
fi

if [ "$LAUNCH_CAMERA" = true ]; then
    echo -e "${BLUE}🎥 ZED 2i Launch and Complete Topic Verification${NC}"
else
    echo -e "${BLUE}🎥 ZED 2i Topic Verification (Camera Pre-launched)${NC}"
fi
echo "============================================================"

# Change to workspace directory
cd /home/rover/workspaces/rover-real

# Source workspace
echo -e "${BLUE}🔧 Sourcing workspace...${NC}"
source /opt/ros/humble/setup.bash
source install/setup.bash

# Launch camera if requested
if [ "$LAUNCH_CAMERA" = true ]; then
    # Launch ZED camera in background
    echo -e "${BLUE}🚀 Launching ZED 2i camera...${NC}"
    ros2 launch zed2i_launch zed2i_driver.launch.py &
    LAUNCH_PID=$!

    # Function to cleanup on exit
    cleanup() {
        echo -e "\n${YELLOW}🧹 Cleaning up...${NC}"
        kill $LAUNCH_PID 2>/dev/null
        wait $LAUNCH_PID 2>/dev/null
        echo -e "${GREEN}✅ Cleanup complete${NC}"
    }
    trap cleanup EXIT
    
    # Wait for camera to initialize
    echo -e "${BLUE}⏳ Waiting for camera to fully initialize (30 seconds)...${NC}"
    sleep 30
else
    echo -e "${BLUE}📡 Using pre-launched camera...${NC}"
    # Check if camera is already running
    echo -e "${BLUE}🔍 Checking for ZED camera topics...${NC}"
    ZED_TOPICS=$(ros2 topic list | grep -c "zed2i" || echo "0")
    if [ "$ZED_TOPICS" -eq 0 ]; then
        echo -e "${RED}❌ No ZED camera topics found! Please launch the camera first:${NC}"
        echo -e "${YELLOW}   ros2 launch zed2i_launch zed2i_driver.launch.py${NC}"
        exit 1
    fi
    echo -e "${GREEN}✅ ZED camera topics detected (${ZED_TOPICS} topics found)${NC}"
    
    # Give camera a moment to stabilize
    echo -e "${BLUE}⏳ Giving camera a moment to stabilize (5 seconds)...${NC}"
    sleep 5
fi

# Quick check if topics are publishing data
echo -e "${BLUE}⏳ Quick check for data publication...${NC}"
if timeout 3 ros2 topic echo /zed2i/zed2i_camera/left/image_rect_color --once >/dev/null 2>&1; then
    echo -e "${GREEN}✅ Topics are publishing data${NC}"
elif timeout 3 ros2 topic echo /zed2i/left/image_rect_color --once >/dev/null 2>&1; then
    echo -e "${GREEN}✅ Topics are publishing data (remapped)${NC}"
else
    echo -e "${YELLOW}⚠️  Will test topics anyway (may be low frequency)${NC}"
fi

# Check if launch is still running (only if we launched it)
if [ "$LAUNCH_CAMERA" = true ]; then
    if ! kill -0 $LAUNCH_PID 2>/dev/null; then
        echo -e "${RED}❌ Camera launch failed!${NC}"
        exit 1
    fi
fi

echo -e "${GREEN}✅ Camera is ready for testing${NC}"

# Define all ACTUAL ZED topics (based on real camera output)
ALL_TOPICS=(
    # Core image topics (full namespace from terminal logs)
    "/zed2i/zed2i_camera/left/image_rect_color"
    "/zed2i/zed2i_camera/left/camera_info"
    "/zed2i/zed2i_camera/right/image_rect_color"
    "/zed2i/zed2i_camera/right/camera_info"
    "/zed2i/zed2i_camera/rgb/image_rect_color"
    "/zed2i/zed2i_camera/rgb/camera_info"
    
    # Depth and point cloud
    "/zed2i/zed2i_camera/depth/depth_registered"
    "/zed2i/zed2i_camera/depth/camera_info"
    "/zed2i/zed2i_camera/confidence/confidence_map"
    "/zed2i/zed2i_camera/point_cloud/cloud_registered"
    
    # IMU and sensors
    "/zed2i/zed2i_camera/imu/data"
    "/zed2i/zed2i_camera/imu/data_raw"
    "/zed2i/zed2i_camera/imu/mag"
    "/zed2i/zed2i_camera/temperature/imu"
    
    # Odometry and pose
    "/zed2i/zed2i_camera/odom"
    "/zed2i/zed2i_camera/pose"
    "/zed2i/zed2i_camera/pose_with_covariance"
    
    # Additional topics from logs
    "/zed2i/zed2i_camera/atm_press"
    "/zed2i/zed2i_camera/temperature/left"
    "/zed2i/zed2i_camera/temperature/right"
)

echo -e "\n${BLUE}📋 Step 1: Checking topic existence...${NC}"
EXISTING_TOPICS=0

# Get topic list once to avoid repeated calls
echo -e "${BLUE}  Getting topic list...${NC}"
TOPIC_LIST=$(ros2 topic list 2>/dev/null)
if [ $? -ne 0 ] || [ -z "$TOPIC_LIST" ]; then
    echo -e "${RED}❌ Failed to get topic list. Is ROS 2 running?${NC}"
    exit 1
fi

for topic in "${ALL_TOPICS[@]}"; do
    if echo "$TOPIC_LIST" | grep -q "^${topic}$"; then
        echo -e "${GREEN}  ✅ ${topic}${NC}"
        ((EXISTING_TOPICS++))
    else
        echo -e "${RED}  ❌ ${topic}${NC}"
    fi
done

echo -e "\n${BLUE}📊 Step 2: Testing data publication...${NC}"
PUBLISHING_TOPICS=0
TOTAL_EXISTING=${#ALL_TOPICS[@]}

for i in "${!ALL_TOPICS[@]}"; do
    topic="${ALL_TOPICS[$i]}"
    progress=$((i + 1))
    
    # Only test topics that exist
    if echo "$TOPIC_LIST" | grep -q "^${topic}$"; then
        echo -e "${BLUE}  Testing (${progress}/${TOTAL_EXISTING}): ${topic}${NC}"
        
        # Use longer timeout for data publication test
        if timeout 5 ros2 topic echo "$topic" --once >/dev/null 2>&1; then
            echo -e "${GREEN}    ✅ Publishing data${NC}"
            ((PUBLISHING_TOPICS++))
        else
            echo -e "${YELLOW}    ⚠️  No data received (may be low frequency or disabled)${NC}"
        fi
    else
        echo -e "${BLUE}  Skipping (${progress}/${TOTAL_EXISTING}): ${topic} (not found)${NC}"
    fi
done

echo -e "\n${BLUE}📈 Step 3: Measuring frequencies for key topics...${NC}"
KEY_TOPICS=(
    "/zed2i/zed2i_camera/left/image_rect_color"
    "/zed2i/zed2i_camera/imu/data"
    "/zed2i/zed2i_camera/odom"
)

for topic in "${KEY_TOPICS[@]}"; do
    if echo "$TOPIC_LIST" | grep -q "^${topic}$"; then
        echo -e "${BLUE}  ${topic}:${NC}"
        timeout 5 ros2 topic hz "$topic" 2>/dev/null | head -1 || echo -e "${RED}    ❌ No frequency data${NC}"
    else
        echo -e "${RED}  ${topic}: Topic not found${NC}"
    fi
done

# Final Summary
echo -e "\n${BLUE}📊 FINAL SUMMARY${NC}"
echo "============================================================"
echo -e "${GREEN}📋 Topics found:     ${EXISTING_TOPICS}/${TOTAL_EXISTING}${NC}"
echo -e "${GREEN}📡 Topics publishing: ${PUBLISHING_TOPICS}/${EXISTING_TOPICS}${NC}"

if [ $EXISTING_TOPICS -eq $TOTAL_EXISTING ] && [ $PUBLISHING_TOPICS -gt $((TOTAL_EXISTING / 2)) ]; then
    echo -e "${GREEN}✅ SUCCESS: ZED 2i camera is working properly!${NC}"
    EXIT_CODE=0
elif [ $EXISTING_TOPICS -gt $((TOTAL_EXISTING / 2)) ]; then
    echo -e "${YELLOW}⚠️  PARTIAL: Most topics exist, some may not be publishing${NC}"
    EXIT_CODE=0
else
    echo -e "${RED}❌ ISSUES: Many topics missing or not publishing${NC}"
    EXIT_CODE=1
fi

echo -e "\n${BLUE}💡 Manual verification commands:${NC}"
echo "  ros2 topic list | grep zed2i"
echo "  ros2 topic hz /zed2i/left/image_rect_color"
echo "  ros2 topic echo /zed2i/imu/data --once"

if [ "$LAUNCH_CAMERA" = true ]; then
    echo -e "\n${BLUE}🔍 Camera is still running for manual inspection...${NC}"
    echo -e "${YELLOW}Press Ctrl+C to stop${NC}"
    
    # Keep the script running so user can manually inspect
    wait $LAUNCH_PID
else
    echo -e "\n${GREEN}✅ Test completed${NC}"
fi

exit $EXIT_CODE