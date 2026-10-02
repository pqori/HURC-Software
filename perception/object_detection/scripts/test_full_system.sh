#!/bin/bash
################################################################################
# Object Detection Full System Test
#
# Comprehensive test: ZED camera + Object detection + monitoring
# Tests the complete ROS2 pipeline from camera to detected objects
#
# Usage:
#   ./test_full_system.sh [OPTIONS]
#
# Options:
#   --no-build          Skip rebuild of package
#   --zed-only          Test only ZED integration (not YOLO node)
#   --conf-thres VALUE  Confidence threshold (default: 0.5)
#   --help              Show this help
################################################################################

set -e

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# PIDs to track
ZED_PID=""
DETECTION_PID=""

# Cleanup function
cleanup() {
    echo ""
    echo -e "${YELLOW}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${YELLOW}Cleaning up processes...${NC}"
    echo -e "${YELLOW}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    
    if [ -n "$DETECTION_PID" ] && ps -p $DETECTION_PID > /dev/null 2>&1; then
        echo "  Killing detection node (PID: $DETECTION_PID)"
        kill $DETECTION_PID 2>/dev/null || true
    fi
    
    if [ -n "$ZED_PID" ] && ps -p $ZED_PID > /dev/null 2>&1; then
        echo "  Killing ZED camera (PID: $ZED_PID)"
        kill $ZED_PID 2>/dev/null || true
    fi
    
    # Extra cleanup for any stragglers
    pkill -f "object_detection.*--ros-args" 2>/dev/null || true
    pkill -f "zed2i_driver" 2>/dev/null || true
    
    sleep 1
    echo -e "${GREEN}✓ All processes stopped${NC}"
    exit 0
}

# Set up trap to catch Ctrl+C and exit signals
trap cleanup SIGINT SIGTERM EXIT

# Default options
NO_BUILD=false
ZED_ONLY=false
CONF_THRES="0.01"  # Default confidence threshold (model outputs very low probabilities)

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --no-build)
            NO_BUILD=true
            shift
            ;;
        --zed-only)
            ZED_ONLY=true
            shift
            ;;
        --conf-thres)
            CONF_THRES="$2"
            shift 2
            ;;
        --help)
            head -n 20 "$0" | grep "^#" | sed 's/^# //'
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            echo "Use --help for usage"
            exit 1
            ;;
    esac
done

cd /home/rover/workspaces/rover

echo -e "${BLUE}════════════════════════════════════════════════════════════════${NC}"
echo -e "${BLUE}           Object Detection Full System Test                    ${NC}"
echo -e "${BLUE}════════════════════════════════════════════════════════════════${NC}"
echo ""
echo "This will:"
echo "  1. Clean up old processes"
echo "  2. Launch ZED 2i camera"
if [ "$ZED_ONLY" = true ]; then
    echo "  3. Launch ZED integrated detection (native 3D)"
else
    echo "  3. Launch object detection node (YOLO + ONNX)"
fi
echo "  4. Monitor detections"
echo ""
echo "Make sure you have detectable objects visible!"
echo "  • URC objects: person, backpack, bottle, cup, chair"
echo -e "${BLUE}════════════════════════════════════════════════════════════════${NC}"
echo ""

# Clean up any existing processes
echo -e "${YELLOW}[1/6] Cleaning up old processes...${NC}"
pkill -f "standalone_object_detector" 2>/dev/null || true
pkill -f "zed2i" 2>/dev/null || true
pkill -f "object_detection" 2>/dev/null || true
pkill -f "zed_object" 2>/dev/null || true
pkill -f "zed_wrapper" 2>/dev/null || true
sleep 2
echo -e "${GREEN}✓ Cleanup complete${NC}"
echo ""

# Fix network config
echo -e "${YELLOW}[2/6] Configuring ROS2 environment...${NC}"
unset CYCLONEDDS_URI
export ROS_LOCALHOST_ONLY=1
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{name}]: {message}"
export RCUTILS_COLORIZED_OUTPUT=1
echo -e "${GREEN}✓ ROS2 environment configured${NC}"
echo ""

# Source workspace
source install/setup.bash

# Build if needed
if [ "$NO_BUILD" = false ]; then
    echo -e "${YELLOW}[3/6] Building object_detection package...${NC}"
    colcon build --packages-select object_detection --symlink-install > /tmp/build_od.log 2>&1
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}✓ Build successful${NC}"
        source install/setup.bash
    else
        echo -e "${RED}✗ Build failed, check /tmp/build_od.log${NC}"
        tail -20 /tmp/build_od.log
        exit 1
    fi
else
    echo -e "${YELLOW}[3/6] Skipping build (--no-build specified)${NC}"
fi
echo ""

# Launch ZED camera
echo -e "${BLUE}════════════════════════════════════════════════════════════════${NC}"
echo -e "${YELLOW}[4/6] Launching ZED camera...${NC}"
echo -e "${BLUE}════════════════════════════════════════════════════════════════${NC}"
echo ""

ros2 launch zed2i_launch zed2i_driver.launch.py > /tmp/zed_od_launch.log 2>&1 &
ZED_PID=$!

echo "  ZED camera PID: $ZED_PID"
echo "  Waiting 8 seconds for ZED initialization..."
sleep 8

# Check if still running
if ! ps -p $ZED_PID > /dev/null; then
    echo -e "${RED}✗ ZED launch failed! Check /tmp/zed_od_launch.log${NC}"
    tail -30 /tmp/zed_od_launch.log
    exit 1
fi

echo -e "${GREEN}✓ ZED camera running${NC}"

# Verify camera topics
echo ""
echo "  Checking camera topics..."
sleep 2  # Give more time for topics to appear

# Look for any image topic
IMAGE_TOPIC=$(ros2 topic list | grep -E "zed.*image.*color|zed.*rgb.*image" | head -1)

if [ -n "$IMAGE_TOPIC" ]; then
    echo -e "${GREEN}  ✓ Found image topic: $IMAGE_TOPIC${NC}"
    ACTUAL_IMAGE_TOPIC="$IMAGE_TOPIC"
else
    echo -e "${YELLOW}  ⚠ Looking for any ZED topics...${NC}"
    echo "  Available ZED topics:"
    ros2 topic list | grep zed2i
    
    # Try common variations
    for topic in "/zed2i/zed_node/left/image_rect_color" "/zed2i/left/image_rect_color" "/zed2i/rgb/image_rect_color" "/zed2i/zed2i_camera/left/image_rect_color"; do
        if ros2 topic list | grep -q "$topic"; then
            ACTUAL_IMAGE_TOPIC="$topic"
            echo -e "${GREEN}  ✓ Found: $ACTUAL_IMAGE_TOPIC${NC}"
            break
        fi
    done
    
    if [ -z "$ACTUAL_IMAGE_TOPIC" ]; then
        echo -e "${RED}  ✗ No image topic found!${NC}"
        echo ""
        echo "  Camera might need different launch parameters."
        echo "  Check: /tmp/zed_od_launch.log"
        read -p "Continue anyway? (y/n) " -n 1 -r
        echo
        if [[ ! $REPLY =~ ^[Yy]$ ]]; then
            kill $ZED_PID
            exit 1
        fi
        ACTUAL_IMAGE_TOPIC="/zed2i/zed2i_camera/left/image_rect_color"  # Use default
    fi
fi

echo ""

# Launch detection node
echo -e "${BLUE}════════════════════════════════════════════════════════════════${NC}"
if [ "$ZED_ONLY" = true ]; then
    echo -e "${YELLOW}[5/6] Launching ZED integrated detection...${NC}"
else
    echo -e "${YELLOW}[5/6] Launching object detection node...${NC}"
fi
echo -e "${BLUE}════════════════════════════════════════════════════════════════${NC}"
echo ""

if [ "$ZED_ONLY" = true ]; then
    # Launch ZED integrated detection (uses ZED wrapper's native detection)
    echo "  Mode: ZED Native Detection (GPU-accelerated)"
    ros2 launch object_detection zed_integrated_detection.launch.py 2>&1 | tee /tmp/zed_integrated.log &
    DETECTION_PID=$!
    DETECTION_TOPIC="/zed_detections_3d"
else
    # Launch YOLO detection node with verbose logging
    echo "  Mode: YOLO ONNX Detection (CPU)"
    echo "  Model: ALL 80 COCO classes (no filter)"
    echo "  Confidence: $CONF_THRES"
    echo "  Image topic: ${ACTUAL_IMAGE_TOPIC:-/zed2i/zed2i_camera/left/image_rect_color}"
    echo "  Logging to: /tmp/object_detection.log"
    
    # Use tee to show output AND save to log
    install/object_detection/bin/object_detection \
        --ros-args \
        -p model_path:=/home/rover/workspaces/rover/src/perception/object_detection/model/model.onnx \
        -p classes_file:=/home/rover/workspaces/rover/src/perception/object_detection/model/classes_coco.txt \
        -p urc_filter_file:="" \
        -p confidence_threshold:=$CONF_THRES \
        -p image_topic:=${ACTUAL_IMAGE_TOPIC:-/zed2i/zed2i_camera/left/image_rect_color} \
        -p output_topic:=/detected_objects \
        --log-level INFO \
        2>&1 | tee /tmp/object_detection.log &
    DETECTION_PID=$!
    DETECTION_TOPIC="/detected_objects"
fi

echo "  Detection PID: $DETECTION_PID"
echo "  Waiting 5 seconds for initialization..."
sleep 5

# Check if still running
if ! ps -p $DETECTION_PID > /dev/null; then
    echo -e "${RED}✗ Detection launch failed!${NC}"
    if [ "$ZED_ONLY" = true ]; then
        tail -30 /tmp/zed_integrated.log
    else
        tail -30 /tmp/object_detection.log
    fi
    kill $ZED_PID 2>/dev/null || true
    exit 1
fi

echo -e "${GREEN}✓ Detection node running${NC}"

# Verify detection topics
echo ""
echo "  Checking detection topics..."
sleep 2
if ros2 topic list | grep -q "$DETECTION_TOPIC"; then
    echo -e "${GREEN}  ✓ Detection topic available: $DETECTION_TOPIC${NC}"
    
    # Get topic info
    TOPIC_INFO=$(ros2 topic info $DETECTION_TOPIC 2>/dev/null)
    if echo "$TOPIC_INFO" | grep -q "Publisher count: 1"; then
        echo -e "${GREEN}  ✓ Detection node is publishing${NC}"
    else
        echo -e "${YELLOW}  ⚠ Publisher not found yet (may be initializing)${NC}"
    fi
else
    echo -e "${RED}  ✗ Detection topic not found!${NC}"
    echo "  Available topics:"
    ros2 topic list | grep -E "detect|object" || echo "  (none)"
fi

echo ""

# Monitor detections
echo -e "${BLUE}════════════════════════════════════════════════════════════════${NC}"
echo -e "${YELLOW}[6/6] Monitoring detections...${NC}"
echo -e "${BLUE}════════════════════════════════════════════════════════════════${NC}"
echo ""
echo -e "${GREEN}Press Ctrl+C to stop monitoring${NC}"
echo ""
echo "Watching for detections on: $DETECTION_TOPIC"
echo "Put detectable objects in camera view:"
echo "  • Person (astronaut/team member)"
echo "  • Backpack (equipment/tools)"
echo "  • Bottle, Cup (containers)"
echo "  • Chair (reference object)"
echo ""
echo -e "${BLUE}────────────────────────────────────────────────────────────────${NC}"

# Trap Ctrl+C
trap cleanup INT

cleanup() {
    echo ""
    echo ""
    echo -e "${BLUE}════════════════════════════════════════════════════════════════${NC}"
    echo -e "${YELLOW}Shutting down...${NC}"
    echo -e "${BLUE}════════════════════════════════════════════════════════════════${NC}"
    
    echo "  Stopping detection node (PID: $DETECTION_PID)..."
    kill $DETECTION_PID 2>/dev/null || true
    
    echo "  Stopping ZED camera (PID: $ZED_PID)..."
    kill $ZED_PID 2>/dev/null || true
    
    sleep 2
    
    # Force kill if needed
    pkill -9 -f "object_detection" 2>/dev/null || true
    pkill -9 -f "zed2i" 2>/dev/null || true
    
    echo ""
    echo -e "${GREEN}✓ Cleanup complete${NC}"
    echo ""
    
    # Show session summary
    echo -e "${BLUE}════════════════════════════════════════════════════════════════${NC}"
    echo -e "${BLUE}                    Session Summary                             ${NC}"
    echo -e "${BLUE}════════════════════════════════════════════════════════════════${NC}"
    echo ""
    
    if [ "$ZED_ONLY" = true ]; then
        echo "Logs available at:"
        echo "  • ZED camera: /tmp/zed_od_launch.log"
        echo "  • ZED detection: /tmp/zed_integrated.log"
    else
        echo "Logs available at:"
        echo "  • ZED camera: /tmp/zed_od_launch.log"
        echo "  • Object detection: /tmp/object_detection.log"
    fi
    
    echo ""
    echo "To view detection performance:"
    if [ "$ZED_ONLY" = true ]; then
        echo "  tail -f /tmp/zed_integrated.log | grep -E 'FPS|detect|Found'"
    else
        echo "  tail -f /tmp/object_detection.log | grep -E 'FPS|detect|Detection'"
    fi
    
    echo ""
    echo -e "${BLUE}════════════════════════════════════════════════════════════════${NC}"
    
    exit 0
}

# Run Python monitor (blocking call - keeps script alive until Ctrl+C)
echo ""
echo -e "${YELLOW}Starting detection monitor...${NC}"
echo "Hold objects in front of the camera"
echo "Press Ctrl+C when done"
echo ""

# Run the Python monitor - this blocks until Ctrl+C
python3 src/perception/object_detection/scripts/monitor_detections.py

# After monitor exits (Ctrl+C pressed or process ended)
echo ""

