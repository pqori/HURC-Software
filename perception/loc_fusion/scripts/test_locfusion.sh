#!/bin/bash
################################################################################
# ZED Localization Fusion Test
#
# Runs the standalone localization + point-cloud capture tool.
# Produces fused poses (ZED VIO + IMU) and optional outputs:
#   - PLY point cloud
#   - Interleaved VBO buffer
#   - 2D occupancy PNG slice
#
# Usage:
#   ./test_locfusion.sh [OPTIONS]
#
# Options:
#   --svo FILE          Use SVO file instead of live camera
#   --frames N          Process N frames then exit (0 = run continuously)
#   --save-ply FILE     Save a point cloud PLY
#   --save-vbo FILE     Save a raw interleaved VBO (.bin)
#   --save-occ FILE     Save a 2D occupancy map (PNG)
#   --ros-enu           Convert to ROS ENU frame
#   --help              Show this help
################################################################################

set -e

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

LOC_PID=""

cleanup() {
    echo ""
    echo -e "${YELLOW}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${YELLOW}Cleaning up localization fusion...${NC}"
    echo -e "${YELLOW}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"

    if [ -n "$LOC_PID" ] && ps -p $LOC_PID > /dev/null 2>&1; then
        echo "  Killing process (PID: $LOC_PID)"
        kill $LOC_PID 2>/dev/null || true
    fi

    pkill -f "standalone_locfusion" 2>/dev/null || true
    sleep 1
    echo -e "${GREEN}✓ Localization fusion stopped${NC}"
    exit 0
}

trap cleanup SIGINT SIGTERM EXIT

# Default options
SVO_FILE=""
FRAMES=0
SAVE_PLY=""
SAVE_VBO=""
SAVE_OCC=""
ROS_ENU=""
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --svo)       SVO_FILE="$2"; shift 2 ;;
        --frames)    FRAMES="$2"; shift 2 ;;
        --save-ply)  SAVE_PLY="$2"; shift 2 ;;
        --save-vbo)  SAVE_VBO="$2"; shift 2 ;;
        --save-occ)  SAVE_OCC="$2"; shift 2 ;;
        --ros-enu)   ROS_ENU="--ros-enu"; shift ;;
        --help)
            head -n 20 "$0" | grep "^#" | sed 's/^# //'
            exit 0 ;;
        *) echo "Unknown option: $1"; exit 1 ;;
    esac
done

echo -e "${BLUE}════════════════════════════════════════════════════════════════${NC}"
echo -e "${BLUE}       ZED Localization Fusion Test Runner                      ${NC}"
echo -e "${BLUE}════════════════════════════════════════════════════════════════${NC}"
echo ""

echo -e "${YELLOW}[1/4] Checking dependencies...${NC}"
MISSING=()

if ! command -v python3 &> /dev/null; then MISSING+=("python3"); fi
if ! python3 -c "import pyzed.sl" 2>/dev/null; then MISSING+=("pyzed (ZED SDK)"); fi
if ! python3 -c "import numpy" 2>/dev/null; then MISSING+=("numpy"); fi
if ! python3 -c "import cv2" 2>/dev/null; then MISSING+=("opencv-python"); fi

if [ ${#MISSING[@]} -ne 0 ]; then
    echo -e "${RED}✗ Missing dependencies:${NC}"
    for dep in "${MISSING[@]}"; do echo "  - $dep"; done
    exit 1
fi
echo -e "${GREEN}✓ All Python dependencies found${NC}"
echo ""

echo -e "${YELLOW}[2/4] Validating inputs...${NC}"
if [ -n "$SVO_FILE" ] && [ ! -f "$SVO_FILE" ]; then
    echo -e "${RED}✗ SVO file not found: $SVO_FILE${NC}"
    exit 1
fi
echo -e "${GREEN}✓ Inputs verified${NC}"
echo ""

echo -e "${YELLOW}[3/4] Preparing to launch...${NC}"

CMD="python3 $SCRIPT_DIR/standalone_locfusion.py"
[ -n "$SVO_FILE" ] && CMD=\"$CMD --svo $SVO_FILE\"
[ "$FRAMES" -gt 0 ] && CMD=\"$CMD --frames $FRAMES\"
[ -n "$SAVE_PLY" ] && CMD=\"$CMD --save-ply $SAVE_PLY\"
[ -n "$SAVE_VBO" ] && CMD=\"$CMD --save-vbo $SAVE_VBO\"
[ -n "$SAVE_OCC" ] && CMD=\"$CMD --save-occ $SAVE_OCC\"
[ -n "$ROS_ENU" ] && CMD=\"$CMD $ROS_ENU\"

echo -e \"${BLUE}════════════════════════════════════════════════════════════════${NC}\"
echo -e \"${GREEN}🚀 Launching ZED Localization Fusion${NC}\"
echo -e \"${BLUE}════════════════════════════════════════════════════════════════${NC}\"
echo \"\"
echo \"Input: ${SVO_FILE:-Live ZED camera}\"
echo \"Frames: ${FRAMES}\"
[ -n \"$SAVE_PLY\" ] && echo \"Save PLY: $SAVE_PLY\"
[ -n \"$SAVE_VBO\" ] && echo \"Save VBO: $SAVE_VBO\"
[ -n \"$SAVE_OCC\" ] && echo \"Save OCC: $SAVE_OCC\"
[ -n \"$ROS_ENU\" ] && echo \"Frame: ROS ENU\"
echo \"\"

# Run
$CMD &
LOC_PID=$!

wait $LOC_PID
EXIT_CODE=$?

echo \"\"
echo -e \"${BLUE}════════════════════════════════════════════════════════════════${NC}\"
if [ $EXIT_CODE -eq 0 ]; then
    echo -e \"${GREEN}✓ Localization test completed successfully${NC}\"
else
    echo -e \"${RED}✗ Localization test failed (exit code: $EXIT_CODE)${NC}\"
fi
echo -e \"${BLUE}════════════════════════════════════════════════════════════════${NC}\"
echo \"\"

exit $EXIT_CODE
