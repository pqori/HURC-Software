#!/bin/bash
################################################################################
# Standalone Object Detection Test
#
# Tests object detection with 3D visualization (OpenGL + CV viewers)
# Similar to ArUco's standalone detector test.
#
# Usage:
#   ./test_standalone.sh [OPTIONS]
#
# Options:
#   --svo FILE          Use SVO file instead of live camera
#   --conf-thres VALUE  Confidence threshold (default: 0.5)
#   --no-gl             Disable 3D OpenGL viewer (2D only)
#   --help              Show this help
#
# Controls (during visualization):
#   Q/ESC: Quit
#   I: Zoom in (birds-eye view)
#   O: Zoom out (birds-eye view)
################################################################################

set -e

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# PID tracking
DETECTOR_PID=""

# Cleanup function
cleanup() {
    echo ""
    echo -e "${YELLOW}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${YELLOW}Cleaning up standalone detector...${NC}"
    echo -e "${YELLOW}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    
    if [ -n "$DETECTOR_PID" ] && ps -p $DETECTOR_PID > /dev/null 2>&1; then
        echo "  Killing detector (PID: $DETECTOR_PID)"
        kill $DETECTOR_PID 2>/dev/null || true
    fi
    
    # Extra cleanup
    pkill -f "standalone_object_detector" 2>/dev/null || true
    
    sleep 1
    echo -e "${GREEN}✓ Detector stopped${NC}"
    exit 0
}

# Set up trap to catch Ctrl+C and exit signals
trap cleanup SIGINT SIGTERM EXIT

# Default options
SVO_FILE=""
CONF_THRES="0.05"  # Match full system test (chairs detect at 1-6% confidence)
NO_GL=""
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_ROOT="$(dirname "$SCRIPT_DIR")"

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --svo)
            SVO_FILE="$2"
            shift 2
            ;;
        --conf-thres)
            CONF_THRES="$2"
            shift 2
            ;;
        --no-gl)
            NO_GL="--no-gl"
            shift
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

echo -e "${BLUE}════════════════════════════════════════════════════════════════${NC}"
echo -e "${BLUE}     Object Detection - Standalone Visualization Test          ${NC}"
echo -e "${BLUE}════════════════════════════════════════════════════════════════${NC}"
echo ""

# Clean up any existing processes
echo -e "${YELLOW}[0/5] Cleaning up old processes...${NC}"
pkill -f "standalone_object_detector" 2>/dev/null || true
pkill -f "zed2i" 2>/dev/null || true
pkill -f "object_detection" 2>/dev/null || true
sleep 2
echo -e "${GREEN}✓ Cleanup complete${NC}"
echo ""

# Check dependencies
echo -e "${YELLOW}[1/5] Checking dependencies...${NC}"

MISSING_DEPS=()

if ! command -v python3 &> /dev/null; then
    MISSING_DEPS+=("python3")
fi

if ! python3 -c "import pyzed.sl" 2>/dev/null; then
    MISSING_DEPS+=("pyzed (ZED SDK)")
fi

if ! python3 -c "import onnxruntime" 2>/dev/null; then
    MISSING_DEPS+=("onnxruntime")
fi

if ! python3 -c "import cv2" 2>/dev/null; then
    MISSING_DEPS+=("opencv-python")
fi

if ! python3 -c "import numpy" 2>/dev/null; then
    MISSING_DEPS+=("numpy")
fi

if [ ${#MISSING_DEPS[@]} -ne 0 ]; then
    echo -e "${RED}✗ Missing dependencies:${NC}"
    for dep in "${MISSING_DEPS[@]}"; do
        echo -e "  - $dep"
    done
    exit 1
fi

echo -e "${GREEN}✓ All dependencies found${NC}"
echo ""

# Check model files
echo -e "${YELLOW}[2/5] Checking model files...${NC}"

if [ ! -f "$PKG_ROOT/model/model.onnx" ]; then
    echo -e "${RED}✗ Model file not found: $PKG_ROOT/model/model.onnx${NC}"
    exit 1
fi

if [ ! -f "$PKG_ROOT/model/classes_coco.txt" ]; then
    echo -e "${RED}✗ Classes file not found: $PKG_ROOT/model/classes_coco.txt${NC}"
    exit 1
fi

echo -e "${GREEN}✓ Model files found${NC}"
MODEL_SIZE=$(du -h "$PKG_ROOT/model/model.onnx" | cut -f1)
echo "  - Model: model.onnx ($MODEL_SIZE)"
echo "  - Classes: ALL 80 COCO objects (no filter)"
echo ""

# Check viewers
echo -e "${YELLOW}[3/5] Checking visualization modules...${NC}"

if [ ! -d "$SCRIPT_DIR/cv_viewer" ] || [ ! -d "$SCRIPT_DIR/ogl_viewer" ]; then
    echo -e "${RED}✗ Viewer modules not found${NC}"
    exit 1
fi

if [ -z "$NO_GL" ]; then
    # Check OpenGL availability
    if ! python3 -c "from OpenGL.GL import *" 2>/dev/null; then
        echo -e "${YELLOW}⚠ OpenGL not available - will use 2D viewer only${NC}"
        NO_GL="--no-gl"
    else
        echo -e "${GREEN}✓ OpenGL viewer available${NC}"
    fi
fi

echo -e "${GREEN}✓ Visualization modules ready${NC}"
echo ""

# Check and set display
echo -e "${YELLOW}[4/5] Configuring display...${NC}"

# Always set DISPLAY for Jetson monitor
export DISPLAY=:0

echo -e "${GREEN}✓ Display configured: $DISPLAY${NC}"
echo ""

# Build command
echo -e "${YELLOW}[5/5] Preparing to launch...${NC}"

CMD="python3 $SCRIPT_DIR/standalone_continuous_object_detector.py"
CMD="$CMD --onnx $PKG_ROOT/model/model.onnx"
CMD="$CMD --classes $PKG_ROOT/model/classes_coco.txt"
# CMD="$CMD --urc-filter $PKG_ROOT/config/urc_classes.txt"  # DISABLED - Show all 80 COCO objects
CMD="$CMD --conf-thres $CONF_THRES"

if [ -n "$SVO_FILE" ]; then
    if [ ! -f "$SVO_FILE" ]; then
        echo -e "${RED}✗ SVO file not found: $SVO_FILE${NC}"
        exit 1
    fi
    CMD="$CMD --svo $SVO_FILE"
    echo "  Input: SVO file ($SVO_FILE)"
else
    echo "  Input: Live ZED camera"
fi

if [ -n "$NO_GL" ]; then
    CMD="$CMD --no-gl"
    echo "  Viewer: 2D OpenCV only"
else
    echo "  Viewer: 3D OpenGL + Birds-eye view"
fi

echo "  Confidence: $CONF_THRES"
echo ""

# Run detection
echo -e "${BLUE}════════════════════════════════════════════════════════════════${NC}"
echo -e "${GREEN}🚀 Launching Object Detection Visualization${NC}"
echo -e "${BLUE}════════════════════════════════════════════════════════════════${NC}"
echo ""
echo -e "${YELLOW}Controls:${NC}"
echo "  Q or ESC: Quit"
echo "  I: Zoom in (birds-eye view)"
echo "  O: Zoom out (birds-eye view)"
echo ""
echo -e "${BLUE}════════════════════════════════════════════════════════════════${NC}"
echo ""

# Execute and track PID
$CMD &
DETECTOR_PID=$!

# Wait for detector to finish
wait $DETECTOR_PID

EXIT_CODE=$?

echo ""
echo -e "${BLUE}════════════════════════════════════════════════════════════════${NC}"

if [ $EXIT_CODE -eq 0 ]; then
    echo -e "${GREEN}✓ Visualization test completed successfully${NC}"
else
    echo -e "${RED}✗ Visualization test failed (exit code: $EXIT_CODE)${NC}"
fi

echo -e "${BLUE}════════════════════════════════════════════════════════════════${NC}"
echo ""

exit $EXIT_CODE

