#!/bin/bash
################################################################################
# Download Better YOLOv8 Model
#
# Your current model gives 1-6% confidence (too low!)
# This script downloads official YOLOv8 models with 20-80% confidence
################################################################################

set -e

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

cd "$(dirname "$0")/.."

echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${BLUE}         Download Better YOLOv8 Model for URC                  ${NC}"
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo ""
echo "Your current model: 1-6% confidence (too low!)"
echo "New model will have: 20-80% confidence"
echo ""

# Check ultralytics
echo -e "${YELLOW}[1/5] Checking ultralytics...${NC}"
if ! python3 -c "import ultralytics" 2>/dev/null; then
    echo "Installing ultralytics..."
    pip3 install -q ultralytics
    echo -e "${GREEN}✓ Ultralytics installed${NC}"
else
    echo -e "${GREEN}✓ Ultralytics already installed${NC}"
fi
echo ""

# Choose model
echo -e "${YELLOW}[2/5] Choose model:${NC}"
echo "  1) YOLOv8n (Nano)   - Fastest, ~40 FPS, 37.3 mAP"
echo "  2) YOLOv8s (Small)  - Balanced, ~25 FPS, 44.9 mAP"
echo "  3) YOLOv8m (Medium) - Best accuracy, ~15 FPS, 50.2 mAP"
echo ""
read -p "Enter choice [1-3] (default: 1): " choice
choice=${choice:-1}

case $choice in
    1) MODEL="yolov8n" ;;
    2) MODEL="yolov8s" ;;
    3) MODEL="yolov8m" ;;
    *) echo -e "${RED}Invalid choice${NC}"; exit 1 ;;
esac

echo -e "${GREEN}✓ Selected: $MODEL${NC}"
echo ""

# Download and export
echo -e "${YELLOW}[3/5] Downloading and exporting $MODEL to ONNX...${NC}"
echo "This may take 2-5 minutes..."
python3 << PYTHON
from ultralytics import YOLO
import sys

try:
    print(f"  Downloading ${MODEL}.pt...")
    model = YOLO('${MODEL}.pt')
    
    print(f"  Exporting to ONNX (640x640)...")
    model.export(format='onnx', imgsz=640, simplify=True, opset=12)
    
    print("  ✓ Export complete")
except Exception as e:
    print(f"  ✗ Error: {e}")
    sys.exit(1)
PYTHON

if [ $? -ne 0 ]; then
    echo -e "${RED}✗ Export failed${NC}"
    exit 1
fi

echo -e "${GREEN}✓ Model exported${NC}"
echo ""

# Backup old model
echo -e "${YELLOW}[4/5] Backing up old model...${NC}"
if [ -f "model/model.onnx" ]; then
    TIMESTAMP=$(date +%Y%m%d_%H%M%S)
    mv model/model.onnx "model/model_old_${TIMESTAMP}.onnx"
    echo -e "${GREEN}✓ Old model backed up: model_old_${TIMESTAMP}.onnx${NC}"
else
    echo -e "${YELLOW}⚠ No old model found (first time setup)${NC}"
fi
echo ""

# Install new model
echo -e "${YELLOW}[5/5] Installing new model...${NC}"
mv "${MODEL}.onnx" model/model.onnx
echo -e "${GREEN}✓ New model installed: $MODEL${NC}"
echo ""

# Summary
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${GREEN}✅ Installation Complete!${NC}"
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo ""
echo "Model: $MODEL"
echo "Location: model/model.onnx"
echo ""
echo "Expected confidence: 20-80% (vs old 1-6%)"
echo ""
echo -e "${YELLOW}Next steps:${NC}"
echo "  1. Test standalone:"
echo "     ./scripts/test_standalone.sh --conf-thres 0.3"
echo ""
echo "  2. Test full system:"
echo "     ./scripts/test_full_system.sh"
echo ""
echo "You should now see bounding boxes in OpenGL! 🎯"
echo ""

