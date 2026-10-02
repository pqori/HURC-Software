#!/bin/bash
# Easy runner for the ArUco detector

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DETECTOR_DIR="$(dirname "$SCRIPT_DIR")"

export PYTHONPATH="${DETECTOR_DIR}:${PYTHONPATH}"
export DISPLAY=:0

echo "=============================================="
echo "ArUco Detector - 3D Visualization"
echo "=============================================="
echo "PYTHONPATH: $PYTHONPATH"
echo ""

# Run with provided arguments
python3 "${SCRIPT_DIR}/standalone_detector_3d.py" "$@"

