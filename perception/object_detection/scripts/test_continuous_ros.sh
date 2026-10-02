#!/bin/bash
# Standalone object detection runner.
# Supports ROS topic mode (default) and direct ZED SDK mode.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_ROOT="$(dirname "$SCRIPT_DIR")"

SOURCE="ros"
SVO_FILE=""
CONF_THRES="0.05"
NO_GL=""

while [[ $# -gt 0 ]]; do
    case "$1" in
        --source)
            SOURCE="$2"
            shift 2
            ;;
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
            cat <<'USAGE'
Usage: ./test_continuous_ros.sh [OPTIONS]

Options:
  --source ros|zed     Input source mode (default: ros)
  --svo FILE           ZED mode only: use SVO file
  --conf-thres VALUE   Confidence threshold (default: 0.05)
  --no-gl              Disable OpenGL view (zed mode)
USAGE
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            exit 1
            ;;
    esac
done

if [[ "$SOURCE" != "ros" && "$SOURCE" != "zed" ]]; then
    echo "Invalid --source '$SOURCE' (expected ros or zed)"
    exit 1
fi

if [[ ! -f "$PKG_ROOT/model/URC_custom_model.onnx" ]]; then
    echo "Missing model: $PKG_ROOT/model/URC_custom_model.onnx"
    exit 1
fi

if [[ ! -f "$PKG_ROOT/model/URC_custom_classes.txt" ]]; then
    echo "Missing classes: $PKG_ROOT/model/URC_custom_classes.txt"
    exit 1
fi

if [[ "$SOURCE" == "ros" ]]; then
    CMD=(
        python3 "$SCRIPT_DIR/standalone_continuous_object_detector_ros.py"
        --onnx "$PKG_ROOT/model/URC_custom_model.onnx"
        --classes "$PKG_ROOT/model/URC_custom_classes.txt"
        --image-topic /zed2i/zed2i_camera/left/image_rect_color
        --conf-thres "$CONF_THRES"
    )
    [[ -n "$NO_GL" ]] && CMD+=("$NO_GL")
    echo "Running object detector in ROS topic mode"
    echo "Image topic: /zed2i/zed2i_camera/left/image_rect_color"
else
    CMD=(
        python3 "$SCRIPT_DIR/standalone_continuous_object_detector.py"
        --onnx "$PKG_ROOT/model/URC_custom_model.onnx"
        --classes "$PKG_ROOT/model/URC_custom_classes.txt"
        --conf-thres "$CONF_THRES"
    )
    [[ -n "$NO_GL" ]] && CMD+=("$NO_GL")

    if [[ -n "$SVO_FILE" ]]; then
        if [[ ! -f "$SVO_FILE" ]]; then
            echo "SVO file not found: $SVO_FILE"
            exit 1
        fi
        CMD+=(--svo "$SVO_FILE")
    fi

    echo "Running object detector in direct ZED mode"
fi

exec "${CMD[@]}"
