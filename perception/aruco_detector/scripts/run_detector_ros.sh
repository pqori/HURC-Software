#!/bin/bash
# Easy runner for the ArUco detector

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DETECTOR_DIR="$(dirname "$SCRIPT_DIR")"

export PYTHONPATH="${DETECTOR_DIR}:${PYTHONPATH}"
# Respect an existing DISPLAY (e.g. noVNC:0.0 in Docker).
# Fallback to :0 only when running directly on a local desktop session.
if [[ -z "${DISPLAY:-}" ]]; then
    export DISPLAY=:0
fi

SOURCE="ros"
ARGS=()

while [[ $# -gt 0 ]]; do
    case "$1" in
        --source)
            SOURCE="$2"
            shift 2
            ;;
        *)
            ARGS+=("$1")
            shift
            ;;
    esac
done

echo "=============================================="
echo "ArUco Detector - 3D Visualization"
echo "=============================================="
echo "PYTHONPATH: $PYTHONPATH"
echo "SOURCE: $SOURCE"
echo ""

# Run with provided arguments
if [[ "$SOURCE" == "ros" ]]; then
    python3 "${SCRIPT_DIR}/standalone_detector_ros.py" --use-depth "${ARGS[@]}"
elif [[ "$SOURCE" == "zed" ]]; then
    python3 "${SCRIPT_DIR}/standalone_detector_3d.py" "${ARGS[@]}"
else
    echo "Unknown source '$SOURCE' (expected 'ros' or 'zed')"
    exit 1
fi

