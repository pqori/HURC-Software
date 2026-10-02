#!/usr/bin/env bash
set -e

# Path to this package’s installed HEX file
HEX_FILE="$(ros2 pkg prefix arm_firmware)/share/arm_firmware/firmware/arm_firmware.hex"

# Teensy loader CLI (make sure it’s installed on your system)
TEENSY_LOADER_CLI=teensy_loader_cli

echo "Uploading $HEX_FILE to Teensy..."
$TEENSY_LOADER_CLI -mmcu=TEENSY41 -w -v "$HEX_FILE"
