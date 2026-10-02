#!/usr/bin/env bash
# Open the Gazebo (Harmonic) GUI as a separate client process, attached to a
# Gazebo server that is already running (e.g. from `pixi run sim`).
# Used by `pixi run gz-gui`. On macOS this is the only way to get the Gazebo
# GUI: the server and the GUI cannot share one process there, so the launch
# file always starts the server with -s.
set -eo pipefail
cd "$(dirname "$0")/.."

if [ ! -f install/setup.bash ]; then
  echo "install/setup.bash not found; run 'pixi run build' first." >&2
  exit 1
fi
# shellcheck disable=SC1091
source install/setup.bash

# The GUI loads the rover meshes itself, so it needs the same resource path
# that gazebo.launch.py gives the server (package://rover_description/...).
GZ_SIM_RESOURCE_PATH="$(ros2 pkg prefix rover_description)/share${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
export GZ_SIM_RESOURCE_PATH

exec gz sim -g "$@"
