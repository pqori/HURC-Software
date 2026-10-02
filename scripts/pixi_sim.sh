#!/usr/bin/env bash
# Launch the Gazebo simulation from the pixi environment.
# Used by `pixi run sim` / `pixi run sim-headless`; extra arguments are
# passed through to the launch file (e.g. rviz:=false headless:=true).
set -eo pipefail
cd "$(dirname "$0")/.."

if [ ! -f install/setup.bash ]; then
  echo "install/setup.bash not found; run 'pixi run build' first." >&2
  exit 1
fi
# shellcheck disable=SC1091
source install/setup.bash

exec ros2 launch rover_description gazebo.launch.py "$@"
