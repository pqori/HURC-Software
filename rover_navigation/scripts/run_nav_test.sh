#!/usr/bin/env bash
# Headless end-to-end Nav2 test: start the sim, start sim_nav.launch.py, drive
# to a goal with go_to_pose.py, record evidence, and clean up only the
# processes this script started. Exit code mirrors the mission result
# (0 = SUCCEEDED, 1 = mission failed, 2 = setup failed).
#
#   rover_navigation/scripts/run_nav_test.sh                 # empty world, goal (8, 0)
#   rover_navigation/scripts/run_nav_test.sh --log-dir /tmp/run1
#   rover_navigation/scripts/run_nav_test.sh \
#       --sim-launch "rover_sim sim.launch.py" --sim-args "world:=obstacles.sdf headless:=true rviz:=false"
#
# Options (defaults in brackets):
#   --domain-id N      ROS_DOMAIN_ID for every process [82]
#   --partition NAME   GZ_PARTITION for every process [hurc_b]
#   --sim-launch "PKG FILE"   sim launch [rover_description gazebo.launch.py]
#   --sim-args "ARGS"  extra sim launch args [headless:=true rviz:=false]
#   --nav-args "ARGS"  extra sim_nav.launch.py args []
#   --x X --y Y --yaw YAW     goal in map [8.0 0.0 0.0]
#   --timeout S        mission timeout in sim seconds [180]
#   --log-dir DIR      where logs go [./nav_test_logs/<timestamp>]
#   --keep-running     do not stop sim/nav at the end (prints how to stop them)
#
# The domain and partition are deliberately NOT taken from an inherited
# ROS_DOMAIN_ID / GZ_PARTITION so a sim on the user's default domain is never
# touched. Run from the repo root (it re-enters itself through `pixi run` if
# ros2 is not on PATH).
set -uo pipefail

DOMAIN_ID=82
PARTITION=hurc_b
SIM_LAUNCH="rover_description gazebo.launch.py"
SIM_ARGS="headless:=true rviz:=false"
NAV_ARGS=""
GOAL_X=8.0
GOAL_Y=0.0
GOAL_YAW=0.0
TIMEOUT=180
LOG_DIR=""
KEEP_RUNNING=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --domain-id) DOMAIN_ID="$2"; shift 2 ;;
    --partition) PARTITION="$2"; shift 2 ;;
    --sim-launch) SIM_LAUNCH="$2"; shift 2 ;;
    --sim-args) SIM_ARGS="$2"; shift 2 ;;
    --nav-args) NAV_ARGS="$2"; shift 2 ;;
    --x) GOAL_X="$2"; shift 2 ;;
    --y) GOAL_Y="$2"; shift 2 ;;
    --yaw) GOAL_YAW="$2"; shift 2 ;;
    --timeout) TIMEOUT="$2"; shift 2 ;;
    --log-dir) LOG_DIR="$2"; shift 2 ;;
    --keep-running) KEEP_RUNNING=1; shift ;;
    -h|--help) sed -n '2,30p' "$0"; exit 0 ;;
    *) echo "unknown option: $1" >&2; exit 2 ;;
  esac
done

# Locate the workspace root (directory holding install/setup.bash).
SCRIPT_PATH="$(cd "$(dirname "$0")" && pwd -P)/$(basename "$0")"
if [[ -L "$0" ]]; then SCRIPT_PATH="$(readlink "$0")"; fi
WS_ROOT="${WS_ROOT:-}"
if [[ -z "$WS_ROOT" ]]; then
  d="$(dirname "$SCRIPT_PATH")"
  while [[ "$d" != "/" ]]; do
    if [[ -f "$d/pixi.toml" ]]; then WS_ROOT="$d"; break; fi
    d="$(dirname "$d")"
  done
fi
[[ -z "$WS_ROOT" && -f "$PWD/pixi.toml" ]] && WS_ROOT="$PWD"
if [[ -z "$WS_ROOT" ]]; then echo "cannot find the workspace root (pixi.toml)" >&2; exit 2; fi
cd "$WS_ROOT" || exit 2

# Enter the pixi environment if needed.
if ! command -v ros2 >/dev/null 2>&1; then
  export PATH="$HOME/.pixi/bin:$PATH"
  exec pixi run bash "$SCRIPT_PATH" --domain-id "$DOMAIN_ID" --partition "$PARTITION" \
    --sim-launch "$SIM_LAUNCH" --sim-args "$SIM_ARGS" --nav-args "$NAV_ARGS" \
    --x "$GOAL_X" --y "$GOAL_Y" --yaw "$GOAL_YAW" --timeout "$TIMEOUT" \
    ${LOG_DIR:+--log-dir "$LOG_DIR"} $([[ $KEEP_RUNNING == 1 ]] && echo --keep-running)
fi

# shellcheck disable=SC1091
set +u  # colcon's setup scripts read unset variables
source "$WS_ROOT/install/setup.bash" || { echo "failed to source install/setup.bash" >&2; exit 2; }
set -u
export ROS_DOMAIN_ID="$DOMAIN_ID"
export GZ_PARTITION="$PARTITION"
# Every child inherits this marker; cleanup only touches processes carrying it.
export NAV_TEST_RUN_ID="navtest_$$_$(date +%s)"

LOG_DIR="${LOG_DIR:-$WS_ROOT/nav_test_logs/$(date +%Y%m%d_%H%M%S)}"
mkdir -p "$LOG_DIR"
LOG_DIR="$(cd "$LOG_DIR" && pwd -P)"
SUMMARY="$LOG_DIR/summary.txt"

log() { echo "[run_nav_test $(date +%H:%M:%S)] $*" | tee -a "$SUMMARY"; }
# Run a command with a wall-clock limit (macOS has no `timeout`).
with_timeout() { local s="$1"; shift; perl -e 'alarm shift; exec @ARGV' "$s" "$@"; }

set -m  # each background job gets its own process group
PGIDS=()
start_bg() {  # start_bg <logfile> <cmd...>
  local logf="$1"; shift
  "$@" >"$logf" 2>&1 &
  PGIDS+=("$!")
}

own_pids() {  # PIDs in our process groups, or carrying this run's env marker
  # (gz sim's ruby wrapper hides its env from ps -E, hence the group check)
  local g
  for g in "${PGIDS[@]:-}"; do
    [[ -n "$g" ]] && ps -axo pid=,pgid= | awk -v g="$g" -v self=$$ '$2 == g && $1 != self {print $1}'
  done
  own_env_pids
}
own_env_pids() {
  # (skips this script, its subshells and the ps/grep/awk helpers themselves)
  ps -Eww -axo pid=,command= 2>/dev/null | grep -F "NAV_TEST_RUN_ID=$NAV_TEST_RUN_ID" \
    | awk -v self=$$ '$1 != self && $2 !~ /(^|\/)(ps|grep|awk|bash|sh|perl|sleep|tee|sed)$/ {print $1}'
}

cleanup() {
  local rc=$?
  trap - EXIT INT TERM
  if [[ $KEEP_RUNNING == 1 ]]; then
    log "--keep-running: leaving process groups ${PGIDS[*]} running (kill -INT -- -PGID to stop)"
    exit "${FINAL_RC:-$rc}"
  fi
  log "cleaning up process groups: ${PGIDS[*]:-none}"
  for sig in INT TERM KILL; do
    for g in "${PGIDS[@]:-}"; do [[ -n "$g" ]] && kill -s "$sig" -- "-$g" 2>/dev/null; done
    local pids; pids="$(own_pids)"
    [[ -n "$pids" ]] && kill -s "$sig" $pids 2>/dev/null
    for _ in $(seq 1 20); do
      [[ -z "$(own_pids)" ]] && break
      sleep 0.5
    done
    [[ -z "$(own_pids)" ]] && break
  done
  ros2 daemon stop >/dev/null 2>&1
  local left; left="$(own_pids)"
  if [[ -n "$left" ]]; then
    log "WARNING: processes still alive after cleanup: $left"
  else
    log "cleanup complete; no processes from this run remain"
  fi
  exit "${FINAL_RC:-$rc}"
}
trap cleanup EXIT
trap 'FINAL_RC=130; exit 130' INT TERM

wait_for() {  # wait_for <description> <seconds> <cmd...>
  local what="$1" secs="$2"; shift 2
  local t0=$SECONDS
  while (( SECONDS - t0 < secs )); do
    if "$@" >/dev/null 2>&1; then
      log "$what: ok after $((SECONDS - t0)) s"
      return 0
    fi
    sleep 2
  done
  log "$what: NOT ready after $secs s"
  return 1
}

controllers_active() {
  local out
  out="$(with_timeout 20 ros2 control list_controllers 2>/dev/null)" || return 1
  for c in joint_state_broadcaster rover_drive_controller rover_arm_controller; do
    echo "$out" | grep -E "^\s*$c\b.*\bactive\b" >/dev/null || return 1
  done
}

nav_active() {
  with_timeout 15 ros2 service call /lifecycle_manager_navigation/is_active std_srvs/srv/Trigger 2>/dev/null \
    | grep -q "success=True"
}

log "log dir: $LOG_DIR"
log "ROS_DOMAIN_ID=$ROS_DOMAIN_ID GZ_PARTITION=$GZ_PARTITION run id $NAV_TEST_RUN_ID"
log "sim: ros2 launch $SIM_LAUNCH $SIM_ARGS"
log "goal: x=$GOAL_X y=$GOAL_Y yaw=$GOAL_YAW timeout=${TIMEOUT}s"

# shellcheck disable=SC2086
start_bg "$LOG_DIR/sim.log" ros2 launch $SIM_LAUNCH $SIM_ARGS
if ! wait_for "controllers active" 300 controllers_active; then
  FINAL_RC=2; exit 2
fi
with_timeout 20 ros2 control list_controllers >"$LOG_DIR/controllers.txt" 2>&1
with_timeout 10 gz model -m rover -p >"$LOG_DIR/world_pose_start.txt" 2>&1

# Let the rover settle on its wheels before Nav2 starts.
sleep 5

# shellcheck disable=SC2086
start_bg "$LOG_DIR/nav.log" ros2 launch rover_navigation sim_nav.launch.py $NAV_ARGS
if ! wait_for "Nav2 lifecycle manager active" 240 nav_active; then
  FINAL_RC=2; exit 2
fi

log "running go_to_pose.py"
T0=$SECONDS
python3 "$(ros2 pkg prefix rover_navigation)/lib/rover_navigation/go_to_pose.py" \
  --x "$GOAL_X" --y "$GOAL_Y" --yaw "$GOAL_YAW" --timeout "$TIMEOUT" 2>&1 | tee "$LOG_DIR/go_to_pose.log"
MISSION_RC=${PIPESTATUS[0]}
log "go_to_pose exit code $MISSION_RC (wall $((SECONDS - T0)) s)"
grep -E "RESULT|elapsed|final" "$LOG_DIR/go_to_pose.log" | tee -a "$SUMMARY"

log "final world pose (gz model -m rover -p):"
with_timeout 15 gz model -m rover -p 2>&1 | grep -v -i warning | tee "$LOG_DIR/world_pose_final.txt" | tee -a "$SUMMARY"

log "ros2 topic hz /odometry/filtered (8 s):"
with_timeout 8 ros2 topic hz /odometry/filtered >"$LOG_DIR/odometry_filtered_hz.txt" 2>&1
tail -4 "$LOG_DIR/odometry_filtered_hz.txt" | tee -a "$SUMMARY"

log "TF map -> base_link:"
with_timeout 6 ros2 run tf2_ros tf2_echo map base_link >"$LOG_DIR/tf_map_base_link.txt" 2>&1
grep -m1 -A3 "At time" "$LOG_DIR/tf_map_base_link.txt" | tee -a "$SUMMARY"
# The point cloud is stamped Front_Zed_left_camera_frame (x forward, z up, as
# Gazebo publishes it); the RGB images use the optical frame. Check both.
for f in Front_Zed_left_camera_frame Front_Zed_left_camera_optical_frame; do
  log "TF map -> $f:"
  with_timeout 6 ros2 run tf2_ros tf2_echo map "$f" >"$LOG_DIR/tf_map_$f.txt" 2>&1
  if grep -q "At time" "$LOG_DIR/tf_map_$f.txt"; then
    grep -m1 -A3 "At time" "$LOG_DIR/tf_map_$f.txt" | tee -a "$SUMMARY"
  else
    log "  $f did not resolve:"; grep -m2 -iE "error|exist|warn" "$LOG_DIR/tf_map_$f.txt" | tee -a "$SUMMARY"
  fi
done
log "TF tree (tf2_tools view_frames):"
(cd "$LOG_DIR" && with_timeout 30 ros2 run tf2_tools view_frames >"$LOG_DIR/view_frames.txt" 2>&1)
ls "$LOG_DIR"/frames_* 2>/dev/null | tee -a "$SUMMARY"
GV="$(ls "$LOG_DIR"/frames_*.gv 2>/dev/null | head -1)"
if [[ -n "$GV" ]]; then
  grep -oE '"[^"]+" -> "[^"]+"' "$GV" | sort >"$LOG_DIR/tf_edges.txt"
  log "  $(wc -l <"$LOG_DIR/tf_edges.txt" | tr -d ' ') TF edges; key edges:"
  grep -E '"(map|odom|base_link)" -> "(odom|base_link|Chassis|Front_Zed_[a-z_]+)"' "$LOG_DIR/tf_edges.txt" | tee -a "$SUMMARY"
fi

if [[ $MISSION_RC -eq 0 ]]; then log "MISSION SUCCEEDED"; else log "MISSION FAILED"; fi
FINAL_RC=$MISSION_RC
exit "$MISSION_RC"
