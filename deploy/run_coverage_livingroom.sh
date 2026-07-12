#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Headless coverage-cleaning regression on the STOCK living_room world (with the
# added collision proxies), CLI, CI-friendly. Same harness as
# run_coverage_regression.sh but pointed at the cluttered living_room + its map,
# spawning at the clearest floor cell. The room is tight, so coverage is lower
# than the open test_room by design -- this reports the honest number for it.
set -eo pipefail

source /opt/ros/jazzy/setup.bash
source /ros_ws/install/setup.bash
[ -f /overlay_ws/install/setup.bash ] && source /overlay_ws/install/setup.bash
[ -f "$HOME/oomwoo-dev/install/setup.bash" ] && source "$HOME/oomwoo-dev/install/setup.bash"
export LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-77} ROS_LOCALHOST_ONLY=1

SHARE=$(ros2 pkg prefix oomwoo_sim_support)/share/oomwoo_sim_support
WORLD=${WORLD:-$SHARE/worlds/living_room.world}
MAP=${MAP:-$SHARE/maps/living_room.yaml}
X=${X:-0.32}; Y=${Y:-1.59}; YAW=${YAW:-0.0}
LOG=${LOG:-/tmp/coverage_livingroom.log}

echo "[run] headless coverage on living_room -> $LOG"
echo "[run] world=$WORLD"
echo "[run] map=$MAP  spawn=($X,$Y)"
ros2 launch oomwoo_sim_support coverage_regression.launch.py \
  world:="$WORLD" map:="$MAP" x_pose:="$X" y_pose:="$Y" yaw:="$YAW" > "$LOG" 2>&1 &
LAUNCH_PID=$!
trap 'kill -INT $LAUNCH_PID 2>/dev/null || true' EXIT

set +e
ros2 run oomwoo_sim_support coverage_regression_runner --ros-args -p use_sim_time:=true
CODE=$?
set -e

echo "[run] coverage_report.json:"
cat /root/coverage_report.json 2>/dev/null || true
echo "[run] exit code: $CODE  (0 = PASS at the 90% gate; living_room is expected lower)"
exit $CODE
