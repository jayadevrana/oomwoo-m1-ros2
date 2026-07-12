#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Headless coverage-cleaning regression, CLI, CI-friendly.
# Launches the sim + Nav2 + coverage planner/meter, runs the scoring node, and
# exits 0 iff coverage >= 90% and efficiency >= 80%. No GUI/display required.
set -eo pipefail

source /opt/ros/jazzy/setup.bash
source /ros_ws/install/setup.bash
[ -f /overlay_ws/install/setup.bash ] && source /overlay_ws/install/setup.bash
[ -f "$HOME/oomwoo-dev/install/setup.bash" ] && source "$HOME/oomwoo-dev/install/setup.bash"
export LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe
# isolate DDS discovery so a co-running ROS graph can't interfere
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-77} ROS_LOCALHOST_ONLY=1

LOG=${LOG:-/tmp/coverage_regression.log}
echo "[run] launching headless coverage stack -> $LOG"
ros2 launch oomwoo_sim_support coverage_regression.launch.py > "$LOG" 2>&1 &
LAUNCH_PID=$!
trap 'kill -INT $LAUNCH_PID 2>/dev/null || true' EXIT

# score the run (blocks until target / plateau / max time, exit code = pass/fail)
set +e
ros2 run oomwoo_sim_support coverage_regression_runner --ros-args -p use_sim_time:=true
CODE=$?
set -e

echo "[run] coverage_report.json:"
cat /root/coverage_report.json 2>/dev/null || true
echo "[run] exit code: $CODE  (0 = PASS)"
exit $CODE
