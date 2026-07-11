#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Headless kidnapped-robot relocalization regression, CLI, CI-friendly.
# Launches the (light, nav-free) sim + AMCL + recovery + injector, runs N kidnap
# trials, and exits 0 iff >= 90% relocalize within 30 s and 2 m. No GUI required.
set -euo pipefail

source /opt/ros/jazzy/setup.bash
source /ros_ws/install/setup.bash
[ -f /overlay_ws/install/setup.bash ] && source /overlay_ws/install/setup.bash
[ -f "$HOME/oomwoo-dev/install/setup.bash" ] && source "$HOME/oomwoo-dev/install/setup.bash"
export LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe

TRIALS=${TRIALS:-10}
LOG=${LOG:-/tmp/reloc_regression.log}
echo "[run] launching headless relocalization stack -> $LOG"
ros2 launch oomwoo_sim_support relocalize_regression.launch.py > "$LOG" 2>&1 &
LAUNCH_PID=$!
trap 'kill -INT $LAUNCH_PID 2>/dev/null || true' EXIT

set +e
ros2 run oomwoo_sim_support reloc_regression_runner --ros-args \
  -p num_trials:="$TRIALS" -p use_sim_time:=true
CODE=$?
set -e

echo "[run] reloc_report.json:"
cat /root/reloc_report.json 2>/dev/null || true
echo "[run] exit code: $CODE  (0 = PASS)"
exit $CODE
