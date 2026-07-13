#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Headless coverage-cleaning regression, CLI, CI-friendly.
# Launches the sim + Nav2 + coverage planner/meter, runs the scoring node, and
# exits 0 iff coverage >= 90% and efficiency >= 80%. No GUI/display required.
#
#   RUNS=3 ./run_coverage_regression.sh    # repeat 3x and report the spread
#
# Exit codes: 0 = all runs pass, 1 = a run missed its targets,
#             2 = measurement invalid (sim unstable on this host).
set -eo pipefail

source /opt/ros/jazzy/setup.bash
source /ros_ws/install/setup.bash
[ -f /overlay_ws/install/setup.bash ] && source /overlay_ws/install/setup.bash
[ -f "$HOME/oomwoo-dev/install/setup.bash" ] && source "$HOME/oomwoo-dev/install/setup.bash"
export LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe
# isolate DDS discovery so a co-running ROS graph can't interfere
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-77} ROS_LOCALHOST_ONLY=1

RUNS=${RUNS:-1}
LOG=${LOG:-/tmp/coverage_regression.log}
LAUNCH_ARGS=${LAUNCH_ARGS:-}
REPORT_DIR=$(mktemp -d /tmp/coverage_runs.XXXX)
WORST=0

for i in $(seq 1 "$RUNS"); do
  echo "[run] ($i/$RUNS) launching headless coverage stack -> $LOG.$i"
  # shellcheck disable=SC2086
  ros2 launch oomwoo_sim_support coverage_regression.launch.py $LAUNCH_ARGS \
    > "$LOG.$i" 2>&1 &
  LAUNCH_PID=$!
  trap 'kill -INT $LAUNCH_PID 2>/dev/null || true' EXIT

  set +e
  ros2 run oomwoo_sim_support coverage_regression_runner --ros-args \
    -p use_sim_time:=true -p report_path:="$REPORT_DIR/run$i.json"
  CODE=$?
  set -e
  [ "$CODE" -gt "$WORST" ] && WORST=$CODE

  echo "[run] ($i/$RUNS) report:"
  cat "$REPORT_DIR/run$i.json" 2>/dev/null || true
  cp -f "$REPORT_DIR/run$i.json" /root/coverage_report.json 2>/dev/null || true
  echo
  echo "[run] ($i/$RUNS) exit code: $CODE"

  kill -INT $LAUNCH_PID 2>/dev/null || true
  wait $LAUNCH_PID 2>/dev/null || true
  pkill -f "gz sim" 2>/dev/null || true
  sleep 5
done

if [ "$RUNS" -gt 1 ]; then
  echo "[run] ===== VARIANCE over $RUNS runs ====="
  python3 - "$REPORT_DIR" <<'PY'
import json, glob, statistics as st, sys
rs = [json.load(open(p)) for p in sorted(glob.glob(sys.argv[1] + '/run*.json'))]
for k in ('coverage', 'efficiency'):
    v = [r[k] for r in rs]
    print(f"  {k:11}: min={min(v):.4f} max={max(v):.4f} mean={st.mean(v):.4f}"
          + (f" stdev={st.stdev(v):.4f}" if len(v) > 1 else ""))
print(f"  passes     : {sum(r['pass'] for r in rs)}/{len(rs)}"
      f"  unstable: {sum(bool(r.get('sim_unstable')) for r in rs)}")
PY
fi
echo "[run] overall exit: $WORST  (0=PASS, 1=target missed, 2=sim unstable on this host)"
exit $WORST
