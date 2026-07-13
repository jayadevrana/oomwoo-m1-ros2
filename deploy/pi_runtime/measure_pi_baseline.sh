#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Measure the OOMWOO onboard runtime's RSS/PSS/CPU baseline, per xbattlax's
# pi4_4gb_runtime_plan.md. Runs three phases and records each with
# measure_baseline.py (a /proc sampler, no external deps):
#
#   idle : robot_state_publisher only            -> the floor
#   slam : + slam_toolbox, driven by a 5 Hz bag  -> mapping cost
#   nav  : + AMCL + Nav2 + M1 behaviours on a map -> navigation cost
#
# No robot is attached; a recorded scan+odom+tf bag (BAG) replays with --clock
# so SLAM/Nav2 run at the real 5 Hz LiDAR rate. Writes one JSON per phase plus a
# combined baseline_report.json. Run ON the target (Pi 4/5); the numbers are the
# deliverable. Also works on any Linux box for a pipeline dry-run (CPU% differs;
# RSS/PSS is representative).
set -eo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/jazzy/setup.bash
# source whichever overlay workspace holds the robot description + M1 behaviours
[ -f "$HOME/oomwoo_runtime_ws/install/setup.bash" ] && source "$HOME/oomwoo_runtime_ws/install/setup.bash"  # Pi runtime
[ -f /ros_ws/install/setup.bash ] && source /ros_ws/install/setup.bash                                      # makerspet image
[ -f "$HOME/oomwoo-dev/install/setup.bash" ] && source "$HOME/oomwoo-dev/install/setup.bash"                # dev box
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-88} ROS_LOCALHOST_ONLY=1

LAUNCH="$HERE/oomwoo_runtime.launch.py"
MEASURE="$HERE/measure_baseline.py"
SERIAL="$HERE/oomwoo_sim_mcu_serial.py"
BAG=${BAG:-$HERE/scan_bag}   # the clean 5 Hz bag bundled next to this script
OUT=${OUT:-/tmp/pi_baseline}
SETTLE=${SETTLE:-18}
WINDOW=${WINDOW:-25}
mkdir -p "$OUT"

# the simulated MCU serial link runs across every phase (it's always-on onboard)
python3 "$SERIAL" --link /tmp/oomwoo-mcu-serial >/tmp/mcu.log 2>&1 &
MCU=$!
trap 'kill $MCU 2>/dev/null; pkill -f oomwoo_runtime 2>/dev/null; pkill -f "bag play" 2>/dev/null || true' EXIT

run_phase() {
  local mode="$1"
  echo "==== phase: $mode ===="
  local BP=""
  if [ "$mode" != idle ]; then
    # bag LEADS the graph: on a real robot the odom TF + /clock are always
    # flowing, so SLAM/Nav2 must see them before activating or the costmap
    # aborts on a missing base_link->odom transform. Give it a head start.
    ros2 bag play "$BAG" --clock 100 > "$OUT/$mode.bag.log" 2>&1 &
    BP=$!
    sleep 8
  fi
  ros2 launch "$LAUNCH" mode:="$mode" use_sim_time:=true > "$OUT/$mode.launch.log" 2>&1 &
  local LP=$!
  sleep "$SETTLE"
  python3 "$MEASURE" --label "$mode" --duration "$WINDOW" --interval 3 \
    --out "$OUT/$mode.json"
  kill -INT $LP 2>/dev/null || true
  [ -n "$BP" ] && kill -INT $BP 2>/dev/null || true
  sleep 5
}

run_phase idle
run_phase slam
run_phase nav

echo "==== combined baseline ===="
python3 - "$OUT" <<'PY'
import json, glob, os, sys
d = sys.argv[1]
rows = []
for m in ('idle', 'slam', 'nav'):
    p = os.path.join(d, m + '.json')
    if os.path.exists(p):
        r = json.load(open(p))
        rows.append({'phase': m, 'n_proc': r['n_proc'],
                     'rss_mb': r['peak_total_rss_mb'],
                     'pss_mb': r['peak_total_pss_mb'],
                     'cpu_pct': r['total_cpu_pct']})
print(f"{'phase':6} {'procs':>5} {'RSS_MB':>8} {'PSS_MB':>8} {'CPU%':>7}")
for r in rows:
    print(f"{r['phase']:6} {r['n_proc']:5d} {r['rss_mb']:8.1f} "
          f"{r['pss_mb']:8.1f} {r['cpu_pct']:7.1f}")
json.dump({'phases': rows}, open(os.path.join(d, 'baseline_report.json'), 'w'),
          indent=2)
print('\\nwrote', os.path.join(d, 'baseline_report.json'))
PY
