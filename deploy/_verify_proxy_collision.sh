#!/usr/bin/env bash
# Prove the collision proxies work, using Gazebo's true model pose as ground truth
# (from /world/default/pose/info). Spawn the robot north of the Sofa, drive south
# into it, compare the real world y before/after.
#   my world  (proxy present):   halts at the sofa face   (y stays > ~-0.6)
#   stock world (no mesh colln): ghosts toward sofa centre (y < ~-0.9)
# Usage: _verify_proxy_collision.sh <world>
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source /ros_ws/install/setup.bash
source "$HOME/oomwoo-dev/install/setup.bash"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-77} ROS_LOCALHOST_ONLY=1

SHARE=$(ros2 pkg prefix oomwoo_sim_support)/share/oomwoo_sim_support
WORLD=${1:-$SHARE/worlds/living_room.world}
LAUNCH=$HOME/oomwoo-dev/deploy/diag_livingroom.launch.py
SX=${SX:-0.394}; SY=${SY:--0.30}; SYAW=${SYAW:--1.5708}; ITEM=${ITEM:-Sofa}

ros2 launch "$LAUNCH" world:="$WORLD" x_pose:="$SX" y_pose:="$SY" yaw:="$SYAW" \
  > /tmp/verify_col.log 2>&1 &
LP=$!
trap 'kill -INT $LP 2>/dev/null || true' EXIT
sleep 27

get_y() {
  timeout 5 gz topic -e -t /world/default/pose/info -n 1 2>/dev/null | python3 -c '
import sys, re
txt = sys.stdin.read()
for b in txt.split("\npose {"):
    if "name: \"oomwoo_one\"" in b:
        m = re.search(r"position \{([^}]*)\}", b)
        y = 0.0
        if m:
            ym = re.search(r"y:\s*(-?[0-9.eE+]+)", m.group(1))
            if ym: y = float(ym.group(1))
        print(f"{y:.3f}"); break
'
}

Y0=$(get_y)
echo "[verify] world=$WORLD  item=$ITEM  spawn=($SX,$SY)"
echo "[verify] y_before=$Y0"
( timeout 12 ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
    '{linear: {x: 0.2}, angular: {z: 0.0}}' > /dev/null 2>&1 ) || true
sleep 2
Y1=$(get_y)
echo "[verify] y_after=$Y1"
python3 -c "
d=abs(float('$Y1')-float('$Y0'))
print(f'[verify] travelled {d:.2f} m ->', 'STOPPED at $ITEM (collision present)' if d<0.6 else 'PASSED THROUGH $ITEM (NO collision)')"
