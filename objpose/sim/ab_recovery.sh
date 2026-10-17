#!/usr/bin/env bash
# A/B test of the recovery setup on the Gazebo warehouse (fresh sim per arm):
#   A = stock Nav2 recovery tree, no stuck_guard;  B = back-off-first tree + stuck_guard.py
# usage: ab_recovery.sh [scenario ...]   (default: nose_wall boxed low_box popup_obstacle crowd_boxes)
here=$(cd "$(dirname "$0")" && pwd); cd "$here"
scen=${*:-nose_wall boxed low_box popup_obstacle crowd_boxes}
for arm in A B; do
  ./sim_down.sh >/dev/null; ./sim_up.sh warehouse --headless >/dev/null
  source ./sim_env.sh
  for i in $(seq 1 30); do ros2 topic list 2>/dev/null | grep -q platform/odom && break; sleep 5; done
  if [ $arm = A ]; then BT_DEFAULT=1 ./sim_nav.sh >/dev/null; else ./sim_nav.sh >/dev/null; fi
  for i in $(seq 1 30); do grep -q "Managed nodes are active\|Aborting bringup" logs/nav2.log && break; sleep 3; done
  if [ $arm = B ]; then
    USE_SIM_TIME=true setsid nohup python3 -u "$HOME/objpose/nav/stuck_guard.py" > logs/stuck_guard.log 2>&1 < /dev/null & echo $! > logs/guard.pid
  fi
  echo "=== arm $arm: bt=$(ros2 param get /a200_0881/bt_navigator default_nav_to_pose_bt_xml 2>/dev/null | awk '{print $NF}')"
  python3 hard_tests.py logs/ab_$arm $scen 2>&1 | grep -vE "^\[(INFO|WARN)"
  [ $arm = B ] && { echo "--- stuck_guard events"; grep -E "escape|stuck" logs/stuck_guard.log | sed 's/.*\]: //' ; }
done
./sim_down.sh >/dev/null
