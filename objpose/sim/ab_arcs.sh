#!/usr/bin/env bash
# A/B of the stuck_guard escape planner on the Gazebo warehouse (fresh sim per arm, back-off-first BT in both):
#   A = GUARD_ARCS=0 (straight forward/backward only), B = GUARD_ARCS=1 (arcs, scan-simulated footprint)
# usage: ab_arcs.sh [scenario ...]   (default: side_pin_backleft side_pin_back side_pin_left nose_wall boxed)
here=$(cd "$(dirname "$0")" && pwd); cd "$here"
scen=${*:-side_pin_backleft side_pin_back side_pin_left nose_wall boxed}
for arm in A B; do
  ./sim_down.sh >/dev/null; ./sim_up.sh warehouse --headless >/dev/null
  source ./sim_env.sh
  for i in $(seq 1 30); do ros2 topic list 2>/dev/null | grep -q platform/odom && break; sleep 5; done
  ./sim_nav.sh >/dev/null
  for i in $(seq 1 30); do grep -q "Managed nodes are active\|Aborting bringup" logs/nav2.log && break; sleep 3; done
  arcs=$([ $arm = A ] && echo 0 || echo 1)
  GUARD_ARCS=$arcs USE_SIM_TIME=true SCAN_TOPIC=/a200_0881/sensors/lidar3d_0/scan \
    setsid nohup python3 -u "$HOME/objpose/nav/stuck_guard.py" > logs/stuck_guard_$arm.log 2>&1 < /dev/null & echo $! > logs/guard.pid
  echo "=== arm $arm (GUARD_ARCS=$arcs)  $(python3 probe.py 5 2>/dev/null | cut -c1-60)  load $(cut -d" " -f1 /proc/loadavg)"
  python3 hard_tests.py logs/arcs_$arm $scen 2>&1 | grep -vE "^\[(INFO|WARN)" | grep -E '^\{|^  \[' | cut -c1-420
  echo "--- stuck_guard events"; grep -E "escape|stuck" logs/stuck_guard_$arm.log | sed 's/.*\]: //' | cut -c1-300
done
./sim_down.sh >/dev/null
