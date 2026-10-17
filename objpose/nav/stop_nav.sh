#!/usr/bin/env bash
# Stop a running step2_nav_bag.sh (its TERM trap kills hdl, relay, Nav2 and the bag player by process group).
p=$(pgrep -f "bash ./step2_nav_bag[.]sh|step2_nav_bag[.]sh ")
[ -n "$p" ] && kill -TERM $p
for _ in $(seq 1 10); do
  n=$(ps -eo cmd | grep -cE "[c]ontroller_server|[g]raph_slam_node|[m]ap_server|[b]ag play|[m]ap_odom_relay")
  [ "$n" = 0 ] && break; sleep 1
done
echo "nav processes left: $n"
