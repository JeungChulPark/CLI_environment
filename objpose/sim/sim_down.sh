#!/usr/bin/env bash
# Stop everything started by sim_up.sh / sim_nav.sh / sim_hdl_nav.sh (each is started with setsid, so kill its whole
# process group). Leftovers are matched by name AND must carry the simulation environment (ROS_DOMAIN_ID=5 or
# GZ_PARTITION=husky_sim): the real Clearpath driver also runs as this user under the a200_0881 namespace and must
# not be touched (2026-10-02: a name-only match killed the real ros2_control/ekf/twist_mux nodes).
here=$(cd "$(dirname "$0")" && pwd)
for f in "$here"/logs/*.pid; do
  [ -f "$f" ] || continue; p=$(cat "$f"); kill -INT -- -"$p" 2>/dev/null; rm -f "$f"
done
sleep 5
pat='[s]tuck_guard|[h]dl_graph_slam|[h]dl_sim|[h]dl_map_relay|[h]dl_grid|[p]refiltering_node|[s]can_matching_odometry|[g]raph_slam_node|[c]learpath_sim|[a]200_0881|[g]z sim|[p]arameter_bridge|[r]os_gz|[s]lam_toolbox|[n]av2_|[p]ointcloud_to_laserscan'
skip=" $$ "; p=$$; while [ "$p" -gt 1 ]; do p=$(ps -o ppid= -p "$p" | tr -d ' '); skip+="$p "; done
victims() {
  pgrep -f "$pat" | while read -r v; do
    [[ $skip == *" $v "* ]] && continue
    { tr '\0' '\n' < /proc/$v/environ; } 2>/dev/null | grep -qxE 'ROS_DOMAIN_ID=5|GZ_PARTITION=husky_sim' && echo "$v"
  done
}
v=$(victims); [ -n "$v" ] && kill -INT $v 2>/dev/null; sleep 3
v=$(victims); [ -n "$v" ] && kill -KILL $v 2>/dev/null
v=$(victims); [ -n "$v" ] && ps -o pid,cmd -p $v || echo "stopped (no sim processes left)"
