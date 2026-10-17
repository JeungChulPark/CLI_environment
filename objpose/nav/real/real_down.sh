#!/usr/bin/env bash
# Stop what real_up.sh started (not the Clearpath driver; use husky-down for that).
here=$(cd "$(dirname "$0")" && pwd)
for f in "$here"/logs/*.pid; do [ -f "$f" ] || continue; kill -INT -- -"$(cat "$f")" 2>/dev/null; rm -f "$f"; done
sleep 4
pat='[n]av_bridge|[m]ap_localizer|[s]tuck_guard|[v]lp16_publisher|[h]dl_graph_slam|[p]refiltering_node|[s]can_matching_odometry|[g]raph_slam_node|[f]loor_detection|[h]dl_map_relay|[h]dl_grid|[m]ap_server|[l]ifecycle_manager_map|[m]ap_server|[l]ifecycle_manager_map|[v]lp_to_scan|[r]obot_lidar_tf|[h]dl_lidar_tf|[n]av2_|[c]ontroller_server|[p]lanner_server|[b]t_navigator|[b]ehavior_server|[s]moother_server|[v]elocity_smoother|[c]ollision_monitor|[w]aypoint_follower|[d]ocking_server|[r]oute_server|[l]ifecycle_manager_navigation'
skip=" $$ "; p=$$; while [ "$p" -gt 1 ]; do p=$(ps -o ppid= -p "$p" | tr -d ' '); skip+="$p "; done
v=$(pgrep -f "$pat" | while read -r x; do [[ $skip == *" $x "* ]] || echo "$x"; done); [ -n "$v" ] && kill -INT $v 2>/dev/null; sleep 2
v=$(pgrep -f "$pat" | while read -r x; do [[ $skip == *" $x "* ]] || echo "$x"; done); [ -n "$v" ] && kill -KILL $v 2>/dev/null
echo stopped
