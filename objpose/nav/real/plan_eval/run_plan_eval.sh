#!/usr/bin/env bash
# Run plan_eval.py for planner variants A/B/C (global costmap inflation + Smac cost_travel_multiplier) on a saved map.
here=$(cd "$(dirname "$0")" && pwd); cd "$here"
map=${1:-$here/../maps/globalmap_261002.yaml}
source /opt/ros/jazzy/setup.bash; export ROS_DOMAIN_ID=7 RMW_IMPLEMENTATION=rmw_fastrtps_cpp; unset ROS_STATIC_PEERS FASTRTPS_DEFAULT_PROFILES_FILE
python3 ../make_nav2_real_params.py base.yaml > /dev/null
for v in "A 0.8 2.5 3" "B 1.1 1.5 5" "C 1.3 1.0 6"; do
  set -- $v; name=$1
  python3 - "$2" "$3" "$4" <<PY
import sys, yaml
p = yaml.safe_load(open('base.yaml')); r, s, t = float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])
g = p['global_costmap']['global_costmap']['ros__parameters']
g['inflation_layer']['inflation_radius'] = r; g['inflation_layer']['cost_scaling_factor'] = s
g['rolling_window'] = False; g.pop('width', None); g.pop('height', None)   # static map extent, robot-independent
p['planner_server']['ros__parameters']['GridBased']['cost_travel_multiplier'] = t
yaml.safe_dump(p, open('variant.yaml', 'w'), sort_keys=False)
PY
  setsid nohup ros2 launch "$here/planner_eval.launch.py" params:="$here/variant.yaml" map:="$map" > "eval_$name.log" 2>&1 < /dev/null &
  pid=$!; sleep 12
  grep -E "inflation|GridBased of type|Inflation layer" "eval_$name.log" | head -2 | sed 's/.*\]: //' | cut -c1-110
  echo -n "$name (inflation $2, scaling $3, travel $4): "; python3 plan_eval.py "$map" "eval_$name.json" 20 2>&1 | tail -1
  kill -INT -- -$pid 2>/dev/null; sleep 4; kill -KILL -- -$pid 2>/dev/null
done
