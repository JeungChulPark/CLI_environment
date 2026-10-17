#!/usr/bin/env bash
# Current planner (Smac 2D: a point robot, the goal heading is ignored) vs Smac Hybrid-A* (plans x, y AND heading with the
# real footprint; Reeds-Shepp = may reverse) on the saved map, no robot. Poses: "<sx,sy,syaw>;<gx,gy,gyaw>" ...
here=$(cd "$(dirname "$0")" && pwd); cd "$here"
map=$here/../maps/globalmap_261002.yaml
source /opt/ros/jazzy/setup.bash; export ROS_DOMAIN_ID=7 RMW_IMPLEMENTATION=rmw_fastrtps_cpp; unset ROS_STATIC_PEERS FASTRTPS_DEFAULT_PROFILES_FILE
for name in 2d hybrid; do
  # the same generator as the robot uses, so what is measured here is what real_up.sh / real_loc_up.sh deploy
  PLANNER=$name python3 ../make_nav2_real_params.py base.yaml > /dev/null
  python3 - <<PY
import yaml
p = yaml.safe_load(open('base.yaml')); g = p['global_costmap']['global_costmap']['ros__parameters']
g['rolling_window'] = False; g.pop('width', None); g.pop('height', None)      # static map extent, robot-independent
yaml.safe_dump(p, open('variant.yaml', 'w'), sort_keys=False)
PY
  setsid nohup ros2 launch "$here/planner_eval.launch.py" params:="$here/variant.yaml" map:="$map" > "approach_$name.log" 2>&1 < /dev/null &
  pid=$!; sleep 14
  echo "== $name: $(grep -E "GridBased of type" approach_$name.log | head -1 | sed 's/.*\]: //' | cut -c1-90)"
  python3 approach_eval.py "approach_$name.json" "$@" 2>&1 | tail -8
  grep -iE "error|fail" "approach_$name.log" | grep -v "Inflation layer either" | head -3 | cut -c1-160
  kill -INT -- -$pid 2>/dev/null; sleep 4; kill -KILL -- -$pid 2>/dev/null
done
