# Build nav2_sim.yaml from the Clearpath A200 Nav2 demo params, adjusted for the simulated laptop setup:
#  - 2D scan topic = VLP-16 ring slice (sensors/lidar3d_0/scan)
#  - global costmap rolling window 20 m -> 60 m, so goals anywhere in the warehouse are inside its bounds
#  - MPPI 2000 samples x 56 steps @ 20 Hz only reached ~8 Hz next to Gazebo on the laptop CPU:
#    1000 samples, 15 Hz, same ~2.8 s horizon (MPPI requires model_dt >= control period)
#  - local costmap from the live scan only (no static map layer): the hdl-derived grid is thicker/noisier than
#    slam_toolbox's and boxed the robot in at the start pose
#  - local voxel grid 16 x 0.05 m = 0.8 m tall, but the horizontal VLP ring is ~0.82 m above the ground, so it marked
#    nothing: 16 x 0.1 m
#  - planner NavFn -> SmacPlanner2D: NavFn kept failing with "legal potential found" on the partly unknown hdl grid
#    (inflation stays 0.8 m: below the 0.71 m circumscribed radius the costmaps log an error on every update)
#  (2026-10-01, from the real robot; shared by make_nav2_real_params.py)
#  - local costmap scan topic absolute: the relative name never reached the scan, so the local costmap was empty
#  - footprint 1.04 x 0.76 m (Husky ~1.0 x 0.67 + margin): the 1.1 x 0.9 box blocked turning next to obstacles
#  - inflation cost_scaling 4 -> 2.5, Smac cost_travel_multiplier 3: keep paths away from obstacles
#  - global costmap without the scan obstacle layer (smeared walls during turns, never cleared)
#  - bt_navigator: back-off-first recovery tree (objpose/nav/bt_backup_first.xml), default_server_timeout 20 -> 200 ms
#    (20 ms made the BT abort goals whenever the planner was busy: "Timed out ... compute_path_to_pose")
import os, sys, yaml
src = '/opt/ros/jazzy/share/clearpath_nav2_demos/config/a200/nav2.yaml'
p = yaml.safe_load(open(src))
scan = 'sensors/lidar3d_0/scan'
def fix_topics(d):
    if isinstance(d, dict):
        for k, v in d.items():
            if k == 'topic' and v == 'sensors/lidar2d_0/scan': d[k] = scan
            else: fix_topics(v)
    elif isinstance(d, list):
        for v in d: fix_topics(v)
fix_topics(p)
g = p['global_costmap']['global_costmap']['ros__parameters']; g['width'] = g['height'] = 60
c = p['controller_server']['ros__parameters']; c['controller_frequency'] = 15.0
f = c['FollowPath']; f['batch_size'] = 1000; f['model_dt'] = 0.07; f['time_steps'] = 40
l = p['local_costmap']['local_costmap']['ros__parameters']
l['plugins'] = [x for x in l['plugins'] if x != 'static_layer']
l['voxel_layer']['z_resolution'] = 0.1
p['planner_server']['ros__parameters']['GridBased'] = {
    'plugin': 'nav2_smac_planner::SmacPlanner2D', 'tolerance': 0.5, 'allow_unknown': True,
    'downsample_costmap': False, 'max_planning_time': 2.0, 'cost_travel_multiplier': 2.0, 'use_final_approach_orientation': False}
gc = p['global_costmap']['global_costmap']['ros__parameters']
gc['plugins'] = [x for x in gc['plugins'] if x != 'obstacle_layer']
fp = '[ [0.52, 0.38], [0.52, -0.38], [-0.52, -0.38], [-0.52, 0.38] ]'
for c in (gc, l):
    c['footprint'] = fp; c['inflation_layer']['cost_scaling_factor'] = 2.5
l['voxel_layer']['scan']['topic'] = '/a200_0881/' + scan
p['planner_server']['ros__parameters']['GridBased']['cost_travel_multiplier'] = 3.0
bt = p['bt_navigator']['ros__parameters']
bt['default_server_timeout'] = 1000   # 200 ms still aborted goals on the real laptop (planner busy, 2026-10-02)
if os.environ.get('BT_DEFAULT') != '1':   # BT_DEFAULT=1: stock Nav2 recovery tree (for A/B tests)
    bt['default_nav_to_pose_bt_xml'] = os.path.expanduser('~/objpose/nav/bt_backup_first.xml')
yaml.safe_dump(p, open(sys.argv[1], 'w'), sort_keys=False)
print('wrote', sys.argv[1])
