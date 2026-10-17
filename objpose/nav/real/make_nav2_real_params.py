# nav2_real.yaml for the real A200: the simulation params (objpose/sim/make_nav2_sim_params.py: SmacPlanner2D,
# MPPI 1000x40 @15 Hz, scan-only local costmap, 60 m global costmap) with
#  - scan = /a200_0881/scan from pointcloud_to_laserscan (frame base_link, so the voxel layer sees it at ~0.15-1.5 m)
#  - wall clock, and a 0.3 m/s / 0.6 rad/s cap for the first runs
#  - global costmap = hdl grid (static layer) + inflation only: its scan obstacle layer smeared walls into arcs
#    during skid-steer turns (wheel yaw is poor until the next hdl correction) and never cleared them; live
#    obstacles are the rolling local costmap's job
#  - local costmap: absolute scan topic (the relative 'scan' never reached /a200_0881/scan, so the local costmap
#    stayed empty) and min_obstacle_height -1 (the projected scan has z = 0 in base_link)
#  - footprint 1.04 x 0.76 m (Husky ~1.0 x 0.67 + margin) instead of 1.1 x 0.9: with a 0.71 m corner radius the
#    collision monitor blocked turning in place 0.73 m from an obstacle
#  - inflation cost_scaling 4 -> 2.5 and Smac cost_travel_multiplier 2 -> 3: plan farther from obstacles
import math, subprocess, sys, yaml, os
here = os.path.dirname(os.path.realpath(__file__))
sim = os.path.expanduser('~/objpose/sim/make_nav2_sim_params.py')
tmp = os.path.join(here, '.nav2_sim_base.yaml')
subprocess.run([sys.executable, sim, tmp], check=True, stdout=subprocess.DEVNULL)
p = yaml.safe_load(open(tmp))
def walk(d):
    if isinstance(d, dict):
        for k, v in list(d.items()):
            if k == 'topic' and v == 'sensors/lidar3d_0/scan': d[k] = 'scan'
            elif k == 'use_sim_time': d[k] = False
            else: walk(v)
    elif isinstance(d, list):
        for v in d: walk(v)
walk(p)
# Turn rate 0.6 -> 1.0 rad/s (2026-10-14): Hybrid-A* plans turns down to a 0.30 m radius, but at 0.3 m/s a 0.6 rad/s cap only
# allows 0.5 m; the robot ran wide through 13 % of the planned turns and came back in an S (route run 261014_110723).
# Raising the planner's radius instead (0.45 / 0.55 m) doubled the reversing manoeuvres in this room. WZ_MAX=0.6 = before.
WZ = float(os.environ.get('WZ_MAX', '1.0'))
f = p['controller_server']['ros__parameters']['FollowPath']; f['vx_max'] = 0.3; f['vx_min'] = -0.2; f['wz_max'] = WZ
vs = p.get('velocity_smoother', {}).get('ros__parameters')
if vs: vs['max_velocity'] = [0.3, 0.0, WZ]; vs['min_velocity'] = [-0.2, 0.0, -WZ]   # the smoother caps the command again
gc = p['global_costmap']['global_costmap']['ros__parameters']
lc = p['local_costmap']['local_costmap']['ros__parameters']
fp = '[ [0.52, 0.38], [0.52, -0.38], [-0.52, -0.38], [-0.52, 0.38] ]'
for c in (gc, lc):
    c['footprint'] = fp; c['inflation_layer']['cost_scaling_factor'] = 2.5
vl = lc['voxel_layer']; vl['min_obstacle_height'] = -1.0
vl['scan'].update({'topic': '/a200_0881/scan', 'min_obstacle_height': -1.0})
p['planner_server']['ros__parameters']['GridBased']['cost_travel_multiplier'] = 3.0
# the room map is ~24 m across: a 30 m rolling global costmap (not the warehouse's 60 m) and a 1 s planning budget keep
# replanning well inside the BT's server timeout while map_localizer shares the CPU
gc['width'] = gc['height'] = 30
# (2026-10-02) a wider/flatter inflation (1.0 m, scaling 1.5) and cost_travel_multiplier 5 to keep paths in the middle
# of the corridors were reverted at the user's request: in this small room everything sat inside the inflation.
# Back to inflation 0.8 m / scaling 2.5 (shared params) and cost_travel_multiplier 3.
# Then, planner side ONLY (global costmap + Smac; the local costmap and MPPI keep their defaults), measured offline
# on globalmap_261002 with plan_eval/ (20 start/goal pairs): inflation 1.1 m / scaling 1.5 / travel 5 raised the
# median minimum clearance of the paths 0.78 -> 0.86 m and the mean 1.05 -> 1.18 m for +6 % length.
gc['inflation_layer']['inflation_radius'] = 1.1; gc['inflation_layer']['cost_scaling_factor'] = 1.5
# Keep the map's in-between values as costs (not just free / wall): the "driven path" preference map (pcd2grid.py --track,
# <map>_pref.yaml) makes cells away from the hand-driven path a little more expensive, so the planner keeps to that path
# when it can. A plain free/wall/unknown map gives exactly the same costmap as before.
gc['static_layer']['trinary_costmap'] = False
p['planner_server']['ros__parameters']['GridBased']['cost_travel_multiplier'] = 5.0
# MPPI weights stay at the Clearpath defaults (cost 3.81, path align 14, path follow 5): cost 6 / align 8 / follow 3
# made the controller crawl (0.05-0.15 m/s, half the commands zero) in this room, where everything lies inside the
# inflation, and the progress checker aborted every 10 s. Progress check relaxed for slow manoeuvres near obstacles.
pc = p['controller_server']['ros__parameters']['progress_checker']
pc['required_movement_radius'] = 0.3; pc['movement_time_allowance'] = 15.0
p['planner_server']['ros__parameters']['GridBased']['max_planning_time'] = 1.0
gc['plugins'] = [x for x in gc['plugins'] if x != 'obstacle_layer']
# ...but on a SAVED map (real_loc_up.sh sets OBSTACLES_IN_PLAN=1) the planner gets live obstacles again (2026-10-14): it
# only had the map, so a person standing on the path was invisible to it and the controller alone could not find the way
# round (it stalled 0.8 m in front with a 1.3 m gap free on the other side, then "failed to make progress"). Kept out while
# MAPPING (real_up.sh): there the map itself drifts and the layer left ghosts (simulation, 2026-09-30).
if os.environ.get('OBSTACLES_IN_PLAN', '0') == '1':
    gc['plugins'] = [q for q in gc['plugins'] if q != 'inflation_layer'] + ['obstacle_layer', 'inflation_layer']
# left side = object-pose camera (it looks to the robot's left). Measured 2026-10-06: the camera sits 0.32 m left of
# the centre line (+-5 cm), i.e. inside the 0.38 m half width, so 0.1 m extra on that side covers it and its mount.
# (It was 0.5 m, which made the local footprint wider than long, 1.04 x 1.26 m.) Only the local footprint (MPPI, the
# collision monitor, BackUp/Spin) is extended; the local inflation must reach the new corner radius or the costmap
# logs an error on every update. The global planner (Smac 2D, circular robot) is unchanged.
LEFT_MARGIN = 0.1
lc['footprint'] = f'[ [0.52, {0.38 + LEFT_MARGIN}], [0.52, -0.38], [-0.52, -0.38], [-0.52, {0.38 + LEFT_MARGIN}] ]'
lc['inflation_layer']['inflation_radius'] = round(math.hypot(0.52, 0.38 + LEFT_MARGIN) + 0.03, 2)
# Arrival tolerances (env XY_GOAL_TOL / YAW_GOAL_TOL). The goal checker stops the robot as soon as it is inside them, so
# they are also how far off it ends: with 0.25 m / 0.25 rad the four arrivals of 2026-10-12 were 13-22 cm and 10-13 deg
# off. xy 0.25 -> 0.35 m (user, 2026-10-12): a goal 0.6 m from a wall left the robot shuffling 0.31 m short for 72 s.
import os
gch = p['controller_server']['ros__parameters']['general_goal_checker']
gch['xy_goal_tolerance'] = float(os.environ.get('XY_GOAL_TOL', 0.35))
gch['yaw_goal_tolerance'] = float(os.environ.get('YAW_GOAL_TOL', 0.25))
# Global planner (env PLANNER=hybrid|2d, default hybrid since 2026-10-12). Smac 2D plans for a point and ignores the goal
# heading, so its paths hook into the goal sideways and the skid-steer robot shuffled there. Smac Hybrid-A* plans x, y and
# heading with the real footprint (Reeds-Shepp: it may reverse; PLANNER_MOTION=DUBIN for forward only) and arrives along the
# goal heading. Offline on globalmap_261002, four legs of the 10/12 drive (plan_eval/run_approach_eval.sh): planned in
# 2-9 ms, end exactly on the goal, path heading 0.25 m before the goal 12-26 deg off instead of 27-74 deg.
if os.environ.get('PLANNER', 'hybrid') == 'hybrid':
    p['planner_server']['ros__parameters']['GridBased'] = dict(
        plugin='nav2_smac_planner::SmacPlannerHybrid', tolerance=0.25, downsample_costmap=False, allow_unknown=True,
        max_iterations=1000000, max_on_approach_iterations=1000, max_planning_time=1.0,
        motion_model_for_search=os.environ.get('PLANNER_MOTION', 'REEDS_SHEPP'), angle_quantization_bins=72,
        analytic_expansion_ratio=3.5, analytic_expansion_max_length=3.0, minimum_turning_radius=0.30,
        reverse_penalty=2.0, change_penalty=0.0, non_straight_penalty=1.2, cost_penalty=2.0, retrospective_penalty=0.015,
        lookup_table_size=20.0, cache_obstacle_heuristic=False, smooth_path=True)
yaml.safe_dump(p, open(sys.argv[1], 'w'), sort_keys=False)
print('wrote', sys.argv[1])
