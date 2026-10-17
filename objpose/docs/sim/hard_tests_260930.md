# Husky A200 sim: hard navigation scenarios on hdl_graph_slam (2026-09-30)

Stack: laptop Gazebo warehouse (headless) + VLP-16 -> hdl_graph_slam (floor detection) -> hdl_map_relay (map->odom)
-> hdl_grid (2D map from hdl map points) -> Nav2 (nav2_sim.yaml: SmacPlanner2D, MPPI 1000x40 @15 Hz, local costmap
scan-only). Harness: `objpose/sim/hard_tests.py` (ground truth from Gazebo). Rows: `objpose/sim/logs/hard_hdl2.csv`
(turn_around / tiny_goal come from the first run, which stopped on a harness bug before writing its CSV).
Images: `map_after.png` (hdl 2D grid), `gcost_after_c.png` (global costmap) after all scenarios.

| scenario | expected | result | notes |
|---|---|---|---|
| turn_around (1 m behind, yaw 180) | turn in place | ABORTED 8 s | BT: planner ack timeout; yaw err 17.7 deg |
| tiny_goal (0.15 m) | done at once | SUCCEEDED | inside tolerance, no motion |
| far_corner (12,-20) | reach | ABORTED at 4.4 m | loc err 0.75 m, planner ack timeout |
| long_corridor (2,-17) | reach | SUCCEEDED | arrival 0.5 m off, loc err 0.49 m |
| narrow_gap (0.8 m < footprint) | refuse | ABORTED 25 s | right outcome, but 14 replans before giving up |
| goal_in_obstacle (shelf) | refuse fast | ABORTED 24 s | same: "no valid path" x14 |
| goal_outside (22,0) | refuse | TIMEOUT 150 s | drove 53 m and left the building through a gap (planning through unknown) |
| popup_obstacle (box at 5,0 after 4 s) | detour | ABORTED | planner ack timeout, robot ended at (6.8,5.2) |
| crowd_boxes (slalom, 4 boxes) | pass | SUCCEEDED | 6.1 m, arrival 0.09 m |
| blocked_corridor (wall at y=-6) | give up | ABORTED 4 s | aborted by ack timeout, not by "no path" |
| preempt (new goal after 4 s) | switch | SUCCEEDED | |
| clutter (table/chairs corner) | reach | ABORTED at 2.5 m | planner ack timeout |
| laps (3 x 5 goals) | no drift | 4/15 SUCCEEDED | 12 ack timeouts; loc err 0.02-0.53 m |
| kidnap (teleport 4.5 m) | recover | ABORTED | hdl never relocalized: loc err 2.3 m after 5 s, 7.8 m / 80 deg at the end |

## Problems found
1. **Planner ack timeouts abort goals (main failure, 20+ goals).** bt_navigator gives up when planner_server does not
   acknowledge within its server timeout. Load average 6.8 on 8 cores; `hdl_grid.py` republishes the whole grid
   (453x654 cells from 555k map points) every ~2 s and each publish makes the 1000x1000 global costmap reprocess its
   static layer while SmacPlanner2D plans on it.
2. **Obstacles that should not exist (gcost_after_c.png).**
   - The hdl map keeps every point ever seen: the spawned boxes and the test wall stay in the 2D map after removal.
   - Localization drift (0.5-0.75 m far from the start) records the same walls/shelves twice, offset.
   - The global costmap obstacle layer marks live scans in the map frame at the drifted pose; it only clears within
     the 3 m raytrace range, so wrong marks persist.
   - After the kidnap the misregistered keyframes added slanted walls across the hall.
   - Every thin line is inflated by 0.8 m, so these artefacts fill the hall with lethal area.
3. **Robot stuck next to obstacles (up to 14.8 s without motion).** Consequence of 2: start inside inflated/lethal
   cells ("Start occupied", "no valid path"), MPPI finds no collision-free rollout.
4. **Localization.** hdl error 0.1-0.75 m (slam_toolbox run: <= 0.055 m); no relocalization after a kidnap.
5. **Goals outside / in unknown space are not rejected** (allow_unknown), and goals inside obstacles take ~25 s of
   replanning to abort.
6. **Goal heading:** 10-18 deg yaw error at arrival.

## Fix candidates (not applied yet)
- Publish the grid only when hdl adds keyframes / at a low rate; raise bt_navigator `default_server_timeout`.
- Build the 2D grid by ray casting keyframes (like `objpose/nav/pcd2grid.py`) so later observations clear removed
  obstacles; limit the global obstacle layer (or clear with a longer raytrace) and keep dynamic obstacles local.
- Navigate on a fixed pre-built map with a localizer that can relocalize (e.g. hdl_localization) instead of live SLAM.
- Validate goals (reject lethal/unknown/outside) before sending them to Nav2.
