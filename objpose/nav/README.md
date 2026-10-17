# Husky A200 navigation on hdl_graph_slam (laptop) + RViz goals (PC)

Roles: the laptop (`ssh samsung`, 192.168.2.1, ROS 2 Jazzy) runs hdl_graph_slam, Nav2 and later the Clearpath
driver; this PC (192.168.2.2, conda env `ros_jazzy`) shows RViz2 and sends goals (`/goal_pose`). DDS domain 0, fastrtps.
Files here are copied to the laptop's `~/objpose/nav/` (scp).

## Recorded bag (no robot)
On the laptop:
```
~/objpose/nav/step1_map.sh 260915_inlong_middle 1.0          # hdl map -> maps/<s>.pcd + maps/<s>_dump
python3 ~/objpose/nav/pcd2grid.py ~/objpose/nav/maps/260915_inlong_middle_dump ~/objpose/nav/maps/260915_inlong_middle
~/objpose/nav/step2_nav_bag.sh 260915_inlong_middle 0.3      # hdl live pose + relay + Nav2 while the bag replays
~/objpose/nav/stop_nav.sh                                    # stop early
```
On this PC: `objpose/nav/rviz_nav.sh --sim`, then "2D Goal Pose" on the map.

A goal ends as "Goal failed" after ~30 s: the recorded robot does not follow Nav2's commands. Planning (`/plan`)
and the controller output (`/cmd_vel_nav`) are what can be checked with a bag.

## Pieces
- `hdl_nav.launch.py`: hdl_graph_slam_501 with graph_slam `odom_frame_id: hdl_odom`.
- `map_odom_relay.py`: hdl stamps map->odom with the last keyframe time, which goes stale while the robot stands
  still and made Nav2 fail at activation (extrapolation into the past). The relay re-broadcasts it as map->odom at
  20 Hz with the current stamp (+0.1 s), like AMCL.
- `pcd2grid.py`: keyframe clouds + optimized poses -> 2D ray casting -> PGM/YAML (floor = densest z below the sensor).
- `make_nav2_params.py` -> `nav2_husky_hdl.yaml`: nav2_bringup defaults, Husky footprint, /scan obstacle layers,
  0.5 m/s cap, no AMCL.
- Frames in bag mode: map -> odom (relay) -> base_link (hdl scan matching) -> velodyne (identity).

## With the robot (not done yet)
Husky driver: `husky-up` / `husky-down` / `husky-status` on the laptop (autostart off). The wheel odometry then owns
odom -> base_link, so hdl's scan-matching frame must be renamed and the relay must compute
map->odom = map->hdl_odom * hdl_odom->base_link(t) * (odom->base_link(t))^-1. cmd_vel goes to /a200_0881/cmd_vel.

## Navigation and live object pose together (2026-10-06)
One hdl_graph_slam serves both: the Nav2 stack owns it, the object-pose hub only attaches to it, so the hub's world
frame is Nav2's `map`.
1. laptop: `husky-up`, then `~/objpose/nav/real/real_up.sh`
2. PC: `objpose/nav/real/rviz_real.sh`
3. PC: `LAPTOP_SSH=samsung bash objpose/run_live.sh hdl-nav` -> viewer http://192.168.2.2:8765

`hdl-nav` starts `lidar/hdl_stream.py --attach 1` on the laptop (domain 0, map->hdl_odom from /hdl/tf): no second hdl
and no second VLP-16 receiver. hdl reports base_link there, so the hub's pose is the robot itself (the viewer draws
its footprint and a front arrow) and run_live.sh composes base_link->camera = base_link->velodyne (`LIDAR_X/Y/Z/YAW`,
same defaults as real_up.sh) . lidar->camera (`X_LIDAR`) into `<out>/X_base_sam.json`.
`stop_live.sh` leaves the Nav2 stack alone (it only stops hdl/driver processes whose ROS_DOMAIN_ID is not 0);
`real_down.sh` ends the object-pose stream within 5 s (no /odom).

Lidar->camera on the Husky: `rt/261006_husky/X_lidar_sam_261006_husky.json` (default of `hdl-nav`; static LiDAR-to-depth
registration, scripts and overlays in that folder). Camera lateral offset is only good to about +-5 cm (one static scene).

### Fixed map instead of SLAM (2026-10-06)
Same idea on the saved map (no hdl_graph_slam at run time):
1. laptop: `husky-up`, then `~/objpose/nav/real/real_loc_up.sh` (map `globalmap_261002`, global relocalization at start)
2. PC: `objpose/nav/real/rviz_real.sh`
3. PC: `LAPTOP_SSH=samsung bash objpose/run_live.sh loc-nav`

`loc-nav` starts `nav/real/loc_stream.py` on the laptop: it forwards `/a200_0881/localization/pose` (map -> base_link)
of map_localizer.py to the hub (port 17007, no keyframes), so object positions are in the saved map's frame in every run.

### Looking at a finished run again
`python objpose/pc/replay_run.py objpose/output/<run dir> [--speed 4] [--loop]` -> http://localhost:8767, the same viewer.
`hdl-nav` / `loc-nav` runs record the viewer events (image included, 5 Hz, `RECORD_HZ`) to `<run>/events.jsonl.gz`;
older runs are rebuilt from the logs without the camera image. Laptop Nav logs are not copied automatically
(`scp samsung:objpose/nav/real/logs/*.log <run>/nav_laptop/` before the next start overwrites them).

### One web page for objects and navigation (2026-10-07)
`real_up.sh` / `real_loc_up.sh` also start `nav/real/nav_bridge.py` (laptop, :8770); `run_live.sh hdl-nav|loc-nav` gives
the hub `--nav-bridge`, and the viewer (http://192.168.2.2:8765) gets a "주행 지도" pane: global map, local costmap
(detected obstacles), LiDAR scan, plan, robot footprint, the objects, and controls — goal (press = position, drag =
arrival heading, with a confirmation), stop, and 30 cm / 30 deg steps (Nav2 drive_on_heading / backup / spin).
Waypoint loop (2026-10-13): "순환 지점 추가" picks several points, "순환 시작" posts them to `/nav/route`; nav_bridge.py
sends them one by one (NavigateToPose action, next point 1 s after "succeeded", 1 -> n -> 1 ...) until "정지". A failed
point is tried once more, then the route stops; a single goal or a jog request also ends / is refused during it.
The PC on battery caps the GPU (~33 W): SAM-6D then takes ~5 s per frame instead of ~0.15 s.

### Calibration after the lidar turned on its mount (2026-10-12)
`rt/261012_husky/`: lidar->camera `X_lidar_sam_261012_husky.json` (default of `hdl-nav` / `loc-nav`) and
`LIDAR_YAW=-2.5447` (default in real_up.sh, real_loc_up.sh, run_live.sh, map_localizer.py). Method, robot standing still:
`cam_cap.py` / `lap_cam_cap.py` + `vlp_dump.py` capture, then `scan_turn.py` scores "the 10/06 rig with the lidar turned
by d" for every d (sharp peak at +41.0 deg for both cameras) and fits freely from there. A free global search alone
(`calib_static.py`) picked a wrong pose in this scene. Check: objects land within 1-3 cm of their 10/07 map positions
(43 cm off before); after any re-mount, "course minus heading" on a straight drive must be about 0.

### New place in one go: map + lidar mounting + lidar-camera registration (2026-10-14)
`bash objpose/new_place.sh start <map>` (laptop: `real_up.sh` mapping + `nav/real/mount_yaw.py`, PC: viewer) -> drive by
hand through the place, with three or more straight runs of 1 m+ -> `new_place.sh status` -> stop where the camera sees
structure 1-4 m away -> `new_place.sh finish` (saves the map with `freeze_map.sh`, stops, captures lidar + camera standing
still, `rt/calib_mount.py`, writes `nav/real/mount.env`, starts `real_loc_up.sh <map>` + `run_live.sh loc-nav`).
`new_place.sh calib` = only the standing-still registration (`DRY=1` to look without writing).
- `mount_yaw.py`: while the wheel odometry says "straight", course minus heading of the lidar pose = how far the lidar was
  turned on its mount (works with a wrong mounting yaw because the lidar sits on the rotation axis).
- `mount.env` (`LIDAR_YAW`, `X_LIDAR_SAM`) is now the one place the mounting lives; `real_up.sh`, `real_loc_up.sh`,
  `run_live.sh` read it (a caller's `LIDAR_YAW` / `X_LIDAR` still wins). Old values are kept as `mount.env.bak-*`.
- Decision: drive and camera agree within 3 deg -> yaw from the camera fit; they disagree -> the camera moved, yaw from the
  drive; no straight runs -> camera only (warning); bad fit (under 30 % of lidar points on the depth surface) -> no change.
- Tested 2026-10-14: the estimate on recorded drives (0 / -142 / +35 deg offsets recovered within 1 deg), `calib_mount.py`
  on the 10/13 capture (same matrix as stored), the decision cases, start's checks. NOT yet run on the robot: `start` ->
  drive -> `finish` end to end (the lidar's USB LAN adapter was down).

### Go to an object (2026-10-14)
Object table in the viewer: "앞으로 이동" sends the robot to a place where the SAM camera (it looks to the robot's left)
sees that object. hub.py remembers the robot pose of every placed estimate (`remember_view`) and gives each map object a
`view` = the usual viewing pose 0.65-1.15 m from it (`best_view`); the page uses it if that spot is still clear, else a
free spot on a 0.9 m ring around the object (body 0.55 m clear of walls / lidar points, clear line camera -> object),
shows it on the navigation map, asks, and posts the ordinary `/nav/goal`.
Checked on the 10/13 recording only (pose rebuilt from an estimate = pose stream within 0.1 cm median; chosen spots have
the object 70-101 deg to the left). Not yet clicked in a live run.

### Road: drive along the path driven by hand while mapping (2026-10-14)
`maps/<map>_track.json` (recorded by `mount_yaw.py` during `new_place.sh start`, copied at `finish`; for place_261014 it
came from the mapping run's viewer recording). `real_loc_up.sh` passes it as `ROAD_TRACK` (`ROAD=0` = off) and
`nav_bridge.py` sends any goal or route point within 0.9 m of the road ALONG it with nav2 FollowPath (`road.py`: smoothed,
5 cm points, shorter way round a closed loop, joins 0.5 m ahead); other goals use NavigateToPose. The viewer draws the
road in light purple. Why: the planner's own line plus MPPI corner cutting gave an S on every leg (plan 0.30 m mean
off the hand-driven loop); a costmap preference (`pcd2grid.py --track`) changed nothing in this room (inflation already
saturates everything off the loop). Route: a point Nav2 gives up on within 0.6 m counts as reached (the 11:34 route
stopped after point 1 on "start occupied" 0.25 m from it). Tested against fake Nav2 actions only.
