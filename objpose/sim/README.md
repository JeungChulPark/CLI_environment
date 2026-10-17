# Husky A200 Gazebo simulation (laptop) + RViz (PC)

Gazebo Harmonic + `ros-jazzy-clearpath-simulator` on the samsung laptop (192.168.2.1), copied to `~/objpose/sim`.
The simulated robot is `robot.yaml` here (A200 + sensor arch + VLP-16 + IMU, namespace `a200_0881`), copied to
`~/clearpath_sim/` at start; the real robot's `/etc/clearpath` is not touched.

Laptop (`ssh samsung`):
```
~/objpose/sim/sim_up.sh [world] [--headless]   # world: warehouse(default) office construction orchard pipeline solar_farm
~/objpose/sim/sim_nav.sh                       # slam_toolbox mapping + Nav2 (scan = sensors/lidar3d_0/scan)
~/objpose/sim/sim_hdl_nav.sh                   # OR hdl_graph_slam on the VLP-16 points + relay + live grid + Nav2
~/objpose/sim/sim_down.sh                      # stop everything
python3 ~/objpose/sim/probe.py 10              # RTF + sensor rates
python3 ~/objpose/sim/drive_test.py            # 0.5 m/s for 4 s
python3 ~/objpose/sim/goal_test.py X Y         # one NavigateToPose goal in map frame
```
PC:
- `objpose/sim/gz_gui_pc.sh`: Gazebo window on the PC attached to the laptop server over gz-transport
  (GZ_IP/GZ_PARTITION=husky_sim; conda env `gz_harmonic` = conda-forge gz-sim8; meshes in ~/gz_assets/share;
  Fuel models via the shared absolute cache /tmp/husky_sim_fuel because the server sends absolute mesh paths).
- `objpose/sim/rviz_sim.sh`: RViz in namespace /a200_0881, sim time. "Nav2 Goal" sends the goal to the laptop.

Navigation test: `~/objpose/sim/nav_test.sh warehouse x1,y1 x2,y2 ...` → per-goal CSV in logs/ (result, time,
distance, arrival error and SLAM localization error against Gazebo ground truth).
Nav2 runs with nav2_sim.yaml from make_nav2_sim_params.py (Clearpath demo params + VLP-16 scan topic,
60 m global costmap, lighter MPPI at 15 Hz).

Measured 2026-09-30 (warehouse, Intel Lunar Lake iGPU):
- with Gazebo window: RTF 0.9–1.0, lidar ~14–15 Hz, GUI alone uses one full core
- `--headless`: RTF 1.00, lidar ~20 Hz, gz server ~50 % of one core
- Nav2 goals (4,1), (0,0), (5,-3) all SUCCEEDED, including one sent from the PC
- Nav test tour 2026-09-30 (10,0 11,-10 2,-10 -8,2 -12,-1 5,6 0,0): 7/7 SUCCEEDED, 88.8 m, arrival error
  0.02–0.14 m, SLAM localization error ≤ 0.055 m / ≤ 3.5°. Before the param changes: 1/8 (20 m rolling global
  costmap rejected far goals; MPPI 2000×56 ran at ~8 Hz and got stuck near a shelf).

## hdl_graph_slam variant (sim_hdl_nav.sh)
- `hdl_sim.launch.py`: ~/objpose/nav/hdl_nav.launch.py with all hdl TF moved to /hdl/tf (its scan-matching
  odom->base_link would fight the EKF's), floor detection on (without it the map floor spread over -1.0..+0.2 m).
- `hdl_map_relay.py`: map->odom for the robot tree = map->hdl_odom * hdl_odom->base_link(t) * inv(odom->base_link(t)).
  The same composition is what the real robot needs (wheel odom owns odom->base_link).
- `hdl_grid.py`: /hdl_graph_slam/map_points -> /a200_0881/map (floor = free, 0.35–1.8 m above floor with >= 3 points =
  occupied); an all-unknown map is published until hdl has map points.
- `lidar_bridge.sh`: replaces the Clearpath lidar gz->ROS bridge. With RViz on the PC pulling the raw VLP-16 cloud
  (0.43 MB x 20 Hz over the LAN) the original bridge fell behind and scans reached hdl/Nav2 3–46 s late; RViz now has
  the raw cloud display off by default.
- Nav test 2026-09-30 after the fixes (11,-10 2,-10 -8,2 -12,-1 5,6 0,0): 5/6 SUCCEEDED, 68.6 m, arrival error
  0.08–0.38 m, hdl localization error 0.04–0.29 m (slam_toolbox: <= 0.055 m). (5,6) failed on a planner timeout.
