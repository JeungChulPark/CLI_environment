# Mapping & Navigation V3 — Operating Guide

This guide is intended for operating the Husky A200 system with an
Intel RealSense D455f camera, an Xsens MTi-630 IMU, and a Velodyne VLP-16
LiDAR.

## 1. Open the workspace

Open a terminal in the project directory:

```bash
cd /path/to/setup_ws/src/mapping_manager/script_navigatable_v3
```

The project scripts automatically source ROS and the workspace through
`scripts/env.bash`.

## 2. Run the system

### Recommended method: use the menu

```bash
bash run.bash
```

Common menu options:

| Option | Function |
|---|---|
| `1` | Run the complete automatic mapping workflow |
| `2` | Start mapping manually |
| `3` | Control the Husky with the keyboard |
| `4` | Save the static 2D map |
| `5` | Export the 3D point cloud |
| `6` | Start localization and navigation |
| `7` | Check topics, TF, and the safety command chain |
| `8` | Open Nav2 RViz |
| `9` | Edit/check sensor extrinsics |
| `10` | Record mapping topics |
| `11` | Stop rosbag recording |
| `12` | Start the VLP-16 |
| `13` | Check the VLP-16 |

The automatic workflow requires `gnome-terminal`.

### Run automatic mapping directly

```bash
bash scripts/auto_mapping.bash
```

The script starts the camera, IMU, EKF, LiDAR, RTAB-Map, data checks, rosbag
recording, and keyboard teleoperation. Drive the robot slowly while mapping:

- Keep linear velocity at or below `0.4 m/s`.
- Keep angular velocity at or below `0.45 rad/s`.
- Follow a closed route and return to previously mapped areas so that RTAB-Map
  can create loop closures.

When the map has sufficient coverage, return to the main terminal and press
`Enter`. The script will guide you through saving the map and exporting the
point cloud.

### Run mapping manually

Terminal 1:

```bash
bash scripts/start_mapping.bash
```

Terminal 2:

```bash
bash scripts/teleop.bash
```

### Run localization and navigation

Only start navigation after both the RTAB-Map database and the static 2D map
have been created:

```bash
bash scripts/start_navigation.bash
```

Open RViz in another terminal:

```bash
source scripts/env.bash
ros2 launch nav2_bringup rviz_launch.py
```

### Select a camera or timing profile

Select a camera by serial number:

```bash
CAMERA_SERIAL=123456789 bash scripts/auto_mapping.bash
```

Run the master or slave computer in a dual-camera system:

```bash
TIMING_PROFILE=dual_camera_master bash scripts/auto_mapping.bash
```

```bash
TIMING_PROFILE=dual_camera_slave bash scripts/auto_mapping.bash
```

The default single-computer profile is `software_single_host`.

## 3. Save the map

### Save the static 2D map

Run this command while the RTAB-Map mapping terminal is still active:

```bash
bash scripts/save_2d_map.bash
```

Output files:

```text
data/maps/map.yaml
data/maps/map.pgm
```

### Export the 3D point cloud

After saving the 2D map, stop mapping with `Ctrl+C` so that RTAB-Map can finish
writing its database. Then run:

```bash
bash scripts/export_maps.bash
```

Main output:

```text
data/maps/rtabmap_cloud.ply
```

Database used for localization:

```text
data/database/rtabmap.db
```

Whenever `scripts/start_mapping.bash` starts a new mapping session, the
existing database is moved to:

```text
data/database/rtabmap_backup_YYYYMMDD_HHMMSS.db
```

Saving a new map may overwrite `map.yaml` and `map.pgm`. Back up the
`data/maps` directory if an approved map must be retained.

## 4. Check the system

Run the complete pipeline check:

```bash
bash scripts/check_pipeline.bash
```

Check the LiDAR separately:

```bash
bash scripts/check_lidar.bash
```

Before running the manual ROS commands in the troubleshooting section, source
the environment:

```bash
source scripts/env.bash
```

## 5. Troubleshooting

### `gnome-terminal is required`

Install `gnome-terminal` or operate the system in a desktop environment where
it is available. If the automatic workflow cannot be used, start mapping and
teleoperation manually in two terminals as described above.


### No wheel odometry on `/a200_0881/platform/odom`

Restart and inspect the Clearpath platform service:

```bash
sudo systemctl restart clearpath-platform.service
sudo systemctl status clearpath-platform.service
```

Check the odometry data:

```bash
ros2 topic echo /a200_0881/platform/odom --once --qos-profile sensor_data
```

### The camera does not publish images or `CameraInfo`

Check the USB 3 cable, camera power, and selected serial number. Then inspect
the camera topics:

```bash
ros2 topic list | grep /camera
ros2 topic echo /camera/camera/color/image_raw --once --no-arr
ros2 topic echo /camera/camera/aligned_depth_to_color/image_raw --once --no-arr
```

If multiple cameras are connected, restart the workflow with the correct
`CAMERA_SERIAL`.

### No data on `/imu/data`

Check the Xsens USB connection and make sure an old driver process is not
holding the device:

```bash
ros2 topic echo /imu/data --once --no-arr
```

Close any old Xsens terminal and restart the workflow.

### EKF covariance does not converge or TF `odom -> base_link` is missing

First confirm that wheel odometry, IMU data, and filtered odometry are
available:

```bash
ros2 topic echo /a200_0881/platform/odom --once --qos-profile sensor_data
ros2 topic echo /imu/data --once --no-arr
ros2 topic echo /odometry/filtered --once --no-arr
```

Only one EKF should publish the `odom -> base_link` transform. Close old
mapping or navigation sessions and restart the workflow. Check the transform
with:

```bash
timeout 8 ros2 run tf2_ros tf2_echo odom base_link
```

### The VLP-16 does not publish `/velodyne_points` or `/scan`

Check the network connection:

```bash
ping 192.168.1.201
```

If the interface or IP addresses differ from the defaults, provide the correct
values when configuring the LiDAR network:

```bash
sudo env \
  LIDAR_INTERFACE=enx001122334455 \
  LIDAR_HOST_IP=192.168.1.100 \
  LIDAR_IP=192.168.1.201 \
  bash scripts/setup_vlp16_network.bash
```

Start the LiDAR in terminal 1:

```bash
bash scripts/start_lidar.bash
```

Check it in terminal 2:

```bash
bash scripts/check_lidar.bash
```

If the check reports multiple publishers for one topic, close old LiDAR
terminals and run only one VLP-16 stack.

### `/rtabmap/rgbd_image` is slower than `2 Hz`

Check the RGB and depth rates and run the complete pipeline check:

```bash
ros2 topic hz /camera/camera/color/image_raw
ros2 topic hz /camera/camera/aligned_depth_to_color/image_raw
bash scripts/check_pipeline.bash
```

Close duplicate camera drivers or RTAB-Map sessions, check the USB 3
connection, and restart the workflow.

### The 2D map cannot be saved

The save command must run while RTAB-Map is still active. Check that its map
service exists:

```bash
ros2 service list | grep /rtabmap/rtabmap/get_map
```

If the service is unavailable, start mapping, wait for `/rtabmap/map`, and try
again:

```bash
ros2 topic list | grep /rtabmap/map
bash scripts/save_2d_map.bash
```

### Navigation reports an invalid map/database pair

Check that all three files exist:

```bash
ls -lh \
  data/database/rtabmap.db \
  data/maps/map.yaml \
  data/maps/map.pgm
```

If the quality check reports that the map has no global constraints, create a
new map using a closed route and confirm a successful loop closure. Do not use
an unoptimized map to evaluate navigation accuracy.
