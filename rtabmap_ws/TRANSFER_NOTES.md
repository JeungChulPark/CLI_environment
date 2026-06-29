# rtabmap_ws transfer notes

## Assumptions

The target PC is assumed to already have the required base environment prepared:

- ROS 2 Humble is installed.
- The shell sources ROS Humble before building, for example `source /opt/ros/humble/setup.bash`.
- Any Python virtual environment or user-level runtime environment used on the target PC is already available or can be recreated separately.

This note only covers additional package dependencies and local path assumptions found in this workspace.

## apt dependencies required before `colcon build`

When checked with ROS Humble sourced, `rosdep check --from-paths src --ignore-src -r` reported the following missing system dependencies:

```bash
sudo apt install \
  ros-humble-apriltag-msgs \
  ros-humble-aruco-msgs \
  ros-humble-aruco-opencv-msgs \
  ros-humble-octomap-msgs \
  ros-humble-grid-map-ros
```

After installing them, the normal build flow on the target PC should be:

```bash
cd ~/rtabmap_ws
rm -rf build install log
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

## Files intentionally excluded from transfer archive

The archive excludes generated colcon artifacts:

- `rtabmap_ws/build/`
- `rtabmap_ws/install/`
- `rtabmap_ws/log/`

These directories are machine-specific build outputs and should be regenerated on the target PC.

## Hard-coded path issue in scripts

`rtabmap_ws/scripts/run_four_slam_bags.py` contains local absolute path assumptions through `Path.home()` plus fixed subpaths:

```python
RTABMAP_WS = HOME / "CLI_environment" / "rtabmap_ws"
ORB_DATA = HOME / "CLI_environment" / "orbslam_ws" / "data"
OUTPUT_ROOT = RTABMAP_WS / "output"
ENV_PREFIX = "source /home/etri/CLI_environment/rtabmap_ws/install/setup.bash && "
```

On another PC, this script will only work unchanged if the same user/path layout exists:

```text
/home/<user>/CLI_environment/rtabmap_ws
/home/<user>/CLI_environment/orbslam_ws/data
```

However, `ENV_PREFIX` is explicitly hard-coded to `/home/etri/CLI_environment/rtabmap_ws/install/setup.bash`, so it must be edited unless the target PC uses exactly that path.

Recommended fix before use on another PC:

- Replace the hard-coded workspace path with a value derived from the script location, an environment variable, or a command-line argument.
- Replace `ENV_PREFIX` with the target workspace's `install/setup.bash` path.
- Confirm the bag data directory used by `ORB_DATA` exists on the target PC.

`rtabmap_ws/scripts/export_rtabmap_outputs.py` is more portable because it accepts output and input paths via arguments, but it still requires the ROS environment and Python dependencies to be available in the shell where it is launched.
