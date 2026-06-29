# src_for_copy

This directory is the reduced source set to copy into a clean RTAB-Map ROS 2
workspace `src` directory.

Included packages:

- `rtabmap`
- `rtabmap_ros/rtabmap_conversions`
- `rtabmap_ros/rtabmap_launch`
- `rtabmap_ros/rtabmap_msgs`
- `rtabmap_ros/rtabmap_odom`
- `rtabmap_ros/rtabmap_ros`
- `rtabmap_ros/rtabmap_slam`
- `rtabmap_ros/rtabmap_sync`
- `rtabmap_ros/rtabmap_util`

Excluded source not needed by `ros2 launch rtabmap_ros rtab_slam.launch.py`:

- RTAB-Map standalone app, GUI library, tools, and examples
- `rtabmap_ros/rtabmap_demos`
- `rtabmap_ros/rtabmap_examples`
- `rtabmap_ros/rtabmap_python`
- `rtabmap_ros/rtabmap_rviz_plugins`
- `rtabmap_ros/rtabmap_viz`
- repository `.git`, `.github`, `.devcontainer`, `.settings`, docker, archive,
  and build directories

The copied `rtabmap` core defaults to a library-only build:
`BUILD_APP=OFF`, `BUILD_TOOLS=OFF`, `BUILD_EXAMPLES=OFF`, `WITH_QT=OFF`.

The copied `rtabmap_odom` and `rtabmap_util` metadata also omit the unused
`pcl_ros` dependency, because these packages do not include `pcl_ros` headers
in this workflow.

Remote refresh flow:

```bash
cd ~/SSH_PC/rtabmap_ws
rm -rf src install log build
mkdir -p src
cp -a src_for_copy/. src/
colcon build
source install/setup.bash
ros2 launch rtabmap_ros rtab_slam.launch.py config:=/path/to/config.yaml
```
