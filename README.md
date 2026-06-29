# CLI_environment

각 워크스페이스는 복사 후 ROS 2 Humble 환경과 해당 conda 환경을 활성화한 뒤 `colcon build`하고 launch하면 된다.

## ORB-SLAM3

```bash
cd orbslam_ws
source ~/miniconda3/etc/profile.d/conda.sh
conda activate orbslam3
source /opt/ros/humble/setup.bash
colcon build --symlink-install --executor sequential --parallel-workers 1 --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch orbslam3_ros2 orb_slam.launch.py config:=no_cli_rgbd.yaml
```

## HDL-Graph-SLAM

`hdlgraphslam_ws/data/vlp16_bag/` is intentionally excluded from Git because
the input bag is larger than GitHub LFS Free/Pro's per-file limit. Copy or
download a compatible VLP16 ROS 2 bag into that path before running the launch
command.

```bash
cd hdlgraphslam_ws
source ~/miniconda3/etc/profile.d/conda.sh
conda activate hdl_graph_slam_humble
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch hdl_graph_slam hdl_bag_run.launch.py config:=no_cli_vlp16.yaml
```

## SAM-6D

```bash
cd sam6d_ws
source ~/miniconda3/etc/profile.d/conda.sh
conda activate sam6d_test
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch sam6d_ros sam6d_inference.launch.py config:=no_cli_milk.yaml
```
