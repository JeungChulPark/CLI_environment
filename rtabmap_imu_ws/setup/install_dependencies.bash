#!/usr/bin/env bash

set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-jazzy}"

if [[ -n "${CONDA_PREFIX:-}" ]]; then
    echo "WARNING: conda environment is active: $CONDA_PREFIX"
    echo "Deactivate conda before building ROS workspaces if colcon uses the wrong Python."
fi

sudo apt update

packages=(
    python3-colcon-common-extensions
    python3-rosdep
    python3-vcstool
    python3-argcomplete
    python3-catkin-pkg
    python3-empy
    python3-lark
    python3-yaml
    "ros-${ROS_DISTRO}-rtabmap-ros"
    "ros-${ROS_DISTRO}-realsense2-camera"
    "ros-${ROS_DISTRO}-compressed-image-transport"
    "ros-${ROS_DISTRO}-compressed-depth-image-transport"
    "ros-${ROS_DISTRO}-robot-localization"
    "ros-${ROS_DISTRO}-xsens-mti-ros2-driver"
    "ros-${ROS_DISTRO}-teleop-twist-keyboard"
    "ros-${ROS_DISTRO}-navigation2"
    "ros-${ROS_DISTRO}-nav2-bringup"
    "ros-${ROS_DISTRO}-pointcloud-to-laserscan"
    "ros-${ROS_DISTRO}-velodyne"
    "ros-${ROS_DISTRO}-velodyne-driver"
    "ros-${ROS_DISTRO}-velodyne-pointcloud"
    "ros-${ROS_DISTRO}-velodyne-laserscan"
    "ros-${ROS_DISTRO}-topic-tools"
    "ros-${ROS_DISTRO}-tf2-ros"
    "ros-${ROS_DISTRO}-rviz2"
    "ros-${ROS_DISTRO}-rosbag2-transport"
    "ros-${ROS_DISTRO}-rosbag2-storage-default-plugins"
    "ros-${ROS_DISTRO}-image-transport"
    "ros-${ROS_DISTRO}-ros-gz"
    "ros-${ROS_DISTRO}-ros-gz-bridge"
    "ros-${ROS_DISTRO}-ros-gz-sim"
    "ros-${ROS_DISTRO}-clearpath-gz"
)

available=()
missing=()
for pkg in "${packages[@]}"; do
    if apt-cache show "$pkg" >/dev/null 2>&1; then
        available+=("$pkg")
    else
        missing+=("$pkg")
    fi
done

if (( ${#available[@]} )); then
    sudo apt install -y "${available[@]}"
fi

if command -v rosdep >/dev/null 2>&1; then
    if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
        sudo rosdep init || true
    fi
    rosdep update || true
fi

if (( ${#missing[@]} )); then
    echo
    echo "These apt packages were not found in the configured apt sources:"
    printf '  %s\n' "${missing[@]}"
    echo "If you need Gazebo/Clearpath or Xsens support, install those packages/source drivers manually."
fi

echo
echo "Dependency install finished. Next:"
echo "  bash setup/build_workspace.bash"
