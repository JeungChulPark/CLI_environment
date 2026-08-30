#!/usr/bin/env python3

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "config"


def generate_launch_description():
    use_custom_ekf = LaunchConfiguration("use_custom_ekf")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_custom_ekf",
            default_value="true",
            choices=["true", "false"],
            description="Fuse wheel odometry and Xsens in the V3 EKF.",
        ),
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            output="screen",
            parameters=[str(CONFIG / "ekf.yaml")],
            remappings=[("odometry/filtered", "/odometry/filtered")],
            condition=IfCondition(use_custom_ekf),
        ),
        Node(
            package="topic_tools",
            executable="relay",
            name="clearpath_filtered_odom_relay",
            output="screen",
            arguments=[
                "/a200_0881/odometry/filtered",
                "/odometry/filtered",
            ],
            condition=UnlessCondition(use_custom_ekf),
        ),
    ])
