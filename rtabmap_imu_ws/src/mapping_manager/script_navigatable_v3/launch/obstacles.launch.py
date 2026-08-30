#!/usr/bin/env python3

from pathlib import Path

from launch import LaunchDescription
from launch_ros.actions import Node


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "config"


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="rtabmap_util",
            executable="point_cloud_xyz",
            name="point_cloud_xyz",
            output="screen",
            parameters=[str(CONFIG / "obstacles.yaml")],
            remappings=[
                ("depth/image", "/camera/camera/aligned_depth_to_color/image_raw"),
                ("depth/camera_info", "/camera/camera/color/camera_info"),
                ("cloud", "/camera/depth_cloud"),
            ],
        ),
        Node(
            package="rtabmap_util",
            executable="obstacles_detection",
            name="obstacles_detection",
            output="screen",
            parameters=[str(CONFIG / "obstacles.yaml")],
            remappings=[
                ("cloud", "/camera/depth_cloud"),
                ("obstacles", "/camera/obstacles"),
                ("ground", "/camera/ground"),
            ],
        ),
        Node(
            package="pointcloud_to_laserscan",
            executable="pointcloud_to_laserscan_node",
            name="pointcloud_to_laserscan",
            output="screen",
            parameters=[str(CONFIG / "obstacles.yaml")],
            remappings=[
                ("cloud_in", "/camera/depth_cloud"),
                ("scan", "/camera/depth_scan"),
            ],
        ),
    ])
