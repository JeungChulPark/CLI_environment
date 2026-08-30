#!/usr/bin/env python3

from pathlib import Path

import yaml
from launch import LaunchDescription
from launch_ros.actions import Node


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "config"


def static_tf_node(name, values):
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name=name,
        output="screen",
        arguments=[
            "--x", str(values["x"]),
            "--y", str(values["y"]),
            "--z", str(values["z"]),
            "--roll", str(values["roll"]),
            "--pitch", str(values["pitch"]),
            "--yaw", str(values["yaw"]),
            "--frame-id", values["parent_frame"],
            "--child-frame-id", values["child_frame"],
        ],
    )


def generate_launch_description():
    with (CONFIG / "extrinsics.yaml").open() as stream:
        extrinsics = yaml.safe_load(stream)

    return LaunchDescription([
        static_tf_node("base_to_camera", extrinsics["camera"]),
        static_tf_node("base_to_xsens", extrinsics["imu"]),
    ])
