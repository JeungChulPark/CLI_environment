#!/usr/bin/env python3

from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


ROOT = Path(__file__).resolve().parents[1]
SYNC_CONFIG = ROOT / "config" / "timestamp_sync.yaml"


def generate_launch_description():
    with SYNC_CONFIG.open() as stream:
        sync_config = yaml.safe_load(stream)
    active_profile = sync_config["active_profile"]
    xsens_sync = sync_config["profiles"][active_profile]["xsens"]

    xsens_params = (
        Path(get_package_share_directory("xsens_mti_ros2_driver"))
        / "param"
        / "xsens_mti_node.yaml"
    )
    time_option = LaunchConfiguration("time_option")

    return LaunchDescription([
        DeclareLaunchArgument(
            "time_option",
            default_value=str(xsens_sync["time_option"]),
            choices=["0", "1", "2"],
            description=(
                "Xsens timestamp source: 0=MTi UTC, 1=MTi SampleTimeFine, "
                "2=host ROS callback time. The supported profiles use host time."
            ),
        ),
        Node(
            package="xsens_mti_ros2_driver",
            executable="xsens_mti_node",
            name="xsens_mti_node",
            output="screen",
            parameters=[
                str(xsens_params),
                {
                    "time_option": ParameterValue(time_option, value_type=int),
                    "frame_id": "imu_link",
                    "orientation_stddev": [0.03, 0.03, 0.08],
                    "angular_velocity_stddev": [0.01, 0.01, 0.015],
                    "linear_acceleration_stddev": [0.10, 0.10, 0.15],
                    "enable_deviceConfig": False,
                    "pub_sampletime": True,
                    "pub_utctime": True,
                    "pub_status": True,
                    # static_tf.launch.py owns base_link -> imu_link. Publishing
                    # the driver's world -> imu_link transform gives imu_link two
                    # parents and makes the sensor tree ambiguous.
                    "pub_transform": False,
                },
            ],
        ),
    ])
