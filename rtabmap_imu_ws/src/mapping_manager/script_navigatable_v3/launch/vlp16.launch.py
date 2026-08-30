#!/usr/bin/env python3

from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "config"


def static_tf_node(values, publish_tf):
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_velodyne",
        output="screen",
        condition=IfCondition(publish_tf),
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
    with (CONFIG / "vlp16.yaml").open() as stream:
        config = yaml.safe_load(stream)
    with (CONFIG / "timestamp_sync.yaml").open() as stream:
        sync_config = yaml.safe_load(stream)
    active_profile = sync_config["active_profile"]
    profile_config = sync_config["profiles"][active_profile]

    driver_cfg = config["driver"]
    time_cfg = profile_config["velodyne_vlp16"]
    transform_cfg = config["transform"]
    scan_cfg = config["laser_scan"]
    extrinsics = config["extrinsics"]

    device_ip = LaunchConfiguration("device_ip")
    port = LaunchConfiguration("port")
    rpm = LaunchConfiguration("rpm")
    time_offset = LaunchConfiguration("time_offset")
    timestamp_first_packet = LaunchConfiguration("timestamp_first_packet")
    scan_mode = LaunchConfiguration("scan_mode")
    scan_ring = LaunchConfiguration("scan_ring")
    publish_scan = LaunchConfiguration("publish_scan")
    publish_tf = LaunchConfiguration("publish_tf")
    start_driver = LaunchConfiguration("start_driver")
    start_transform = LaunchConfiguration("start_transform")

    calibration = (
        Path(get_package_share_directory("velodyne_pointcloud"))
        / "params"
        / "VLP16db.yaml"
    )

    return LaunchDescription([
        DeclareLaunchArgument("device_ip", default_value=str(driver_cfg["device_ip"])),
        DeclareLaunchArgument("port", default_value=str(driver_cfg["port"])),
        DeclareLaunchArgument("rpm", default_value=str(driver_cfg["rpm"])),
        DeclareLaunchArgument("time_offset", default_value=str(time_cfg["time_offset"])),
        DeclareLaunchArgument("timestamp_first_packet", default_value=str(time_cfg["timestamp_first_packet"]).lower(), choices=["true", "false"]),
        DeclareLaunchArgument(
            "scan_mode",
            default_value=str(scan_cfg["mode"]),
            choices=["all_rings", "single_ring"],
            description=(
                "all_rings projects a height-filtered cloud; single_ring is "
                "a diagnostic fallback."
            ),
        ),
        DeclareLaunchArgument("scan_ring", default_value=str(scan_cfg["ring"])),
        DeclareLaunchArgument("publish_scan", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("publish_tf", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("start_driver", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("start_transform", default_value="true", choices=["true", "false"]),
        static_tf_node(extrinsics, publish_tf),
        Node(
            package="velodyne_driver",
            executable="velodyne_driver_node",
            name="velodyne_driver_node",
            output="screen",
            condition=IfCondition(start_driver),
            parameters=[{
                "device_ip": ParameterValue(device_ip, value_type=str),
                # All supported profiles use the host clock for VLP-16 data.
                "gps_time": False,
                "time_offset": ParameterValue(time_offset, value_type=float),
                "enabled": True,
                "read_once": False,
                "read_fast": False,
                "repeat_delay": 0.0,
                "frame_id": str(driver_cfg["frame_id"]),
                "model": "VLP16",
                "rpm": ParameterValue(rpm, value_type=float),
                "port": ParameterValue(port, value_type=int),
                "timestamp_first_packet": ParameterValue(timestamp_first_packet, value_type=bool),
            }],
        ),
        Node(
            package="velodyne_pointcloud",
            executable="velodyne_transform_node",
            name="velodyne_transform_node",
            output="screen",
            condition=IfCondition(start_transform),
            parameters=[{
                "calibration": str(calibration),
                "model": "VLP16",
                "min_range": float(transform_cfg["min_range"]),
                "max_range": float(transform_cfg["max_range"]),
                "view_direction": 0.0,
                "fixed_frame": str(transform_cfg["fixed_frame"]),
                "target_frame": str(transform_cfg["target_frame"]),
                "organize_cloud": bool(transform_cfg["organize_cloud"]),
            }],
        ),
        Node(
            package="pointcloud_to_laserscan",
            executable="pointcloud_to_laserscan_node",
            name="velodyne_pointcloud_to_laserscan",
            output="screen",
            condition=IfCondition(
                PythonExpression([
                    "'", publish_scan, "' == 'true' and '",
                    scan_mode, "' == 'all_rings'",
                ])
            ),
            parameters=[{
                "target_frame": str(scan_cfg["target_frame"]),
                "transform_tolerance": float(
                    scan_cfg["transform_tolerance"]
                ),
                "min_height": float(scan_cfg["min_height"]),
                "max_height": float(scan_cfg["max_height"]),
                "angle_min": float(scan_cfg["angle_min"]),
                "angle_max": float(scan_cfg["angle_max"]),
                "angle_increment": float(scan_cfg["resolution"]),
                "scan_time": float(scan_cfg["scan_time"]),
                "range_min": float(scan_cfg["range_min"]),
                "range_max": float(scan_cfg["range_max"]),
                "use_inf": bool(scan_cfg["use_inf"]),
                "inf_epsilon": float(scan_cfg["inf_epsilon"]),
            }],
            remappings=[
                ("cloud_in", "/velodyne_points"),
                ("scan", "/scan"),
            ],
        ),
        Node(
            package="velodyne_laserscan",
            executable="velodyne_laserscan_node",
            name="velodyne_laserscan_node",
            output="screen",
            condition=IfCondition(
                PythonExpression([
                    "'", publish_scan, "' == 'true' and '",
                    scan_mode, "' == 'single_ring'",
                ])
            ),
            parameters=[{
                "ring": ParameterValue(scan_ring, value_type=int),
                "resolution": float(scan_cfg["resolution"]),
            }],
        ),
    ])
