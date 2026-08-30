#!/usr/bin/env python3

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "config"
DATA = ROOT / "data"


def generate_launch_description():
    rgb_topic = LaunchConfiguration("rgb_topic")
    depth_topic = LaunchConfiguration("depth_topic")
    rgb_image_transport = LaunchConfiguration("rgb_image_transport")
    depth_image_transport = LaunchConfiguration("depth_image_transport")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    odom_topic = LaunchConfiguration("odom_topic")
    use_scan = LaunchConfiguration("use_scan")
    scan_topic = LaunchConfiguration("scan_topic")
    database_path = LaunchConfiguration("database_path")

    input_parameters = {
        "frame_id": "base_link",
        "subscribe_rgbd": True,
        "subscribe_rgb": False,
        "subscribe_depth": False,
        "subscribe_odom": True,
        "subscribe_scan": ParameterValue(use_scan, value_type=bool),
        "topic_queue_size": 30,
        "sync_queue_size": 30,
        # Bag playback commonly replays sensor streams as best-effort.
        "qos": 2,
        "qos_image": 2,
        "qos_camera_info": 2,
        "qos_odom": 2,
        "qos_scan": 2,
        "wait_for_transform": 0.5,
    }

    sync_parameters = {
        "approx_sync": True,
        "approx_sync_max_interval": 0.12,
        "topic_queue_size": 30,
        "sync_queue_size": 30,
        "qos": 2,
        "qos_camera_info": 2,
        "rgb_image_transport": ParameterValue(
            rgb_image_transport, value_type=str
        ),
        "depth_image_transport": ParameterValue(
            depth_image_transport, value_type=str
        ),
    }

    remappings = [
        ("rgb/image", rgb_topic),
        ("depth/image", depth_topic),
        ("rgb/camera_info", camera_info_topic),
        ("rgbd_image", "/rtabmap/rgbd_image"),
        ("odom", odom_topic),
        ("scan", scan_topic),
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("rtabmap_viz", default_value="true"),
            DeclareLaunchArgument(
                "rgb_topic",
                default_value="/camera/camera/color/image_raw",
            ),
            DeclareLaunchArgument(
                "depth_topic",
                default_value="/camera/camera/aligned_depth_to_color/image_raw",
            ),
            DeclareLaunchArgument("rgb_image_transport", default_value="raw"),
            DeclareLaunchArgument("depth_image_transport", default_value="raw"),
            DeclareLaunchArgument(
                "camera_info_topic",
                default_value="/camera/camera/color/camera_info",
            ),
            DeclareLaunchArgument("odom_topic", default_value="/odometry/filtered"),
            DeclareLaunchArgument("use_scan", default_value="false"),
            DeclareLaunchArgument("scan_topic", default_value="/camera/depth_scan"),
            DeclareLaunchArgument(
                "database_path",
                default_value=str(DATA / "database" / "rtabmap_rosbag.db"),
            ),
            SetParameter(name="use_sim_time", value=True),
            Node(
                package="rtabmap_sync",
                executable="rgbd_sync",
                name="rgbd_sync",
                namespace="rtabmap",
                output="screen",
                parameters=[sync_parameters],
                remappings=remappings,
            ),
            Node(
                package="rtabmap_slam",
                executable="rtabmap",
                name="rtabmap",
                namespace="rtabmap",
                output="screen",
                parameters=[
                    str(CONFIG / "rtabmap_mapping.yaml"),
                    input_parameters,
                    {"database_path": ParameterValue(database_path, value_type=str)},
                ],
                remappings=remappings,
                arguments=["-d"],
            ),
            Node(
                package="rtabmap_viz",
                executable="rtabmap_viz",
                name="rtabmap_viz",
                namespace="rtabmap",
                output="screen",
                condition=IfCondition(LaunchConfiguration("rtabmap_viz")),
                parameters=[
                    str(CONFIG / "rtabmap_mapping.yaml"),
                    input_parameters,
                ],
                remappings=remappings,
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                condition=IfCondition(LaunchConfiguration("rviz")),
            ),
        ]
    )
