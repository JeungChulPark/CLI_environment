#!/usr/bin/env python3

from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "config"
DATA = ROOT / "data"

with (CONFIG / "timestamp_sync.yaml").open() as stream:
    SYNC_POLICY = yaml.safe_load(stream)
ACTIVE_PROFILE = SYNC_POLICY["active_profile"]
PROFILE_CONFIG = SYNC_POLICY["profiles"][ACTIVE_PROFILE]

RGBD_SYNC_PARAMETERS = {
    "approx_sync": True,
    "approx_sync_max_interval": float(
        SYNC_POLICY["rtabmap"]["rgbd_max_interval"]
    ),
    "topic_queue_size": 30,
    "sync_queue_size": 30,
    # The D455 driver advertises these streams as reliable on this system.
    # Matching that QoS prevents large RGB/depth samples from being dropped.
    "qos": 1,
    "qos_camera_info": 1,
    "rgb_image_transport": "raw",
    "depth_image_transport": "raw",
}

RTABMAP_INPUT_PARAMETERS = {
    "frame_id": "base_link",
    "subscribe_rgbd": True,
    "subscribe_rgb": False,
    "subscribe_depth": False,
    "subscribe_odom": True,
    "topic_queue_size": 30,
    "sync_queue_size": 30,
    "qos": 1,
    "qos_image": 1,
    "qos_camera_info": 1,
    "qos_odom": 1,
}


def generate_launch_description():
    timing_profile = LaunchConfiguration("timing_profile")
    serial_no = LaunchConfiguration("serial_no")
    camera_fps = LaunchConfiguration("camera_fps")
    use_custom_ekf = LaunchConfiguration("use_custom_ekf")
    start_sensors = LaunchConfiguration("start_sensors")
    database_path = LaunchConfiguration("database_path")
    start_lidar = LaunchConfiguration("start_lidar")
    scan_topic = LaunchConfiguration("scan_topic")
    xsens_time_option = LaunchConfiguration("xsens_time_option")
    realsense_inter_cam_sync_mode = LaunchConfiguration(
        "realsense_inter_cam_sync_mode"
    )
    realsense_global_time_enabled = LaunchConfiguration(
        "realsense_global_time_enabled"
    )
    vlp16_time_offset = LaunchConfiguration("vlp16_time_offset")
    vlp16_timestamp_first_packet = LaunchConfiguration(
        "vlp16_timestamp_first_packet"
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "timing_profile",
            default_value=ACTIVE_PROFILE,
            choices=sorted(SYNC_POLICY["profiles"]),
            description="Timing contract propagated to the sensor stack.",
        ),
        DeclareLaunchArgument(
            "serial_no",
            default_value="",
            description=(
                "Optional D455f-A serial number propagated to the sensor stack."
            ),
        ),
        DeclareLaunchArgument(
            "camera_fps",
            default_value=str(PROFILE_CONFIG["camera_fps"]),
            choices=["30", "60"],
        ),
        DeclareLaunchArgument(
            "use_custom_ekf", default_value="true", choices=["true", "false"]
        ),
        DeclareLaunchArgument(
            "start_sensors",
            default_value="true",
            choices=["true", "false"],
            description="Start sensors, static TF and EKF with the mapping stack.",
        ),
        DeclareLaunchArgument(
            "database_path",
            default_value=str(DATA / "database" / "rtabmap.db"),
        ),
        DeclareLaunchArgument(
            "start_lidar",
            default_value=str(SYNC_POLICY["rtabmap"]["start_lidar"]).lower(),
            choices=["true", "false"],
            description=(
                "Start the Velodyne VLP-16 LiDAR with the mapping stack. "
                "The supported Laptop A profiles default to true."
            ),
        ),
        DeclareLaunchArgument(
            "scan_topic",
            default_value=str(SYNC_POLICY["rtabmap"]["scan_topic"]),
            description="LaserScan topic used by RTAB-Map; the default is VLP-16 /scan.",
        ),
        DeclareLaunchArgument(
            "xsens_time_option",
            default_value=str(PROFILE_CONFIG["xsens"]["time_option"]),
            choices=["0", "1", "2"],
            description="Xsens timestamp source for mapping sensors.",
        ),
        DeclareLaunchArgument(
            "realsense_inter_cam_sync_mode",
            default_value="auto",
            choices=["auto", "0", "1", "2", "3"],
            description=(
                "D455f sync mode: auto=profile default, 0=standalone, 1=master, "
                "2=slave, 3=full slave."
            ),
        ),
        DeclareLaunchArgument(
            "realsense_global_time_enabled",
            default_value=str(
                PROFILE_CONFIG["realsense"]["global_time_enabled"]
            ).lower(),
            choices=["true", "false"],
        ),
        DeclareLaunchArgument(
            "vlp16_time_offset",
            default_value=str(
                PROFILE_CONFIG["velodyne_vlp16"]["time_offset"]
            ),
        ),
        DeclareLaunchArgument(
            "vlp16_timestamp_first_packet",
            default_value=str(
                PROFILE_CONFIG["velodyne_vlp16"]["timestamp_first_packet"]
            ).lower(),
            choices=["true", "false"],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(ROOT / "launch" / "sensors.launch.py")),
            launch_arguments={
                "timing_profile": timing_profile,
                "serial_no": serial_no,
                "camera_fps": camera_fps,
                "use_custom_ekf": use_custom_ekf,
                "xsens_time_option": xsens_time_option,
                "realsense_inter_cam_sync_mode": realsense_inter_cam_sync_mode,
                "realsense_global_time_enabled": realsense_global_time_enabled,
            }.items(),
            condition=IfCondition(start_sensors),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(ROOT / "launch" / "obstacles.launch.py")),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(ROOT / "launch" / "vlp16.launch.py")),
            launch_arguments={
                "time_offset": vlp16_time_offset,
                "timestamp_first_packet": vlp16_timestamp_first_packet,
            }.items(),
            condition=IfCondition(start_lidar),
        ),
        Node(
            package="rtabmap_sync",
            executable="rgbd_sync",
            name="rgbd_sync",
            namespace="rtabmap",
            output="screen",
            parameters=[RGBD_SYNC_PARAMETERS],
            remappings=[
                ("rgb/image", "/camera/camera/color/image_raw"),
                ("depth/image", "/camera/camera/aligned_depth_to_color/image_raw"),
                ("rgb/camera_info", "/camera/camera/color/camera_info"),
                ("rgbd_image", "/rtabmap/rgbd_image"),
            ],
        ),
        Node(
            package="rtabmap_slam",
            executable="rtabmap",
            name="rtabmap",
            namespace="rtabmap",
            output="screen",
            parameters=[
                str(CONFIG / "rtabmap_mapping.yaml"),
                RTABMAP_INPUT_PARAMETERS,
                {"database_path": ParameterValue(database_path, value_type=str)},
            ],
            remappings=[
                ("rgbd_image", "/rtabmap/rgbd_image"),
                ("odom", "/odometry/filtered"),
                ("scan", scan_topic),
            ],
        ),
        Node(
            package="rtabmap_viz",
            executable="rtabmap_viz",
            name="rtabmap_viz",
            namespace="rtabmap",
            output="screen",
            parameters=[
                str(CONFIG / "rtabmap_mapping.yaml"),
                RTABMAP_INPUT_PARAMETERS,
            ],
            remappings=[
                ("rgbd_image", "/rtabmap/rgbd_image"),
                ("odom", "/odometry/filtered"),
                ("scan", scan_topic),
            ],
        ),
    ])
