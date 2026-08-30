#!/usr/bin/env python3

from pathlib import Path
import sys

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml


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
    "qos": 1,
    "qos_camera_info": 1,
    "rgb_image_transport": "raw",
    "depth_image_transport": "raw",
}


def generate_launch_description():
    timing_profile = LaunchConfiguration("timing_profile")
    serial_no = LaunchConfiguration("serial_no")
    camera_fps = LaunchConfiguration("camera_fps")
    use_custom_ekf = LaunchConfiguration("use_custom_ekf")
    start_sensors = LaunchConfiguration("start_sensors")
    database_path = LaunchConfiguration("database_path")
    map_yaml = LaunchConfiguration("map")
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

    nav2_launch = (
        Path(get_package_share_directory("nav2_bringup"))
        / "launch"
        / "navigation_launch.py"
    )
    nav2_params = RewrittenYaml(
        source_file=str(CONFIG / "nav2_params.yaml"),
        param_rewrites={
            "default_nav_to_pose_bt_xml": str(
                ROOT / "behavior_trees" / "nav_to_pose_precise.xml"
            )
        },
        convert_types=True,
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
            description="Start sensors, static TF and EKF with navigation.",
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
                "Start the Velodyne VLP-16 LiDAR with navigation. "
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
            description="Xsens timestamp source for navigation sensors.",
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
        DeclareLaunchArgument(
            "map",
            default_value=str(DATA / "maps" / "map.yaml"),
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
        # Keep the VLP-16 as the 360-degree obstacle and scan source.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(ROOT / "launch" / "vlp16.launch.py")),
            launch_arguments={
                "time_offset": vlp16_time_offset,
                "timestamp_first_packet": vlp16_timestamp_first_packet,
            }.items(),
            condition=IfCondition(start_lidar),
        ),
        # VLP-16 is configured with a 0.9 m minimum range. Generate a compact
        # front RGB-D cloud as a complementary close-obstacle source for both
        # Nav2 costmaps and the collision monitor.
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
                str(CONFIG / "rtabmap_localization.yaml"),
                {"database_path": ParameterValue(database_path, value_type=str)},
            ],
            remappings=[
                ("rgbd_image", "/rtabmap/rgbd_image"),
                ("odom", "/odometry/filtered"),
                ("scan", scan_topic),
                # Nav2 RViz publishes the 2D Pose Estimate on /initialpose.
                # Without this remap, the namespaced RTAB-Map node listens on
                # /rtabmap/initialpose and silently misses the operator input.
                ("initialpose", "/initialpose"),
            ],
        ),
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[
                {
                    "yaml_filename": ParameterValue(map_yaml, value_type=str),
                    "topic_name": "map",
                    "frame_id": "map",
                }
            ],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_static_map",
            output="screen",
            parameters=[
                {
                    "autostart": True,
                    "node_names": ["map_server"],
                }
            ],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(nav2_launch)),
            launch_arguments={
                "use_sim_time": "false",
                "autostart": "true",
                "use_composition": "False",
                "use_respawn": "True",
                "params_file": nav2_params,
            }.items(),
        ),
        ExecuteProcess(
            cmd=[sys.executable, str(ROOT / "nodes" / "cmd_vel_bridge.py")],
            # This process is also the RTAB-Map localization readiness gate;
            # keep lock/unlock diagnostics visible to the operator.
            output="screen",
        ),
    ])
