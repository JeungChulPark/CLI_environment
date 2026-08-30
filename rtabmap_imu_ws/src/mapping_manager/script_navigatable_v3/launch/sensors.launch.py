#!/usr/bin/env python3

from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


ROOT = Path(__file__).resolve().parents[1]
LAUNCH = ROOT / "launch"


def include_launch(filename, arguments=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(LAUNCH / filename)),
        launch_arguments=(arguments or {}).items(),
    )


def generate_launch_description():
    with (ROOT / "config" / "timestamp_sync.yaml").open() as stream:
        sync_config = yaml.safe_load(stream)
    active_profile = sync_config["active_profile"]
    profile_config = sync_config["profiles"][active_profile]

    timing_profile = LaunchConfiguration("timing_profile")
    serial_no = LaunchConfiguration("serial_no")
    camera_fps = LaunchConfiguration("camera_fps")
    use_custom_ekf = LaunchConfiguration("use_custom_ekf")
    xsens_time_option = LaunchConfiguration("xsens_time_option")
    realsense_inter_cam_sync_mode = LaunchConfiguration(
        "realsense_inter_cam_sync_mode"
    )
    realsense_global_time_enabled = LaunchConfiguration(
        "realsense_global_time_enabled"
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "timing_profile",
            default_value=active_profile,
            choices=sorted(sync_config["profiles"]),
            description="Timing contract propagated to all sensor launch files.",
        ),
        DeclareLaunchArgument(
            "serial_no",
            default_value="",
            description=(
                "Optional D455f-A serial number propagated to the RealSense node."
            ),
        ),
        DeclareLaunchArgument(
            "camera_fps",
            default_value=str(profile_config["camera_fps"]),
            choices=["30", "60"],
            description="Matched RGB and depth FPS.",
        ),
        DeclareLaunchArgument(
            "use_custom_ekf",
            default_value="true",
            choices=["true", "false"],
            description="Fuse wheel odometry and Xsens in the V3 EKF.",
        ),
        DeclareLaunchArgument(
            "xsens_time_option",
            default_value=str(profile_config["xsens"]["time_option"]),
            choices=["0", "1", "2"],
            description=(
                "Xsens timestamp source: 0=MTi UTC, 1=SampleTimeFine, "
                "2=host ROS time. The supported profiles use host ROS time."
            ),
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
                profile_config["realsense"]["global_time_enabled"]
            ).lower(),
            choices=["true", "false"],
        ),
        include_launch("static_tf.launch.py"),
        TimerAction(
            period=1.0,
            actions=[include_launch(
                "camera.launch.py",
                {
                    "timing_profile": timing_profile,
                    "serial_no": serial_no,
                    "camera_fps": camera_fps,
                    "inter_cam_sync_mode": realsense_inter_cam_sync_mode,
                    "global_time_enabled": realsense_global_time_enabled,
                },
            )],
        ),
        TimerAction(
            period=4.0,
            actions=[include_launch(
                "xsens.launch.py",
                {"time_option": xsens_time_option},
            )],
        ),
        TimerAction(
            period=6.0,
            actions=[include_launch(
                "ekf.launch.py",
                {"use_custom_ekf": use_custom_ekf},
            )],
        ),
    ])
