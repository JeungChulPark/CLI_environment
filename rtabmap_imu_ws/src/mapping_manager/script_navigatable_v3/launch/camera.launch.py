#!/usr/bin/env python3

from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


ROOT = Path(__file__).resolve().parents[1]
SYNC_CONFIG = ROOT / "config" / "timestamp_sync.yaml"


def resolve_inter_cam_sync_mode(profile_name, requested_mode, profiles):
    if profile_name not in profiles:
        raise RuntimeError(
            f"Unknown timing profile: {profile_name}. "
            f"Expected one of: {', '.join(sorted(profiles))}."
        )

    camera_sync = profiles[profile_name]["realsense"]
    allowed_modes = tuple(
        int(mode)
        for mode in camera_sync.get(
            "allowed_inter_cam_sync_modes",
            [camera_sync["inter_cam_sync_mode"]],
        )
    )
    if requested_mode == "auto":
        mode = int(camera_sync["inter_cam_sync_mode"])
    else:
        try:
            mode = int(requested_mode)
        except ValueError as error:
            raise RuntimeError(
                "D455f inter-camera sync mode must be auto, 0, 1, 2 or 3."
            ) from error

    if mode not in allowed_modes:
        raise RuntimeError(
            f"D455f inter-camera sync mode {mode} is invalid for timing profile "
            f"{profile_name}; allowed mode(s): "
            f"{', '.join(str(value) for value in allowed_modes)}."
        )
    return mode


def validate_camera_timing(context, profiles):
    profile_name = LaunchConfiguration("timing_profile").perform(context)
    requested_mode = LaunchConfiguration("inter_cam_sync_mode").perform(context)
    requested_global_time = (
        LaunchConfiguration("global_time_enabled").perform(context).lower()
        == "true"
    )
    mode = resolve_inter_cam_sync_mode(
        profile_name, requested_mode, profiles
    )
    expected_global_time = bool(
        profiles[profile_name]["realsense"]["global_time_enabled"]
    )
    if requested_global_time is not expected_global_time:
        raise RuntimeError(
            f"global_time_enabled={str(requested_global_time).lower()} is "
            f"invalid for timing profile {profile_name}; expected "
            f"{str(expected_global_time).lower()}."
        )
    camera_fps = float(LaunchConfiguration("camera_fps").perform(context))
    if mode in (1, 2, 3) and abs(camera_fps - 30.0) > 1e-6:
        raise RuntimeError(
            "The dual-camera profiles require matched 30 Hz streams; "
            f"got {camera_fps:g} Hz in sync mode {mode}."
        )
    context.launch_configurations["resolved_inter_cam_sync_mode"] = str(mode)
    return []


def generate_launch_description():
    with SYNC_CONFIG.open() as stream:
        sync_config = yaml.safe_load(stream)
    active_profile = sync_config["active_profile"]
    profile_config = sync_config["profiles"][active_profile]
    camera_sync = profile_config["realsense"]

    camera_fps = LaunchConfiguration("camera_fps")
    profile = ParameterValue(["848x480x", camera_fps], value_type=str)
    serial_no = LaunchConfiguration("serial_no")
    resolved_inter_cam_sync_mode = LaunchConfiguration(
        "resolved_inter_cam_sync_mode"
    )
    global_time_enabled = LaunchConfiguration("global_time_enabled")
    depth_auto_exposure = LaunchConfiguration("depth_auto_exposure")
    color_auto_exposure = LaunchConfiguration("color_auto_exposure")
    depth_exposure = LaunchConfiguration("depth_exposure")
    color_exposure = LaunchConfiguration("color_exposure")

    return LaunchDescription([
        DeclareLaunchArgument(
            "timing_profile",
            default_value=active_profile,
            choices=sorted(sync_config["profiles"]),
            description="Timing contract used to validate the requested camera mode.",
        ),
        DeclareLaunchArgument(
            "serial_no",
            default_value="",
            description=(
                "Optional D455f-A serial number. Empty preserves automatic "
                "single-camera selection."
            ),
        ),
        DeclareLaunchArgument(
            "camera_fps",
            default_value=str(profile_config["camera_fps"]),
            choices=["30", "60"],
            description="Matched RGB and depth FPS.",
        ),
        DeclareLaunchArgument(
            "inter_cam_sync_mode",
            default_value="auto",
            choices=["auto", "0", "1", "2", "3"],
            description=(
                "D455f sync mode: auto=profile default, 0=free running, 1=master, "
                "2=external-trigger slave, 3=full slave. "
                "Laptop A defaults to master."
            ),
        ),
        DeclareLaunchArgument(
            "global_time_enabled",
            default_value=str(camera_sync["global_time_enabled"]).lower(),
            choices=["true", "false"],
            description="Map RealSense hardware timestamps into the host clock epoch.",
        ),
        DeclareLaunchArgument(
            "depth_auto_exposure",
            default_value=str(not camera_sync["manual_exposure"]).lower(),
            choices=["true", "false"],
        ),
        DeclareLaunchArgument(
            "color_auto_exposure",
            default_value=str(not camera_sync["manual_exposure"]).lower(),
            choices=["true", "false"],
        ),
        DeclareLaunchArgument(
            "depth_exposure",
            default_value=str(camera_sync["depth_exposure_us"]),
            description="D455f depth exposure in microseconds.",
        ),
        DeclareLaunchArgument(
            "color_exposure",
            default_value=str(camera_sync["color_exposure"]),
            description="D455f RGB exposure in device units.",
        ),
        OpaqueFunction(
            function=validate_camera_timing,
            args=[sync_config["profiles"]],
        ),
        Node(
            package="realsense2_camera",
            executable="realsense2_camera_node",
            namespace="camera",
            name="camera",
            output="screen",
            parameters=[{
                "camera_name": "camera",
                "camera_namespace": "camera",
                "device_type": "d455f",
                "serial_no": ParameterValue(serial_no, value_type=str),
                "enable_color": True,
                "enable_depth": True,
                "rgb_camera.color_profile": profile,
                "depth_module.depth_profile": profile,
                "rgb_camera.color_format": "RGB8",
                "depth_module.depth_format": "Z16",
                "enable_sync": True,
                "align_depth.enable": True,
                "depth_module.inter_cam_sync_mode": ParameterValue(
                    resolved_inter_cam_sync_mode, value_type=int
                ),
                "depth_module.global_time_enabled": ParameterValue(
                    global_time_enabled, value_type=bool
                ),
                "rgb_camera.global_time_enabled": ParameterValue(
                    global_time_enabled, value_type=bool
                ),
                "depth_module.enable_auto_exposure": ParameterValue(
                    depth_auto_exposure, value_type=bool
                ),
                "rgb_camera.enable_auto_exposure": ParameterValue(
                    color_auto_exposure, value_type=bool
                ),
                "depth_module.exposure": ParameterValue(
                    depth_exposure, value_type=int
                ),
                "rgb_camera.exposure": ParameterValue(
                    color_exposure, value_type=int
                ),
                "pointcloud.enable": False,
                "enable_gyro": False,
                "enable_accel": False,
                "enable_infra": False,
                "enable_infra1": False,
                "enable_infra2": False,
                "publish_tf": True,
                "tf_publish_rate": 0.0,
                # The driver prefixes this value with camera_name:
                # "link" -> camera_link. Using "camera_link" here creates
                # the incorrect camera_camera_link root.
                "base_frame_id": "link",
                "tf_prefix": "",
                "spatial_filter.enable": True,
                "temporal_filter.enable": False,
                "hole_filling_filter.enable": False,
            }],
        ),
    ])
