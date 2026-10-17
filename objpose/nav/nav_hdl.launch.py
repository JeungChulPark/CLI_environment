"""Nav2 on an hdl_graph_slam map: map_server (+lifecycle) and nav2_bringup navigation_launch (no AMCL;
hdl_graph_slam provides map->odom->base_link).
  ros2 launch nav_hdl.launch.py map:=<grid.yaml> params:=<nav2 params> use_sim_time:=true"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

BRINGUP = "/opt/ros/jazzy/share/nav2_bringup/launch"


def generate_launch_description():
    sim = LaunchConfiguration("use_sim_time"); params = LaunchConfiguration("params")
    return LaunchDescription([
        DeclareLaunchArgument("map"), DeclareLaunchArgument("params"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        Node(package="nav2_map_server", executable="map_server", name="map_server", output="screen",
             parameters=[params, {"yaml_filename": LaunchConfiguration("map"), "use_sim_time": sim}]),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager", name="lifecycle_manager_map",
             output="screen", parameters=[{"use_sim_time": sim, "autostart": True, "node_names": ["map_server"]}]),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(BRINGUP, "navigation_launch.py")),
                                 launch_arguments={"params_file": params, "use_sim_time": sim,
                                                   "autostart": "true", "use_composition": "False"}.items()),
    ])
