# Fixed 2D map for Nav2 on the real robot: nav2 map_server (+ lifecycle manager) in /a200_0881, publishing
# /a200_0881/map from a pcd2grid.py ray-cast grid (replaces the live hdl_grid.py).
#   ros2 launch real_map.launch.py map:=<grid.yaml>
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map'),
        Node(package='nav2_map_server', executable='map_server', name='map_server', namespace='a200_0881',
             parameters=[{'yaml_filename': LaunchConfiguration('map'), 'use_sim_time': False, 'topic_name': 'map'}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_map',
             namespace='a200_0881', parameters=[{'autostart': True, 'node_names': ['map_server'], 'use_sim_time': False}]),
    ])
