# Standalone global planner on a saved map (no robot): map_server + planner_server (with its global costmap) +
# lifecycle manager, identity map->odom->base_link TF so the costmap can start. Run in a spare ROS domain.
#   ros2 launch planner_eval.launch.py params:=<nav2 params yaml> map:=<grid.yaml>
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# no namespace: the params file is keyed by bare node names (navigation_launch.py rewrites them for a namespace)
NS = ''
REMAP = []


def generate_launch_description():
    params, grid = LaunchConfiguration('params'), LaunchConfiguration('map')
    tf = lambda a, b: Node(package='tf2_ros', executable='static_transform_publisher', namespace=NS, remappings=REMAP,
                           arguments=['--frame-id', a, '--child-frame-id', b], name=f'tf_{a}_{b}')
    return LaunchDescription([
        DeclareLaunchArgument('params'), DeclareLaunchArgument('map'),
        tf('map', 'odom'), tf('odom', 'base_link'),
        Node(package='nav2_map_server', executable='map_server', name='map_server', namespace=NS, remappings=REMAP,
             parameters=[{'yaml_filename': grid, 'use_sim_time': False}]),
        Node(package='nav2_planner', executable='planner_server', name='planner_server', namespace=NS, remappings=REMAP,
             parameters=[params]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_eval', namespace=NS,
             parameters=[{'autostart': True, 'node_names': ['map_server', 'planner_server'], 'use_sim_time': False}]),
    ])
