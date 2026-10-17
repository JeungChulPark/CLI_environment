# Nav2 for the real A200 (wall clock) with nav2_real.yaml, plus pointcloud_to_laserscan: VLP-16 points between
# 0.15 and 1.5 m above base_link -> /a200_0881/scan (frame base_link) for the costmaps and the collision monitor.
import os
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace, SetRemap

NS = 'a200_0881'
PARAMS = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'nav2_real.yaml')


def generate_launch_description():
    p2l = Node(package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node', name='vlp_to_scan',
               remappings=[('cloud_in', '/velodyne_points'), ('scan', f'/{NS}/scan')],
               parameters=[{'target_frame': 'base_link', 'min_height': 0.15, 'max_height': 1.5,
                            'angle_increment': 0.0035, 'range_min': 0.5, 'range_max': 30.0,
                            'use_inf': True, 'transform_tolerance': 0.1}])
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource('/opt/ros/jazzy/share/nav2_bringup/launch/navigation_launch.py'),
        launch_arguments=[('use_sim_time', 'false'), ('params_file', PARAMS),
                          ('use_composition', 'False'), ('namespace', NS)])
    return LaunchDescription([GroupAction([
        PushRosNamespace(NS),
        SetRemap(f'/{NS}/odom', f'/{NS}/platform/odom/filtered'),
        SetRemap('/tf', f'/{NS}/tf'),
        SetRemap('/tf_static', f'/{NS}/tf_static'),
        p2l, nav2])])
