# Nav2 for the simulated A200 with nav2_sim.yaml (same namespace/remaps as clearpath_nav2_demos nav2.launch.py).
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import PushRosNamespace, SetRemap

NS = 'a200_0881'
PARAMS = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'nav2_sim.yaml')


def generate_launch_description():
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')),
        launch_arguments=[('use_sim_time', 'true'), ('params_file', PARAMS),
                          ('use_composition', 'False'), ('namespace', NS)])
    return LaunchDescription([GroupAction([
        PushRosNamespace(NS),
        SetRemap(f'/{NS}/odom', f'/{NS}/platform/odom'),
        SetRemap('/tf', f'/{NS}/tf'),
        SetRemap('/tf_static', f'/{NS}/tf_static'),
        nav2])])
