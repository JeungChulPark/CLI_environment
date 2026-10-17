# Gazebo server only (no GUI window) + simulated Clearpath robot. Same args as clearpath_gz simulation.launch.py.
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_gz = get_package_share_directory('clearpath_gz')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    share_paths = [os.path.join(p, 'share') for p in os.environ['AMENT_PREFIX_PATH'].split(':')]
    args = [
        DeclareLaunchArgument('world', default_value='warehouse'),
        DeclareLaunchArgument('setup_path', default_value=os.path.expanduser('~/clearpath_sim/')),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.3'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('generate', default_value='true'),
    ]
    resource_path = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        ':'.join([os.path.join(pkg_gz, 'worlds'), os.path.join(pkg_gz, 'meshes')] + share_paths))
    gz_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments=[('gz_args', [LaunchConfiguration('world'), '.sdf -r -s -v 2 --headless-rendering'])])
    clock_bridge = Node(package='ros_gz_bridge', executable='parameter_bridge', name='clock_bridge',
                        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'])
    robot_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gz, 'launch', 'robot_spawn.launch.py')),
        launch_arguments=[('use_sim_time', 'true'), ('rviz', 'false'), ('generate', LaunchConfiguration('generate'))] +
                         [(k, LaunchConfiguration(k)) for k in ('setup_path', 'world', 'x', 'y', 'z', 'yaw')])
    return LaunchDescription(args + [resource_path, gz_server, clock_bridge, robot_spawn])
