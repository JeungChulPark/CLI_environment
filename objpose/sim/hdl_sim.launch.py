# hdl_graph_slam on the simulated VLP-16. Reuses ~/objpose/nav/hdl_nav.launch.py, but all hdl TF goes to /hdl/tf so
# its scan-matching odom->base_link does not fight the robot's EKF odom->base_link in /a200_0881/tf;
# hdl_map_relay.py turns the hdl pose into map->odom for the robot tree.
import os
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetRemap

HDL_LAUNCH = os.path.expanduser('~/objpose/nav/hdl_nav.launch.py')


def generate_launch_description():
    return LaunchDescription([GroupAction([
        SetRemap('/tf', '/hdl/tf'),
        SetRemap('/tf_static', '/hdl/tf_static'),
        # sensor arch mount: base_link -> lidar3d_0_laser (from the robot URDF)
        Node(package='tf2_ros', executable='static_transform_publisher', name='hdl_lidar_tf',
             arguments=['--z', '0.562', '--frame-id', 'base_link', '--child-frame-id', 'lidar3d_0_laser']),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(HDL_LAUNCH), launch_arguments=[
            ('use_sim_time', 'true'),
            ('raw_points_topic', '/a200_0881/sensors/lidar3d_0/points'),
            # the included launch adds an identity base_link->lidar_frame_id; point it at an unused frame
            ('lidar_frame_id', 'hdl_unused_lidar'),
            ('graph_keyframe_delta_trans', '1.0'),
            ('graph_keyframe_delta_angle', '0.5'),
            # floor plane constraints keep the map level (without them the floor spread over -1.0..+0.2 m in z)
            ('enable_floor_detection', 'true'),
        ]),
    ])])
