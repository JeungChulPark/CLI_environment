# Real Husky A200: live VLP-16 (/velodyne_points from vlp16_publisher.py) -> hdl_graph_slam, wall clock.
# Same layout as the simulation (objpose/sim/hdl_sim.launch.py): all hdl TF in /hdl/tf so its scan-matching
# odom->base_link does not fight the Clearpath EKF; hdl_map_relay.py composes map->odom for /a200_0881/tf.
# Lidar mounting comes from env LIDAR_X/Y/Z/YAW (base_link -> velodyne; NOT measured yet on the robot).
import os
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetRemap

HDL_LAUNCH = os.path.expanduser('~/objpose/nav/hdl_nav.launch.py')
X = [os.environ.get(k, d) for k, d in (('LIDAR_X', '0'), ('LIDAR_Y', '0'), ('LIDAR_Z', '0.5'), ('LIDAR_YAW', '0'))]


def generate_launch_description():
    return LaunchDescription([GroupAction([
        SetRemap('/tf', '/hdl/tf'),
        SetRemap('/tf_static', '/hdl/tf_static'),
        Node(package='tf2_ros', executable='static_transform_publisher', name='hdl_lidar_tf',
             arguments=['--x', X[0], '--y', X[1], '--z', X[2], '--yaw', X[3],
                        '--frame-id', 'base_link', '--child-frame-id', 'velodyne']),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(HDL_LAUNCH), launch_arguments=[
            ('use_sim_time', 'false'),
            ('raw_points_topic', '/velodyne_points'),
            ('lidar_frame_id', 'hdl_unused_lidar'),
            ('graph_keyframe_delta_trans', '1.0'),
            ('graph_keyframe_delta_angle', '0.5'),
            ('enable_floor_detection', 'true'),
            # ignore points closer than 1 m: things right beside the robot (and people) made scan matching flip
            ('distance_near_thresh', os.environ.get('HDL_NEAR', '1.0')),
        ]),
    ])])
