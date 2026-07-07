# bringup_A.launch.py — 현재 사이트 단일머신 통합 기동 (ROS2 Jazzy)
#   RealSense D435i(=D455f 소프트웨어 대역) + Velodyne VLP-16
#   실행: ros2 launch launch/bringup_A.launch.py
#
# 주의: 이 폴더는 colcon 패키지가 아니므로 launch 파일 경로를 직접 지정해 실행한다.
#       realsense2_camera / velodyne 는 apt(ros-jazzy-*)로 설치되어 있어야 한다.

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    here = os.path.dirname(os.path.realpath(__file__))
    rs_params = os.path.join(here, '..', 'config', 'realsense_A.yaml')

    # --- RealSense (namespace=cam_A, node=camera → 토픽 /cam_A/...) ---
    realsense = Node(
        package='realsense2_camera',
        namespace='cam_A',
        name='camera',
        executable='realsense2_camera_node',
        parameters=[rs_params],
        output='screen',
    )

    # --- Velodyne VLP-16 (드라이버 + pointcloud + transform) ---
    #   gps_time 기본 false → 호스트 UDP 도착시각. 패키지 제공 통합 launch 재사용.
    velodyne_share = get_package_share_directory('velodyne')
    velodyne = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(velodyne_share, 'launch',
                         'velodyne-all-nodes-VLP16-launch.py')
        )
    )

    return LaunchDescription([realsense, velodyne])
