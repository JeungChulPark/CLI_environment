#!/usr/bin/env python3

import time

import rclpy
from launch import LaunchDescription
from launch.actions import LogInfo, OpaqueFunction
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rtabmap_msgs.msg import RGBDImage


RGBD_TOPIC = "/rtabmap/rgbd_image"
CHECK_DURATION_SECONDS = 5.0
MINIMUM_RATE_HZ = 2.0


def check_rgbd_rate(_context):
    ros_context = Context()
    rclpy.init(args=[], context=ros_context)
    node = Node("auto_mapping_rgbd_rate_check", context=ros_context)
    executor = SingleThreadedExecutor(context=ros_context)
    executor.add_node(node)

    count = 0

    def callback(_message):
        nonlocal count
        count += 1

    qos = QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )
    node.create_subscription(RGBDImage, RGBD_TOPIC, callback, qos)

    start = time.monotonic()
    deadline = start + CHECK_DURATION_SECONDS
    try:
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.1)
    finally:
        elapsed = time.monotonic() - start
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown(context=ros_context)

    rate = count / elapsed
    result = f"{rate:.2f} Hz ({count} frames/{elapsed:.1f}s)"
    if rate < MINIMUM_RATE_HZ:
        raise RuntimeError(
            "Synchronized RGB-D rate is too low: "
            f"{result}. Do not drive: RTAB-Map would create multi-meter gaps."
        )

    return [LogInfo(msg=f"Synchronized RGB-D is ready: {result}")]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=check_rgbd_rate),
    ])
