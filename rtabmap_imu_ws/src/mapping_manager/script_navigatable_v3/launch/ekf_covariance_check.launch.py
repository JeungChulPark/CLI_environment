#!/usr/bin/env python3

import math
import time

import rclpy
from launch import LaunchDescription
from launch.actions import LogInfo, OpaqueFunction
from nav_msgs.msg import Odometry
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node


ODOMETRY_TOPIC = "/odometry/filtered"
MAX_POSE_VARIANCE = 0.5
REQUIRED_BOUNDED_SAMPLES = 5
CHECK_TIMEOUT_SECONDS = 8.0


def format_covariance(values):
    x, y, yaw = values
    return f"x={x:.6g} y={y:.6g} yaw={yaw:.6g}"


def check_ekf_covariance(_context):
    ros_context = Context()
    rclpy.init(args=[], context=ros_context)
    node = Node("auto_mapping_covariance_check", context=ros_context)
    executor = SingleThreadedExecutor(context=ros_context)
    executor.add_node(node)

    sample = None
    sample_sequence = 0

    def callback(message):
        nonlocal sample, sample_sequence
        covariance = message.pose.covariance
        sample = (
            float(covariance[0]),
            float(covariance[7]),
            float(covariance[35]),
        )
        sample_sequence += 1

    def is_bounded(values):
        return (
            all(math.isfinite(value) and value >= 0.0 for value in values)
            and all(value <= MAX_POSE_VARIANCE for value in values)
        )

    node.create_subscription(Odometry, ODOMETRY_TOPIC, callback, 10)
    deadline = time.monotonic() + CHECK_TIMEOUT_SECONDS
    last_sequence = -1
    bounded_count = 0
    bounded_sample = None

    try:
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.2)
            if sample_sequence == last_sequence or sample is None:
                continue

            last_sequence = sample_sequence
            if is_bounded(sample):
                bounded_count += 1
                if bounded_count >= REQUIRED_BOUNDED_SAMPLES:
                    bounded_sample = sample
                    break
            else:
                bounded_count = 0
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown(context=ros_context)

    if sample is None:
        raise RuntimeError(f"No message received from {ODOMETRY_TOPIC}.")

    covariance = format_covariance(bounded_sample or sample)
    if bounded_sample is None:
        raise RuntimeError(
            "EKF pose covariance did not converge: "
            f"{covariance}. RTAB-Map graph optimization would reject valid "
            "loop closures."
        )

    return [LogInfo(msg=f"EKF covariance is bounded: {covariance}")]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=check_ekf_covariance),
    ])
