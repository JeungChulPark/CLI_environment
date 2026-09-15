// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from hdl_graph_slam:msg/ScanMatchingOdometryDebug.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "hdl_graph_slam/msg/scan_matching_odometry_debug.h"


#ifndef HDL_GRAPH_SLAM__MSG__DETAIL__SCAN_MATCHING_ODOMETRY_DEBUG__STRUCT_H_
#define HDL_GRAPH_SLAM__MSG__DETAIL__SCAN_MATCHING_ODOMETRY_DEBUG__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"

/// Struct defined in msg/ScanMatchingOdometryDebug in the package hdl_graph_slam.
typedef struct hdl_graph_slam__msg__ScanMatchingOdometryDebug
{
  std_msgs__msg__Header header;
  uint32_t input_point_count;
  uint32_t downsampled_point_count;
  float registration_time_ms;
  float callback_time_ms;
  uint64_t odom_count;
  bool registration_triggered;
  bool has_converged;
  float matching_error;
} hdl_graph_slam__msg__ScanMatchingOdometryDebug;

// Struct for a sequence of hdl_graph_slam__msg__ScanMatchingOdometryDebug.
typedef struct hdl_graph_slam__msg__ScanMatchingOdometryDebug__Sequence
{
  hdl_graph_slam__msg__ScanMatchingOdometryDebug * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hdl_graph_slam__msg__ScanMatchingOdometryDebug__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HDL_GRAPH_SLAM__MSG__DETAIL__SCAN_MATCHING_ODOMETRY_DEBUG__STRUCT_H_
