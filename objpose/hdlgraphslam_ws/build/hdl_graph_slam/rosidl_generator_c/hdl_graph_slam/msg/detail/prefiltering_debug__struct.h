// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from hdl_graph_slam:msg/PrefilteringDebug.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "hdl_graph_slam/msg/prefiltering_debug.h"


#ifndef HDL_GRAPH_SLAM__MSG__DETAIL__PREFILTERING_DEBUG__STRUCT_H_
#define HDL_GRAPH_SLAM__MSG__DETAIL__PREFILTERING_DEBUG__STRUCT_H_

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

/// Struct defined in msg/PrefilteringDebug in the package hdl_graph_slam.
typedef struct hdl_graph_slam__msg__PrefilteringDebug
{
  std_msgs__msg__Header header;
  uint32_t input_point_count;
  uint32_t transformed_point_count;
  uint32_t distance_filtered_point_count;
  uint32_t downsampled_point_count;
  uint32_t output_point_count;
  float callback_time_ms;
  uint64_t output_count;
} hdl_graph_slam__msg__PrefilteringDebug;

// Struct for a sequence of hdl_graph_slam__msg__PrefilteringDebug.
typedef struct hdl_graph_slam__msg__PrefilteringDebug__Sequence
{
  hdl_graph_slam__msg__PrefilteringDebug * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hdl_graph_slam__msg__PrefilteringDebug__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HDL_GRAPH_SLAM__MSG__DETAIL__PREFILTERING_DEBUG__STRUCT_H_
