// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from hdl_graph_slam:msg/FloorCoeffs.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "hdl_graph_slam/msg/floor_coeffs.h"


#ifndef HDL_GRAPH_SLAM__MSG__DETAIL__FLOOR_COEFFS__STRUCT_H_
#define HDL_GRAPH_SLAM__MSG__DETAIL__FLOOR_COEFFS__STRUCT_H_

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
// Member 'coeffs'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/FloorCoeffs in the package hdl_graph_slam.
typedef struct hdl_graph_slam__msg__FloorCoeffs
{
  std_msgs__msg__Header header;
  rosidl_runtime_c__float__Sequence coeffs;
} hdl_graph_slam__msg__FloorCoeffs;

// Struct for a sequence of hdl_graph_slam__msg__FloorCoeffs.
typedef struct hdl_graph_slam__msg__FloorCoeffs__Sequence
{
  hdl_graph_slam__msg__FloorCoeffs * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hdl_graph_slam__msg__FloorCoeffs__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HDL_GRAPH_SLAM__MSG__DETAIL__FLOOR_COEFFS__STRUCT_H_
