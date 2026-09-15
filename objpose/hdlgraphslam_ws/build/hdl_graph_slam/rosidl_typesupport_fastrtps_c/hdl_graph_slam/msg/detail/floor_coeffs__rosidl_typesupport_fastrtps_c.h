// generated from rosidl_typesupport_fastrtps_c/resource/idl__rosidl_typesupport_fastrtps_c.h.em
// with input from hdl_graph_slam:msg/FloorCoeffs.idl
// generated code does not contain a copyright notice
#ifndef HDL_GRAPH_SLAM__MSG__DETAIL__FLOOR_COEFFS__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
#define HDL_GRAPH_SLAM__MSG__DETAIL__FLOOR_COEFFS__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_


#include <stddef.h>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "hdl_graph_slam/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "hdl_graph_slam/msg/detail/floor_coeffs__struct.h"
#include "fastcdr/Cdr.h"

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_hdl_graph_slam
bool cdr_serialize_hdl_graph_slam__msg__FloorCoeffs(
  const hdl_graph_slam__msg__FloorCoeffs * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_hdl_graph_slam
bool cdr_deserialize_hdl_graph_slam__msg__FloorCoeffs(
  eprosima::fastcdr::Cdr &,
  hdl_graph_slam__msg__FloorCoeffs * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_hdl_graph_slam
size_t get_serialized_size_hdl_graph_slam__msg__FloorCoeffs(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_hdl_graph_slam
size_t max_serialized_size_hdl_graph_slam__msg__FloorCoeffs(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_hdl_graph_slam
bool cdr_serialize_key_hdl_graph_slam__msg__FloorCoeffs(
  const hdl_graph_slam__msg__FloorCoeffs * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_hdl_graph_slam
size_t get_serialized_size_key_hdl_graph_slam__msg__FloorCoeffs(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_hdl_graph_slam
size_t max_serialized_size_key_hdl_graph_slam__msg__FloorCoeffs(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_hdl_graph_slam
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, hdl_graph_slam, msg, FloorCoeffs)();

#ifdef __cplusplus
}
#endif

#endif  // HDL_GRAPH_SLAM__MSG__DETAIL__FLOOR_COEFFS__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
