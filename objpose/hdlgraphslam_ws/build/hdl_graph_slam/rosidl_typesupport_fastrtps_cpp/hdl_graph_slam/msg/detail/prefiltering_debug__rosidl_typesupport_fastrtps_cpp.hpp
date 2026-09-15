// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from hdl_graph_slam:msg/PrefilteringDebug.idl
// generated code does not contain a copyright notice

#ifndef HDL_GRAPH_SLAM__MSG__DETAIL__PREFILTERING_DEBUG__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define HDL_GRAPH_SLAM__MSG__DETAIL__PREFILTERING_DEBUG__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include <cstddef>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "hdl_graph_slam/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "hdl_graph_slam/msg/detail/prefiltering_debug__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace hdl_graph_slam
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_hdl_graph_slam
cdr_serialize(
  const hdl_graph_slam::msg::PrefilteringDebug & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_hdl_graph_slam
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  hdl_graph_slam::msg::PrefilteringDebug & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_hdl_graph_slam
get_serialized_size(
  const hdl_graph_slam::msg::PrefilteringDebug & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_hdl_graph_slam
max_serialized_size_PrefilteringDebug(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_hdl_graph_slam
cdr_serialize_key(
  const hdl_graph_slam::msg::PrefilteringDebug & ros_message,
  eprosima::fastcdr::Cdr &);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_hdl_graph_slam
get_serialized_size_key(
  const hdl_graph_slam::msg::PrefilteringDebug & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_hdl_graph_slam
max_serialized_size_key_PrefilteringDebug(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace hdl_graph_slam

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_hdl_graph_slam
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, hdl_graph_slam, msg, PrefilteringDebug)();

#ifdef __cplusplus
}
#endif

#endif  // HDL_GRAPH_SLAM__MSG__DETAIL__PREFILTERING_DEBUG__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
