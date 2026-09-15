// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from hdl_graph_slam:msg/ScanMatchingStatus.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "hdl_graph_slam/msg/detail/scan_matching_status__functions.h"
#include "hdl_graph_slam/msg/detail/scan_matching_status__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace hdl_graph_slam
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void ScanMatchingStatus_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) hdl_graph_slam::msg::ScanMatchingStatus(_init);
}

void ScanMatchingStatus_fini_function(void * message_memory)
{
  auto typed_message = static_cast<hdl_graph_slam::msg::ScanMatchingStatus *>(message_memory);
  typed_message->~ScanMatchingStatus();
}

size_t size_function__ScanMatchingStatus__prediction_labels(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<std_msgs::msg::String> *>(untyped_member);
  return member->size();
}

const void * get_const_function__ScanMatchingStatus__prediction_labels(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<std_msgs::msg::String> *>(untyped_member);
  return &member[index];
}

void * get_function__ScanMatchingStatus__prediction_labels(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<std_msgs::msg::String> *>(untyped_member);
  return &member[index];
}

void fetch_function__ScanMatchingStatus__prediction_labels(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const std_msgs::msg::String *>(
    get_const_function__ScanMatchingStatus__prediction_labels(untyped_member, index));
  auto & value = *reinterpret_cast<std_msgs::msg::String *>(untyped_value);
  value = item;
}

void assign_function__ScanMatchingStatus__prediction_labels(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<std_msgs::msg::String *>(
    get_function__ScanMatchingStatus__prediction_labels(untyped_member, index));
  const auto & value = *reinterpret_cast<const std_msgs::msg::String *>(untyped_value);
  item = value;
}

void resize_function__ScanMatchingStatus__prediction_labels(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<std_msgs::msg::String> *>(untyped_member);
  member->resize(size);
}

size_t size_function__ScanMatchingStatus__prediction_errors(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<geometry_msgs::msg::Pose> *>(untyped_member);
  return member->size();
}

const void * get_const_function__ScanMatchingStatus__prediction_errors(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<geometry_msgs::msg::Pose> *>(untyped_member);
  return &member[index];
}

void * get_function__ScanMatchingStatus__prediction_errors(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<geometry_msgs::msg::Pose> *>(untyped_member);
  return &member[index];
}

void fetch_function__ScanMatchingStatus__prediction_errors(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const geometry_msgs::msg::Pose *>(
    get_const_function__ScanMatchingStatus__prediction_errors(untyped_member, index));
  auto & value = *reinterpret_cast<geometry_msgs::msg::Pose *>(untyped_value);
  value = item;
}

void assign_function__ScanMatchingStatus__prediction_errors(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<geometry_msgs::msg::Pose *>(
    get_function__ScanMatchingStatus__prediction_errors(untyped_member, index));
  const auto & value = *reinterpret_cast<const geometry_msgs::msg::Pose *>(untyped_value);
  item = value;
}

void resize_function__ScanMatchingStatus__prediction_errors(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<geometry_msgs::msg::Pose> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember ScanMatchingStatus_message_member_array[7] = {
  {
    "header",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Header>(),  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hdl_graph_slam::msg::ScanMatchingStatus, header),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "has_converged",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hdl_graph_slam::msg::ScanMatchingStatus, has_converged),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "matching_error",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hdl_graph_slam::msg::ScanMatchingStatus, matching_error),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "inlier_fraction",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hdl_graph_slam::msg::ScanMatchingStatus, inlier_fraction),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "relative_pose",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<geometry_msgs::msg::Pose>(),  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hdl_graph_slam::msg::ScanMatchingStatus, relative_pose),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "prediction_labels",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::String>(),  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hdl_graph_slam::msg::ScanMatchingStatus, prediction_labels),  // bytes offset in struct
    nullptr,  // default value
    size_function__ScanMatchingStatus__prediction_labels,  // size() function pointer
    get_const_function__ScanMatchingStatus__prediction_labels,  // get_const(index) function pointer
    get_function__ScanMatchingStatus__prediction_labels,  // get(index) function pointer
    fetch_function__ScanMatchingStatus__prediction_labels,  // fetch(index, &value) function pointer
    assign_function__ScanMatchingStatus__prediction_labels,  // assign(index, value) function pointer
    resize_function__ScanMatchingStatus__prediction_labels  // resize(index) function pointer
  },
  {
    "prediction_errors",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<geometry_msgs::msg::Pose>(),  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hdl_graph_slam::msg::ScanMatchingStatus, prediction_errors),  // bytes offset in struct
    nullptr,  // default value
    size_function__ScanMatchingStatus__prediction_errors,  // size() function pointer
    get_const_function__ScanMatchingStatus__prediction_errors,  // get_const(index) function pointer
    get_function__ScanMatchingStatus__prediction_errors,  // get(index) function pointer
    fetch_function__ScanMatchingStatus__prediction_errors,  // fetch(index, &value) function pointer
    assign_function__ScanMatchingStatus__prediction_errors,  // assign(index, value) function pointer
    resize_function__ScanMatchingStatus__prediction_errors  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers ScanMatchingStatus_message_members = {
  "hdl_graph_slam::msg",  // message namespace
  "ScanMatchingStatus",  // message name
  7,  // number of fields
  sizeof(hdl_graph_slam::msg::ScanMatchingStatus),
  false,  // has_any_key_member_
  ScanMatchingStatus_message_member_array,  // message members
  ScanMatchingStatus_init_function,  // function to initialize message memory (memory has to be allocated)
  ScanMatchingStatus_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t ScanMatchingStatus_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &ScanMatchingStatus_message_members,
  get_message_typesupport_handle_function,
  &hdl_graph_slam__msg__ScanMatchingStatus__get_type_hash,
  &hdl_graph_slam__msg__ScanMatchingStatus__get_type_description,
  &hdl_graph_slam__msg__ScanMatchingStatus__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace hdl_graph_slam


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<hdl_graph_slam::msg::ScanMatchingStatus>()
{
  return &::hdl_graph_slam::msg::rosidl_typesupport_introspection_cpp::ScanMatchingStatus_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, hdl_graph_slam, msg, ScanMatchingStatus)() {
  return &::hdl_graph_slam::msg::rosidl_typesupport_introspection_cpp::ScanMatchingStatus_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
