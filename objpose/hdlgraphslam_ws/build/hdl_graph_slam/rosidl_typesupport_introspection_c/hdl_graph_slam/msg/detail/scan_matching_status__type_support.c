// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from hdl_graph_slam:msg/ScanMatchingStatus.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "hdl_graph_slam/msg/detail/scan_matching_status__rosidl_typesupport_introspection_c.h"
#include "hdl_graph_slam/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "hdl_graph_slam/msg/detail/scan_matching_status__functions.h"
#include "hdl_graph_slam/msg/detail/scan_matching_status__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `relative_pose`
// Member `prediction_errors`
#include "geometry_msgs/msg/pose.h"
// Member `relative_pose`
// Member `prediction_errors`
#include "geometry_msgs/msg/detail/pose__rosidl_typesupport_introspection_c.h"
// Member `prediction_labels`
#include "std_msgs/msg/string.h"
// Member `prediction_labels`
#include "std_msgs/msg/detail/string__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void hdl_graph_slam__msg__ScanMatchingStatus__rosidl_typesupport_introspection_c__ScanMatchingStatus_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  hdl_graph_slam__msg__ScanMatchingStatus__init(message_memory);
}

void hdl_graph_slam__msg__ScanMatchingStatus__rosidl_typesupport_introspection_c__ScanMatchingStatus_fini_function(void * message_memory)
{
  hdl_graph_slam__msg__ScanMatchingStatus__fini(message_memory);
}

size_t hdl_graph_slam__msg__ScanMatchingStatus__rosidl_typesupport_introspection_c__size_function__ScanMatchingStatus__prediction_labels(
  const void * untyped_member)
{
  const std_msgs__msg__String__Sequence * member =
    (const std_msgs__msg__String__Sequence *)(untyped_member);
  return member->size;
}

const void * hdl_graph_slam__msg__ScanMatchingStatus__rosidl_typesupport_introspection_c__get_const_function__ScanMatchingStatus__prediction_labels(
  const void * untyped_member, size_t index)
{
  const std_msgs__msg__String__Sequence * member =
    (const std_msgs__msg__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void * hdl_graph_slam__msg__ScanMatchingStatus__rosidl_typesupport_introspection_c__get_function__ScanMatchingStatus__prediction_labels(
  void * untyped_member, size_t index)
{
  std_msgs__msg__String__Sequence * member =
    (std_msgs__msg__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void hdl_graph_slam__msg__ScanMatchingStatus__rosidl_typesupport_introspection_c__fetch_function__ScanMatchingStatus__prediction_labels(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const std_msgs__msg__String * item =
    ((const std_msgs__msg__String *)
    hdl_graph_slam__msg__ScanMatchingStatus__rosidl_typesupport_introspection_c__get_const_function__ScanMatchingStatus__prediction_labels(untyped_member, index));
  std_msgs__msg__String * value =
    (std_msgs__msg__String *)(untyped_value);
  *value = *item;
}

void hdl_graph_slam__msg__ScanMatchingStatus__rosidl_typesupport_introspection_c__assign_function__ScanMatchingStatus__prediction_labels(
  void * untyped_member, size_t index, const void * untyped_value)
{
  std_msgs__msg__String * item =
    ((std_msgs__msg__String *)
    hdl_graph_slam__msg__ScanMatchingStatus__rosidl_typesupport_introspection_c__get_function__ScanMatchingStatus__prediction_labels(untyped_member, index));
  const std_msgs__msg__String * value =
    (const std_msgs__msg__String *)(untyped_value);
  *item = *value;
}

bool hdl_graph_slam__msg__ScanMatchingStatus__rosidl_typesupport_introspection_c__resize_function__ScanMatchingStatus__prediction_labels(
  void * untyped_member, size_t size)
{
  std_msgs__msg__String__Sequence * member =
    (std_msgs__msg__String__Sequence *)(untyped_member);
  std_msgs__msg__String__Sequence__fini(member);
  return std_msgs__msg__String__Sequence__init(member, size);
}

size_t hdl_graph_slam__msg__ScanMatchingStatus__rosidl_typesupport_introspection_c__size_function__ScanMatchingStatus__prediction_errors(
  const void * untyped_member)
{
  const geometry_msgs__msg__Pose__Sequence * member =
    (const geometry_msgs__msg__Pose__Sequence *)(untyped_member);
  return member->size;
}

const void * hdl_graph_slam__msg__ScanMatchingStatus__rosidl_typesupport_introspection_c__get_const_function__ScanMatchingStatus__prediction_errors(
  const void * untyped_member, size_t index)
{
  const geometry_msgs__msg__Pose__Sequence * member =
    (const geometry_msgs__msg__Pose__Sequence *)(untyped_member);
  return &member->data[index];
}

void * hdl_graph_slam__msg__ScanMatchingStatus__rosidl_typesupport_introspection_c__get_function__ScanMatchingStatus__prediction_errors(
  void * untyped_member, size_t index)
{
  geometry_msgs__msg__Pose__Sequence * member =
    (geometry_msgs__msg__Pose__Sequence *)(untyped_member);
  return &member->data[index];
}

void hdl_graph_slam__msg__ScanMatchingStatus__rosidl_typesupport_introspection_c__fetch_function__ScanMatchingStatus__prediction_errors(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const geometry_msgs__msg__Pose * item =
    ((const geometry_msgs__msg__Pose *)
    hdl_graph_slam__msg__ScanMatchingStatus__rosidl_typesupport_introspection_c__get_const_function__ScanMatchingStatus__prediction_errors(untyped_member, index));
  geometry_msgs__msg__Pose * value =
    (geometry_msgs__msg__Pose *)(untyped_value);
  *value = *item;
}

void hdl_graph_slam__msg__ScanMatchingStatus__rosidl_typesupport_introspection_c__assign_function__ScanMatchingStatus__prediction_errors(
  void * untyped_member, size_t index, const void * untyped_value)
{
  geometry_msgs__msg__Pose * item =
    ((geometry_msgs__msg__Pose *)
    hdl_graph_slam__msg__ScanMatchingStatus__rosidl_typesupport_introspection_c__get_function__ScanMatchingStatus__prediction_errors(untyped_member, index));
  const geometry_msgs__msg__Pose * value =
    (const geometry_msgs__msg__Pose *)(untyped_value);
  *item = *value;
}

bool hdl_graph_slam__msg__ScanMatchingStatus__rosidl_typesupport_introspection_c__resize_function__ScanMatchingStatus__prediction_errors(
  void * untyped_member, size_t size)
{
  geometry_msgs__msg__Pose__Sequence * member =
    (geometry_msgs__msg__Pose__Sequence *)(untyped_member);
  geometry_msgs__msg__Pose__Sequence__fini(member);
  return geometry_msgs__msg__Pose__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember hdl_graph_slam__msg__ScanMatchingStatus__rosidl_typesupport_introspection_c__ScanMatchingStatus_message_member_array[7] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hdl_graph_slam__msg__ScanMatchingStatus, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "has_converged",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hdl_graph_slam__msg__ScanMatchingStatus, has_converged),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "matching_error",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hdl_graph_slam__msg__ScanMatchingStatus, matching_error),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "inlier_fraction",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hdl_graph_slam__msg__ScanMatchingStatus, inlier_fraction),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "relative_pose",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hdl_graph_slam__msg__ScanMatchingStatus, relative_pose),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "prediction_labels",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hdl_graph_slam__msg__ScanMatchingStatus, prediction_labels),  // bytes offset in struct
    NULL,  // default value
    hdl_graph_slam__msg__ScanMatchingStatus__rosidl_typesupport_introspection_c__size_function__ScanMatchingStatus__prediction_labels,  // size() function pointer
    hdl_graph_slam__msg__ScanMatchingStatus__rosidl_typesupport_introspection_c__get_const_function__ScanMatchingStatus__prediction_labels,  // get_const(index) function pointer
    hdl_graph_slam__msg__ScanMatchingStatus__rosidl_typesupport_introspection_c__get_function__ScanMatchingStatus__prediction_labels,  // get(index) function pointer
    hdl_graph_slam__msg__ScanMatchingStatus__rosidl_typesupport_introspection_c__fetch_function__ScanMatchingStatus__prediction_labels,  // fetch(index, &value) function pointer
    hdl_graph_slam__msg__ScanMatchingStatus__rosidl_typesupport_introspection_c__assign_function__ScanMatchingStatus__prediction_labels,  // assign(index, value) function pointer
    hdl_graph_slam__msg__ScanMatchingStatus__rosidl_typesupport_introspection_c__resize_function__ScanMatchingStatus__prediction_labels  // resize(index) function pointer
  },
  {
    "prediction_errors",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hdl_graph_slam__msg__ScanMatchingStatus, prediction_errors),  // bytes offset in struct
    NULL,  // default value
    hdl_graph_slam__msg__ScanMatchingStatus__rosidl_typesupport_introspection_c__size_function__ScanMatchingStatus__prediction_errors,  // size() function pointer
    hdl_graph_slam__msg__ScanMatchingStatus__rosidl_typesupport_introspection_c__get_const_function__ScanMatchingStatus__prediction_errors,  // get_const(index) function pointer
    hdl_graph_slam__msg__ScanMatchingStatus__rosidl_typesupport_introspection_c__get_function__ScanMatchingStatus__prediction_errors,  // get(index) function pointer
    hdl_graph_slam__msg__ScanMatchingStatus__rosidl_typesupport_introspection_c__fetch_function__ScanMatchingStatus__prediction_errors,  // fetch(index, &value) function pointer
    hdl_graph_slam__msg__ScanMatchingStatus__rosidl_typesupport_introspection_c__assign_function__ScanMatchingStatus__prediction_errors,  // assign(index, value) function pointer
    hdl_graph_slam__msg__ScanMatchingStatus__rosidl_typesupport_introspection_c__resize_function__ScanMatchingStatus__prediction_errors  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers hdl_graph_slam__msg__ScanMatchingStatus__rosidl_typesupport_introspection_c__ScanMatchingStatus_message_members = {
  "hdl_graph_slam__msg",  // message namespace
  "ScanMatchingStatus",  // message name
  7,  // number of fields
  sizeof(hdl_graph_slam__msg__ScanMatchingStatus),
  false,  // has_any_key_member_
  hdl_graph_slam__msg__ScanMatchingStatus__rosidl_typesupport_introspection_c__ScanMatchingStatus_message_member_array,  // message members
  hdl_graph_slam__msg__ScanMatchingStatus__rosidl_typesupport_introspection_c__ScanMatchingStatus_init_function,  // function to initialize message memory (memory has to be allocated)
  hdl_graph_slam__msg__ScanMatchingStatus__rosidl_typesupport_introspection_c__ScanMatchingStatus_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t hdl_graph_slam__msg__ScanMatchingStatus__rosidl_typesupport_introspection_c__ScanMatchingStatus_message_type_support_handle = {
  0,
  &hdl_graph_slam__msg__ScanMatchingStatus__rosidl_typesupport_introspection_c__ScanMatchingStatus_message_members,
  get_message_typesupport_handle_function,
  &hdl_graph_slam__msg__ScanMatchingStatus__get_type_hash,
  &hdl_graph_slam__msg__ScanMatchingStatus__get_type_description,
  &hdl_graph_slam__msg__ScanMatchingStatus__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_hdl_graph_slam
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hdl_graph_slam, msg, ScanMatchingStatus)() {
  hdl_graph_slam__msg__ScanMatchingStatus__rosidl_typesupport_introspection_c__ScanMatchingStatus_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  hdl_graph_slam__msg__ScanMatchingStatus__rosidl_typesupport_introspection_c__ScanMatchingStatus_message_member_array[4].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Pose)();
  hdl_graph_slam__msg__ScanMatchingStatus__rosidl_typesupport_introspection_c__ScanMatchingStatus_message_member_array[5].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, String)();
  hdl_graph_slam__msg__ScanMatchingStatus__rosidl_typesupport_introspection_c__ScanMatchingStatus_message_member_array[6].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Pose)();
  if (!hdl_graph_slam__msg__ScanMatchingStatus__rosidl_typesupport_introspection_c__ScanMatchingStatus_message_type_support_handle.typesupport_identifier) {
    hdl_graph_slam__msg__ScanMatchingStatus__rosidl_typesupport_introspection_c__ScanMatchingStatus_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &hdl_graph_slam__msg__ScanMatchingStatus__rosidl_typesupport_introspection_c__ScanMatchingStatus_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
