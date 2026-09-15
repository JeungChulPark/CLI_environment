// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from hdl_graph_slam:msg/FloorCoeffs.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "hdl_graph_slam/msg/detail/floor_coeffs__rosidl_typesupport_introspection_c.h"
#include "hdl_graph_slam/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "hdl_graph_slam/msg/detail/floor_coeffs__functions.h"
#include "hdl_graph_slam/msg/detail/floor_coeffs__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `coeffs`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void hdl_graph_slam__msg__FloorCoeffs__rosidl_typesupport_introspection_c__FloorCoeffs_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  hdl_graph_slam__msg__FloorCoeffs__init(message_memory);
}

void hdl_graph_slam__msg__FloorCoeffs__rosidl_typesupport_introspection_c__FloorCoeffs_fini_function(void * message_memory)
{
  hdl_graph_slam__msg__FloorCoeffs__fini(message_memory);
}

size_t hdl_graph_slam__msg__FloorCoeffs__rosidl_typesupport_introspection_c__size_function__FloorCoeffs__coeffs(
  const void * untyped_member)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return member->size;
}

const void * hdl_graph_slam__msg__FloorCoeffs__rosidl_typesupport_introspection_c__get_const_function__FloorCoeffs__coeffs(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void * hdl_graph_slam__msg__FloorCoeffs__rosidl_typesupport_introspection_c__get_function__FloorCoeffs__coeffs(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void hdl_graph_slam__msg__FloorCoeffs__rosidl_typesupport_introspection_c__fetch_function__FloorCoeffs__coeffs(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    hdl_graph_slam__msg__FloorCoeffs__rosidl_typesupport_introspection_c__get_const_function__FloorCoeffs__coeffs(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void hdl_graph_slam__msg__FloorCoeffs__rosidl_typesupport_introspection_c__assign_function__FloorCoeffs__coeffs(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    hdl_graph_slam__msg__FloorCoeffs__rosidl_typesupport_introspection_c__get_function__FloorCoeffs__coeffs(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

bool hdl_graph_slam__msg__FloorCoeffs__rosidl_typesupport_introspection_c__resize_function__FloorCoeffs__coeffs(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  rosidl_runtime_c__float__Sequence__fini(member);
  return rosidl_runtime_c__float__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember hdl_graph_slam__msg__FloorCoeffs__rosidl_typesupport_introspection_c__FloorCoeffs_message_member_array[2] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hdl_graph_slam__msg__FloorCoeffs, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "coeffs",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hdl_graph_slam__msg__FloorCoeffs, coeffs),  // bytes offset in struct
    NULL,  // default value
    hdl_graph_slam__msg__FloorCoeffs__rosidl_typesupport_introspection_c__size_function__FloorCoeffs__coeffs,  // size() function pointer
    hdl_graph_slam__msg__FloorCoeffs__rosidl_typesupport_introspection_c__get_const_function__FloorCoeffs__coeffs,  // get_const(index) function pointer
    hdl_graph_slam__msg__FloorCoeffs__rosidl_typesupport_introspection_c__get_function__FloorCoeffs__coeffs,  // get(index) function pointer
    hdl_graph_slam__msg__FloorCoeffs__rosidl_typesupport_introspection_c__fetch_function__FloorCoeffs__coeffs,  // fetch(index, &value) function pointer
    hdl_graph_slam__msg__FloorCoeffs__rosidl_typesupport_introspection_c__assign_function__FloorCoeffs__coeffs,  // assign(index, value) function pointer
    hdl_graph_slam__msg__FloorCoeffs__rosidl_typesupport_introspection_c__resize_function__FloorCoeffs__coeffs  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers hdl_graph_slam__msg__FloorCoeffs__rosidl_typesupport_introspection_c__FloorCoeffs_message_members = {
  "hdl_graph_slam__msg",  // message namespace
  "FloorCoeffs",  // message name
  2,  // number of fields
  sizeof(hdl_graph_slam__msg__FloorCoeffs),
  false,  // has_any_key_member_
  hdl_graph_slam__msg__FloorCoeffs__rosidl_typesupport_introspection_c__FloorCoeffs_message_member_array,  // message members
  hdl_graph_slam__msg__FloorCoeffs__rosidl_typesupport_introspection_c__FloorCoeffs_init_function,  // function to initialize message memory (memory has to be allocated)
  hdl_graph_slam__msg__FloorCoeffs__rosidl_typesupport_introspection_c__FloorCoeffs_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t hdl_graph_slam__msg__FloorCoeffs__rosidl_typesupport_introspection_c__FloorCoeffs_message_type_support_handle = {
  0,
  &hdl_graph_slam__msg__FloorCoeffs__rosidl_typesupport_introspection_c__FloorCoeffs_message_members,
  get_message_typesupport_handle_function,
  &hdl_graph_slam__msg__FloorCoeffs__get_type_hash,
  &hdl_graph_slam__msg__FloorCoeffs__get_type_description,
  &hdl_graph_slam__msg__FloorCoeffs__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_hdl_graph_slam
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hdl_graph_slam, msg, FloorCoeffs)() {
  hdl_graph_slam__msg__FloorCoeffs__rosidl_typesupport_introspection_c__FloorCoeffs_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!hdl_graph_slam__msg__FloorCoeffs__rosidl_typesupport_introspection_c__FloorCoeffs_message_type_support_handle.typesupport_identifier) {
    hdl_graph_slam__msg__FloorCoeffs__rosidl_typesupport_introspection_c__FloorCoeffs_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &hdl_graph_slam__msg__FloorCoeffs__rosidl_typesupport_introspection_c__FloorCoeffs_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
