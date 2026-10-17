// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from hdl_graph_slam:msg/PrefilteringDebug.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "hdl_graph_slam/msg/detail/prefiltering_debug__rosidl_typesupport_introspection_c.h"
#include "hdl_graph_slam/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "hdl_graph_slam/msg/detail/prefiltering_debug__functions.h"
#include "hdl_graph_slam/msg/detail/prefiltering_debug__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void hdl_graph_slam__msg__PrefilteringDebug__rosidl_typesupport_introspection_c__PrefilteringDebug_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  hdl_graph_slam__msg__PrefilteringDebug__init(message_memory);
}

void hdl_graph_slam__msg__PrefilteringDebug__rosidl_typesupport_introspection_c__PrefilteringDebug_fini_function(void * message_memory)
{
  hdl_graph_slam__msg__PrefilteringDebug__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember hdl_graph_slam__msg__PrefilteringDebug__rosidl_typesupport_introspection_c__PrefilteringDebug_message_member_array[8] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hdl_graph_slam__msg__PrefilteringDebug, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "input_point_count",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hdl_graph_slam__msg__PrefilteringDebug, input_point_count),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "transformed_point_count",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hdl_graph_slam__msg__PrefilteringDebug, transformed_point_count),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "distance_filtered_point_count",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hdl_graph_slam__msg__PrefilteringDebug, distance_filtered_point_count),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "downsampled_point_count",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hdl_graph_slam__msg__PrefilteringDebug, downsampled_point_count),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "output_point_count",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hdl_graph_slam__msg__PrefilteringDebug, output_point_count),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "callback_time_ms",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hdl_graph_slam__msg__PrefilteringDebug, callback_time_ms),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "output_count",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hdl_graph_slam__msg__PrefilteringDebug, output_count),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers hdl_graph_slam__msg__PrefilteringDebug__rosidl_typesupport_introspection_c__PrefilteringDebug_message_members = {
  "hdl_graph_slam__msg",  // message namespace
  "PrefilteringDebug",  // message name
  8,  // number of fields
  sizeof(hdl_graph_slam__msg__PrefilteringDebug),
  false,  // has_any_key_member_
  hdl_graph_slam__msg__PrefilteringDebug__rosidl_typesupport_introspection_c__PrefilteringDebug_message_member_array,  // message members
  hdl_graph_slam__msg__PrefilteringDebug__rosidl_typesupport_introspection_c__PrefilteringDebug_init_function,  // function to initialize message memory (memory has to be allocated)
  hdl_graph_slam__msg__PrefilteringDebug__rosidl_typesupport_introspection_c__PrefilteringDebug_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t hdl_graph_slam__msg__PrefilteringDebug__rosidl_typesupport_introspection_c__PrefilteringDebug_message_type_support_handle = {
  0,
  &hdl_graph_slam__msg__PrefilteringDebug__rosidl_typesupport_introspection_c__PrefilteringDebug_message_members,
  get_message_typesupport_handle_function,
  &hdl_graph_slam__msg__PrefilteringDebug__get_type_hash,
  &hdl_graph_slam__msg__PrefilteringDebug__get_type_description,
  &hdl_graph_slam__msg__PrefilteringDebug__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_hdl_graph_slam
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hdl_graph_slam, msg, PrefilteringDebug)() {
  hdl_graph_slam__msg__PrefilteringDebug__rosidl_typesupport_introspection_c__PrefilteringDebug_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!hdl_graph_slam__msg__PrefilteringDebug__rosidl_typesupport_introspection_c__PrefilteringDebug_message_type_support_handle.typesupport_identifier) {
    hdl_graph_slam__msg__PrefilteringDebug__rosidl_typesupport_introspection_c__PrefilteringDebug_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &hdl_graph_slam__msg__PrefilteringDebug__rosidl_typesupport_introspection_c__PrefilteringDebug_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
