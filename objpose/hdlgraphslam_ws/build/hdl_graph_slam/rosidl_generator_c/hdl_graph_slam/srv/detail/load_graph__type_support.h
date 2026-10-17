// generated from rosidl_generator_c/resource/idl__type_support.h.em
// with input from hdl_graph_slam:srv/LoadGraph.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "hdl_graph_slam/srv/load_graph.h"


#ifndef HDL_GRAPH_SLAM__SRV__DETAIL__LOAD_GRAPH__TYPE_SUPPORT_H_
#define HDL_GRAPH_SLAM__SRV__DETAIL__LOAD_GRAPH__TYPE_SUPPORT_H_

#include "rosidl_typesupport_interface/macros.h"

#include "hdl_graph_slam/msg/rosidl_generator_c__visibility_control.h"

#ifdef __cplusplus
extern "C"
{
#endif

#include "rosidl_runtime_c/message_type_support_struct.h"

// Forward declare the get type support functions for this type.
ROSIDL_GENERATOR_C_PUBLIC_hdl_graph_slam
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
  rosidl_typesupport_c,
  hdl_graph_slam,
  srv,
  LoadGraph_Request
)(void);

// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"

// Forward declare the get type support functions for this type.
ROSIDL_GENERATOR_C_PUBLIC_hdl_graph_slam
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
  rosidl_typesupport_c,
  hdl_graph_slam,
  srv,
  LoadGraph_Response
)(void);

// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"

// Forward declare the get type support functions for this type.
ROSIDL_GENERATOR_C_PUBLIC_hdl_graph_slam
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
  rosidl_typesupport_c,
  hdl_graph_slam,
  srv,
  LoadGraph_Event
)(void);

#include "rosidl_runtime_c/service_type_support_struct.h"

// Forward declare the get type support functions for this type.
ROSIDL_GENERATOR_C_PUBLIC_hdl_graph_slam
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(
  rosidl_typesupport_c,
  hdl_graph_slam,
  srv,
  LoadGraph
)(void);

// Forward declare the function to create a service event message for this type.
ROSIDL_GENERATOR_C_PUBLIC_hdl_graph_slam
void *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_CREATE_EVENT_MESSAGE_SYMBOL_NAME(
  rosidl_typesupport_c,
  hdl_graph_slam,
  srv,
  LoadGraph
)(
  const rosidl_service_introspection_info_t * info,
  rcutils_allocator_t * allocator,
  const void * request_message,
  const void * response_message);

// Forward declare the function to destroy a service event message for this type.
ROSIDL_GENERATOR_C_PUBLIC_hdl_graph_slam
bool
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_DESTROY_EVENT_MESSAGE_SYMBOL_NAME(
  rosidl_typesupport_c,
  hdl_graph_slam,
  srv,
  LoadGraph
)(
  void * event_msg,
  rcutils_allocator_t * allocator);

#ifdef __cplusplus
}
#endif

#endif  // HDL_GRAPH_SLAM__SRV__DETAIL__LOAD_GRAPH__TYPE_SUPPORT_H_
