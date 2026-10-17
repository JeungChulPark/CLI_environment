// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from hdl_graph_slam:srv/LoadGraph.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "hdl_graph_slam/srv/load_graph.h"


#ifndef HDL_GRAPH_SLAM__SRV__DETAIL__LOAD_GRAPH__STRUCT_H_
#define HDL_GRAPH_SLAM__SRV__DETAIL__LOAD_GRAPH__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'path'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/LoadGraph in the package hdl_graph_slam.
typedef struct hdl_graph_slam__srv__LoadGraph_Request
{
  rosidl_runtime_c__String path;
} hdl_graph_slam__srv__LoadGraph_Request;

// Struct for a sequence of hdl_graph_slam__srv__LoadGraph_Request.
typedef struct hdl_graph_slam__srv__LoadGraph_Request__Sequence
{
  hdl_graph_slam__srv__LoadGraph_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hdl_graph_slam__srv__LoadGraph_Request__Sequence;

// Constants defined in the message

/// Struct defined in srv/LoadGraph in the package hdl_graph_slam.
typedef struct hdl_graph_slam__srv__LoadGraph_Response
{
  bool success;
} hdl_graph_slam__srv__LoadGraph_Response;

// Struct for a sequence of hdl_graph_slam__srv__LoadGraph_Response.
typedef struct hdl_graph_slam__srv__LoadGraph_Response__Sequence
{
  hdl_graph_slam__srv__LoadGraph_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hdl_graph_slam__srv__LoadGraph_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  hdl_graph_slam__srv__LoadGraph_Event__request__MAX_SIZE = 1
};
// response
enum
{
  hdl_graph_slam__srv__LoadGraph_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/LoadGraph in the package hdl_graph_slam.
typedef struct hdl_graph_slam__srv__LoadGraph_Event
{
  service_msgs__msg__ServiceEventInfo info;
  hdl_graph_slam__srv__LoadGraph_Request__Sequence request;
  hdl_graph_slam__srv__LoadGraph_Response__Sequence response;
} hdl_graph_slam__srv__LoadGraph_Event;

// Struct for a sequence of hdl_graph_slam__srv__LoadGraph_Event.
typedef struct hdl_graph_slam__srv__LoadGraph_Event__Sequence
{
  hdl_graph_slam__srv__LoadGraph_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hdl_graph_slam__srv__LoadGraph_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HDL_GRAPH_SLAM__SRV__DETAIL__LOAD_GRAPH__STRUCT_H_
