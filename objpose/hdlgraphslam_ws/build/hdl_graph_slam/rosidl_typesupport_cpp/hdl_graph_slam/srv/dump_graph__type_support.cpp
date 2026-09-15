// generated from rosidl_typesupport_cpp/resource/idl__type_support.cpp.em
// with input from hdl_graph_slam:srv/DumpGraph.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "hdl_graph_slam/srv/detail/dump_graph__functions.h"
#include "hdl_graph_slam/srv/detail/dump_graph__struct.hpp"
#include "rosidl_typesupport_cpp/identifier.hpp"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
#include "rosidl_typesupport_cpp/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace hdl_graph_slam
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _DumpGraph_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _DumpGraph_Request_type_support_ids_t;

static const _DumpGraph_Request_type_support_ids_t _DumpGraph_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _DumpGraph_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _DumpGraph_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _DumpGraph_Request_type_support_symbol_names_t _DumpGraph_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, hdl_graph_slam, srv, DumpGraph_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, hdl_graph_slam, srv, DumpGraph_Request)),
  }
};

typedef struct _DumpGraph_Request_type_support_data_t
{
  void * data[2];
} _DumpGraph_Request_type_support_data_t;

static _DumpGraph_Request_type_support_data_t _DumpGraph_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _DumpGraph_Request_message_typesupport_map = {
  2,
  "hdl_graph_slam",
  &_DumpGraph_Request_message_typesupport_ids.typesupport_identifier[0],
  &_DumpGraph_Request_message_typesupport_symbol_names.symbol_name[0],
  &_DumpGraph_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t DumpGraph_Request_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_DumpGraph_Request_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &hdl_graph_slam__srv__DumpGraph_Request__get_type_hash,
  &hdl_graph_slam__srv__DumpGraph_Request__get_type_description,
  &hdl_graph_slam__srv__DumpGraph_Request__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace hdl_graph_slam

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<hdl_graph_slam::srv::DumpGraph_Request>()
{
  return &::hdl_graph_slam::srv::rosidl_typesupport_cpp::DumpGraph_Request_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, hdl_graph_slam, srv, DumpGraph_Request)() {
  return get_message_type_support_handle<hdl_graph_slam::srv::DumpGraph_Request>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "hdl_graph_slam/srv/detail/dump_graph__functions.h"
// already included above
// #include "hdl_graph_slam/srv/detail/dump_graph__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace hdl_graph_slam
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _DumpGraph_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _DumpGraph_Response_type_support_ids_t;

static const _DumpGraph_Response_type_support_ids_t _DumpGraph_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _DumpGraph_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _DumpGraph_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _DumpGraph_Response_type_support_symbol_names_t _DumpGraph_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, hdl_graph_slam, srv, DumpGraph_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, hdl_graph_slam, srv, DumpGraph_Response)),
  }
};

typedef struct _DumpGraph_Response_type_support_data_t
{
  void * data[2];
} _DumpGraph_Response_type_support_data_t;

static _DumpGraph_Response_type_support_data_t _DumpGraph_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _DumpGraph_Response_message_typesupport_map = {
  2,
  "hdl_graph_slam",
  &_DumpGraph_Response_message_typesupport_ids.typesupport_identifier[0],
  &_DumpGraph_Response_message_typesupport_symbol_names.symbol_name[0],
  &_DumpGraph_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t DumpGraph_Response_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_DumpGraph_Response_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &hdl_graph_slam__srv__DumpGraph_Response__get_type_hash,
  &hdl_graph_slam__srv__DumpGraph_Response__get_type_description,
  &hdl_graph_slam__srv__DumpGraph_Response__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace hdl_graph_slam

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<hdl_graph_slam::srv::DumpGraph_Response>()
{
  return &::hdl_graph_slam::srv::rosidl_typesupport_cpp::DumpGraph_Response_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, hdl_graph_slam, srv, DumpGraph_Response)() {
  return get_message_type_support_handle<hdl_graph_slam::srv::DumpGraph_Response>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "hdl_graph_slam/srv/detail/dump_graph__functions.h"
// already included above
// #include "hdl_graph_slam/srv/detail/dump_graph__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace hdl_graph_slam
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _DumpGraph_Event_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _DumpGraph_Event_type_support_ids_t;

static const _DumpGraph_Event_type_support_ids_t _DumpGraph_Event_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _DumpGraph_Event_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _DumpGraph_Event_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _DumpGraph_Event_type_support_symbol_names_t _DumpGraph_Event_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, hdl_graph_slam, srv, DumpGraph_Event)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, hdl_graph_slam, srv, DumpGraph_Event)),
  }
};

typedef struct _DumpGraph_Event_type_support_data_t
{
  void * data[2];
} _DumpGraph_Event_type_support_data_t;

static _DumpGraph_Event_type_support_data_t _DumpGraph_Event_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _DumpGraph_Event_message_typesupport_map = {
  2,
  "hdl_graph_slam",
  &_DumpGraph_Event_message_typesupport_ids.typesupport_identifier[0],
  &_DumpGraph_Event_message_typesupport_symbol_names.symbol_name[0],
  &_DumpGraph_Event_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t DumpGraph_Event_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_DumpGraph_Event_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &hdl_graph_slam__srv__DumpGraph_Event__get_type_hash,
  &hdl_graph_slam__srv__DumpGraph_Event__get_type_description,
  &hdl_graph_slam__srv__DumpGraph_Event__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace hdl_graph_slam

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<hdl_graph_slam::srv::DumpGraph_Event>()
{
  return &::hdl_graph_slam::srv::rosidl_typesupport_cpp::DumpGraph_Event_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, hdl_graph_slam, srv, DumpGraph_Event)() {
  return get_message_type_support_handle<hdl_graph_slam::srv::DumpGraph_Event>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "hdl_graph_slam/srv/detail/dump_graph__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/service_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace hdl_graph_slam
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _DumpGraph_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _DumpGraph_type_support_ids_t;

static const _DumpGraph_type_support_ids_t _DumpGraph_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _DumpGraph_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _DumpGraph_type_support_symbol_names_t;
#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _DumpGraph_type_support_symbol_names_t _DumpGraph_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, hdl_graph_slam, srv, DumpGraph)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, hdl_graph_slam, srv, DumpGraph)),
  }
};

typedef struct _DumpGraph_type_support_data_t
{
  void * data[2];
} _DumpGraph_type_support_data_t;

static _DumpGraph_type_support_data_t _DumpGraph_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _DumpGraph_service_typesupport_map = {
  2,
  "hdl_graph_slam",
  &_DumpGraph_service_typesupport_ids.typesupport_identifier[0],
  &_DumpGraph_service_typesupport_symbol_names.symbol_name[0],
  &_DumpGraph_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t DumpGraph_service_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_DumpGraph_service_typesupport_map),
  ::rosidl_typesupport_cpp::get_service_typesupport_handle_function,
  ::rosidl_typesupport_cpp::get_message_type_support_handle<hdl_graph_slam::srv::DumpGraph_Request>(),
  ::rosidl_typesupport_cpp::get_message_type_support_handle<hdl_graph_slam::srv::DumpGraph_Response>(),
  ::rosidl_typesupport_cpp::get_message_type_support_handle<hdl_graph_slam::srv::DumpGraph_Event>(),
  &::rosidl_typesupport_cpp::service_create_event_message<hdl_graph_slam::srv::DumpGraph>,
  &::rosidl_typesupport_cpp::service_destroy_event_message<hdl_graph_slam::srv::DumpGraph>,
  &hdl_graph_slam__srv__DumpGraph__get_type_hash,
  &hdl_graph_slam__srv__DumpGraph__get_type_description,
  &hdl_graph_slam__srv__DumpGraph__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace hdl_graph_slam

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<hdl_graph_slam::srv::DumpGraph>()
{
  return &::hdl_graph_slam::srv::rosidl_typesupport_cpp::DumpGraph_service_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_cpp, hdl_graph_slam, srv, DumpGraph)() {
  return ::rosidl_typesupport_cpp::get_service_type_support_handle<hdl_graph_slam::srv::DumpGraph>();
}

#ifdef __cplusplus
}
#endif
