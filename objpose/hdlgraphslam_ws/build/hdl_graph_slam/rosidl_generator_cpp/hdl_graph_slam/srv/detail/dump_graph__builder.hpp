// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from hdl_graph_slam:srv/DumpGraph.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "hdl_graph_slam/srv/dump_graph.hpp"


#ifndef HDL_GRAPH_SLAM__SRV__DETAIL__DUMP_GRAPH__BUILDER_HPP_
#define HDL_GRAPH_SLAM__SRV__DETAIL__DUMP_GRAPH__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "hdl_graph_slam/srv/detail/dump_graph__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace hdl_graph_slam
{

namespace srv
{

namespace builder
{

class Init_DumpGraph_Request_destination
{
public:
  Init_DumpGraph_Request_destination()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::hdl_graph_slam::srv::DumpGraph_Request destination(::hdl_graph_slam::srv::DumpGraph_Request::_destination_type arg)
  {
    msg_.destination = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hdl_graph_slam::srv::DumpGraph_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::hdl_graph_slam::srv::DumpGraph_Request>()
{
  return hdl_graph_slam::srv::builder::Init_DumpGraph_Request_destination();
}

}  // namespace hdl_graph_slam


namespace hdl_graph_slam
{

namespace srv
{

namespace builder
{

class Init_DumpGraph_Response_success
{
public:
  Init_DumpGraph_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::hdl_graph_slam::srv::DumpGraph_Response success(::hdl_graph_slam::srv::DumpGraph_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hdl_graph_slam::srv::DumpGraph_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::hdl_graph_slam::srv::DumpGraph_Response>()
{
  return hdl_graph_slam::srv::builder::Init_DumpGraph_Response_success();
}

}  // namespace hdl_graph_slam


namespace hdl_graph_slam
{

namespace srv
{

namespace builder
{

class Init_DumpGraph_Event_response
{
public:
  explicit Init_DumpGraph_Event_response(::hdl_graph_slam::srv::DumpGraph_Event & msg)
  : msg_(msg)
  {}
  ::hdl_graph_slam::srv::DumpGraph_Event response(::hdl_graph_slam::srv::DumpGraph_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hdl_graph_slam::srv::DumpGraph_Event msg_;
};

class Init_DumpGraph_Event_request
{
public:
  explicit Init_DumpGraph_Event_request(::hdl_graph_slam::srv::DumpGraph_Event & msg)
  : msg_(msg)
  {}
  Init_DumpGraph_Event_response request(::hdl_graph_slam::srv::DumpGraph_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_DumpGraph_Event_response(msg_);
  }

private:
  ::hdl_graph_slam::srv::DumpGraph_Event msg_;
};

class Init_DumpGraph_Event_info
{
public:
  Init_DumpGraph_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DumpGraph_Event_request info(::hdl_graph_slam::srv::DumpGraph_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_DumpGraph_Event_request(msg_);
  }

private:
  ::hdl_graph_slam::srv::DumpGraph_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::hdl_graph_slam::srv::DumpGraph_Event>()
{
  return hdl_graph_slam::srv::builder::Init_DumpGraph_Event_info();
}

}  // namespace hdl_graph_slam

#endif  // HDL_GRAPH_SLAM__SRV__DETAIL__DUMP_GRAPH__BUILDER_HPP_
