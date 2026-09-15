// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from hdl_graph_slam:srv/LoadGraph.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "hdl_graph_slam/srv/load_graph.hpp"


#ifndef HDL_GRAPH_SLAM__SRV__DETAIL__LOAD_GRAPH__BUILDER_HPP_
#define HDL_GRAPH_SLAM__SRV__DETAIL__LOAD_GRAPH__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "hdl_graph_slam/srv/detail/load_graph__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace hdl_graph_slam
{

namespace srv
{

namespace builder
{

class Init_LoadGraph_Request_path
{
public:
  Init_LoadGraph_Request_path()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::hdl_graph_slam::srv::LoadGraph_Request path(::hdl_graph_slam::srv::LoadGraph_Request::_path_type arg)
  {
    msg_.path = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hdl_graph_slam::srv::LoadGraph_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::hdl_graph_slam::srv::LoadGraph_Request>()
{
  return hdl_graph_slam::srv::builder::Init_LoadGraph_Request_path();
}

}  // namespace hdl_graph_slam


namespace hdl_graph_slam
{

namespace srv
{

namespace builder
{

class Init_LoadGraph_Response_success
{
public:
  Init_LoadGraph_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::hdl_graph_slam::srv::LoadGraph_Response success(::hdl_graph_slam::srv::LoadGraph_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hdl_graph_slam::srv::LoadGraph_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::hdl_graph_slam::srv::LoadGraph_Response>()
{
  return hdl_graph_slam::srv::builder::Init_LoadGraph_Response_success();
}

}  // namespace hdl_graph_slam


namespace hdl_graph_slam
{

namespace srv
{

namespace builder
{

class Init_LoadGraph_Event_response
{
public:
  explicit Init_LoadGraph_Event_response(::hdl_graph_slam::srv::LoadGraph_Event & msg)
  : msg_(msg)
  {}
  ::hdl_graph_slam::srv::LoadGraph_Event response(::hdl_graph_slam::srv::LoadGraph_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hdl_graph_slam::srv::LoadGraph_Event msg_;
};

class Init_LoadGraph_Event_request
{
public:
  explicit Init_LoadGraph_Event_request(::hdl_graph_slam::srv::LoadGraph_Event & msg)
  : msg_(msg)
  {}
  Init_LoadGraph_Event_response request(::hdl_graph_slam::srv::LoadGraph_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_LoadGraph_Event_response(msg_);
  }

private:
  ::hdl_graph_slam::srv::LoadGraph_Event msg_;
};

class Init_LoadGraph_Event_info
{
public:
  Init_LoadGraph_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LoadGraph_Event_request info(::hdl_graph_slam::srv::LoadGraph_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_LoadGraph_Event_request(msg_);
  }

private:
  ::hdl_graph_slam::srv::LoadGraph_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::hdl_graph_slam::srv::LoadGraph_Event>()
{
  return hdl_graph_slam::srv::builder::Init_LoadGraph_Event_info();
}

}  // namespace hdl_graph_slam

#endif  // HDL_GRAPH_SLAM__SRV__DETAIL__LOAD_GRAPH__BUILDER_HPP_
