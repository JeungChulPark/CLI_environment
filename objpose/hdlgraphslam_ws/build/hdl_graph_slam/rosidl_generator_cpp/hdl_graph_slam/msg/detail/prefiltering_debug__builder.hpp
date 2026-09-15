// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from hdl_graph_slam:msg/PrefilteringDebug.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "hdl_graph_slam/msg/prefiltering_debug.hpp"


#ifndef HDL_GRAPH_SLAM__MSG__DETAIL__PREFILTERING_DEBUG__BUILDER_HPP_
#define HDL_GRAPH_SLAM__MSG__DETAIL__PREFILTERING_DEBUG__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "hdl_graph_slam/msg/detail/prefiltering_debug__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace hdl_graph_slam
{

namespace msg
{

namespace builder
{

class Init_PrefilteringDebug_output_count
{
public:
  explicit Init_PrefilteringDebug_output_count(::hdl_graph_slam::msg::PrefilteringDebug & msg)
  : msg_(msg)
  {}
  ::hdl_graph_slam::msg::PrefilteringDebug output_count(::hdl_graph_slam::msg::PrefilteringDebug::_output_count_type arg)
  {
    msg_.output_count = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hdl_graph_slam::msg::PrefilteringDebug msg_;
};

class Init_PrefilteringDebug_callback_time_ms
{
public:
  explicit Init_PrefilteringDebug_callback_time_ms(::hdl_graph_slam::msg::PrefilteringDebug & msg)
  : msg_(msg)
  {}
  Init_PrefilteringDebug_output_count callback_time_ms(::hdl_graph_slam::msg::PrefilteringDebug::_callback_time_ms_type arg)
  {
    msg_.callback_time_ms = std::move(arg);
    return Init_PrefilteringDebug_output_count(msg_);
  }

private:
  ::hdl_graph_slam::msg::PrefilteringDebug msg_;
};

class Init_PrefilteringDebug_output_point_count
{
public:
  explicit Init_PrefilteringDebug_output_point_count(::hdl_graph_slam::msg::PrefilteringDebug & msg)
  : msg_(msg)
  {}
  Init_PrefilteringDebug_callback_time_ms output_point_count(::hdl_graph_slam::msg::PrefilteringDebug::_output_point_count_type arg)
  {
    msg_.output_point_count = std::move(arg);
    return Init_PrefilteringDebug_callback_time_ms(msg_);
  }

private:
  ::hdl_graph_slam::msg::PrefilteringDebug msg_;
};

class Init_PrefilteringDebug_downsampled_point_count
{
public:
  explicit Init_PrefilteringDebug_downsampled_point_count(::hdl_graph_slam::msg::PrefilteringDebug & msg)
  : msg_(msg)
  {}
  Init_PrefilteringDebug_output_point_count downsampled_point_count(::hdl_graph_slam::msg::PrefilteringDebug::_downsampled_point_count_type arg)
  {
    msg_.downsampled_point_count = std::move(arg);
    return Init_PrefilteringDebug_output_point_count(msg_);
  }

private:
  ::hdl_graph_slam::msg::PrefilteringDebug msg_;
};

class Init_PrefilteringDebug_distance_filtered_point_count
{
public:
  explicit Init_PrefilteringDebug_distance_filtered_point_count(::hdl_graph_slam::msg::PrefilteringDebug & msg)
  : msg_(msg)
  {}
  Init_PrefilteringDebug_downsampled_point_count distance_filtered_point_count(::hdl_graph_slam::msg::PrefilteringDebug::_distance_filtered_point_count_type arg)
  {
    msg_.distance_filtered_point_count = std::move(arg);
    return Init_PrefilteringDebug_downsampled_point_count(msg_);
  }

private:
  ::hdl_graph_slam::msg::PrefilteringDebug msg_;
};

class Init_PrefilteringDebug_transformed_point_count
{
public:
  explicit Init_PrefilteringDebug_transformed_point_count(::hdl_graph_slam::msg::PrefilteringDebug & msg)
  : msg_(msg)
  {}
  Init_PrefilteringDebug_distance_filtered_point_count transformed_point_count(::hdl_graph_slam::msg::PrefilteringDebug::_transformed_point_count_type arg)
  {
    msg_.transformed_point_count = std::move(arg);
    return Init_PrefilteringDebug_distance_filtered_point_count(msg_);
  }

private:
  ::hdl_graph_slam::msg::PrefilteringDebug msg_;
};

class Init_PrefilteringDebug_input_point_count
{
public:
  explicit Init_PrefilteringDebug_input_point_count(::hdl_graph_slam::msg::PrefilteringDebug & msg)
  : msg_(msg)
  {}
  Init_PrefilteringDebug_transformed_point_count input_point_count(::hdl_graph_slam::msg::PrefilteringDebug::_input_point_count_type arg)
  {
    msg_.input_point_count = std::move(arg);
    return Init_PrefilteringDebug_transformed_point_count(msg_);
  }

private:
  ::hdl_graph_slam::msg::PrefilteringDebug msg_;
};

class Init_PrefilteringDebug_header
{
public:
  Init_PrefilteringDebug_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PrefilteringDebug_input_point_count header(::hdl_graph_slam::msg::PrefilteringDebug::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_PrefilteringDebug_input_point_count(msg_);
  }

private:
  ::hdl_graph_slam::msg::PrefilteringDebug msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::hdl_graph_slam::msg::PrefilteringDebug>()
{
  return hdl_graph_slam::msg::builder::Init_PrefilteringDebug_header();
}

}  // namespace hdl_graph_slam

#endif  // HDL_GRAPH_SLAM__MSG__DETAIL__PREFILTERING_DEBUG__BUILDER_HPP_
