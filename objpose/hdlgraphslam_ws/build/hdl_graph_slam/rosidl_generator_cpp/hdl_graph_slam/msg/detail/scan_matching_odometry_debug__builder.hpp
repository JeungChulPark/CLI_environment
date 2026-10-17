// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from hdl_graph_slam:msg/ScanMatchingOdometryDebug.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "hdl_graph_slam/msg/scan_matching_odometry_debug.hpp"


#ifndef HDL_GRAPH_SLAM__MSG__DETAIL__SCAN_MATCHING_ODOMETRY_DEBUG__BUILDER_HPP_
#define HDL_GRAPH_SLAM__MSG__DETAIL__SCAN_MATCHING_ODOMETRY_DEBUG__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "hdl_graph_slam/msg/detail/scan_matching_odometry_debug__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace hdl_graph_slam
{

namespace msg
{

namespace builder
{

class Init_ScanMatchingOdometryDebug_matching_error
{
public:
  explicit Init_ScanMatchingOdometryDebug_matching_error(::hdl_graph_slam::msg::ScanMatchingOdometryDebug & msg)
  : msg_(msg)
  {}
  ::hdl_graph_slam::msg::ScanMatchingOdometryDebug matching_error(::hdl_graph_slam::msg::ScanMatchingOdometryDebug::_matching_error_type arg)
  {
    msg_.matching_error = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hdl_graph_slam::msg::ScanMatchingOdometryDebug msg_;
};

class Init_ScanMatchingOdometryDebug_has_converged
{
public:
  explicit Init_ScanMatchingOdometryDebug_has_converged(::hdl_graph_slam::msg::ScanMatchingOdometryDebug & msg)
  : msg_(msg)
  {}
  Init_ScanMatchingOdometryDebug_matching_error has_converged(::hdl_graph_slam::msg::ScanMatchingOdometryDebug::_has_converged_type arg)
  {
    msg_.has_converged = std::move(arg);
    return Init_ScanMatchingOdometryDebug_matching_error(msg_);
  }

private:
  ::hdl_graph_slam::msg::ScanMatchingOdometryDebug msg_;
};

class Init_ScanMatchingOdometryDebug_registration_triggered
{
public:
  explicit Init_ScanMatchingOdometryDebug_registration_triggered(::hdl_graph_slam::msg::ScanMatchingOdometryDebug & msg)
  : msg_(msg)
  {}
  Init_ScanMatchingOdometryDebug_has_converged registration_triggered(::hdl_graph_slam::msg::ScanMatchingOdometryDebug::_registration_triggered_type arg)
  {
    msg_.registration_triggered = std::move(arg);
    return Init_ScanMatchingOdometryDebug_has_converged(msg_);
  }

private:
  ::hdl_graph_slam::msg::ScanMatchingOdometryDebug msg_;
};

class Init_ScanMatchingOdometryDebug_odom_count
{
public:
  explicit Init_ScanMatchingOdometryDebug_odom_count(::hdl_graph_slam::msg::ScanMatchingOdometryDebug & msg)
  : msg_(msg)
  {}
  Init_ScanMatchingOdometryDebug_registration_triggered odom_count(::hdl_graph_slam::msg::ScanMatchingOdometryDebug::_odom_count_type arg)
  {
    msg_.odom_count = std::move(arg);
    return Init_ScanMatchingOdometryDebug_registration_triggered(msg_);
  }

private:
  ::hdl_graph_slam::msg::ScanMatchingOdometryDebug msg_;
};

class Init_ScanMatchingOdometryDebug_callback_time_ms
{
public:
  explicit Init_ScanMatchingOdometryDebug_callback_time_ms(::hdl_graph_slam::msg::ScanMatchingOdometryDebug & msg)
  : msg_(msg)
  {}
  Init_ScanMatchingOdometryDebug_odom_count callback_time_ms(::hdl_graph_slam::msg::ScanMatchingOdometryDebug::_callback_time_ms_type arg)
  {
    msg_.callback_time_ms = std::move(arg);
    return Init_ScanMatchingOdometryDebug_odom_count(msg_);
  }

private:
  ::hdl_graph_slam::msg::ScanMatchingOdometryDebug msg_;
};

class Init_ScanMatchingOdometryDebug_registration_time_ms
{
public:
  explicit Init_ScanMatchingOdometryDebug_registration_time_ms(::hdl_graph_slam::msg::ScanMatchingOdometryDebug & msg)
  : msg_(msg)
  {}
  Init_ScanMatchingOdometryDebug_callback_time_ms registration_time_ms(::hdl_graph_slam::msg::ScanMatchingOdometryDebug::_registration_time_ms_type arg)
  {
    msg_.registration_time_ms = std::move(arg);
    return Init_ScanMatchingOdometryDebug_callback_time_ms(msg_);
  }

private:
  ::hdl_graph_slam::msg::ScanMatchingOdometryDebug msg_;
};

class Init_ScanMatchingOdometryDebug_downsampled_point_count
{
public:
  explicit Init_ScanMatchingOdometryDebug_downsampled_point_count(::hdl_graph_slam::msg::ScanMatchingOdometryDebug & msg)
  : msg_(msg)
  {}
  Init_ScanMatchingOdometryDebug_registration_time_ms downsampled_point_count(::hdl_graph_slam::msg::ScanMatchingOdometryDebug::_downsampled_point_count_type arg)
  {
    msg_.downsampled_point_count = std::move(arg);
    return Init_ScanMatchingOdometryDebug_registration_time_ms(msg_);
  }

private:
  ::hdl_graph_slam::msg::ScanMatchingOdometryDebug msg_;
};

class Init_ScanMatchingOdometryDebug_input_point_count
{
public:
  explicit Init_ScanMatchingOdometryDebug_input_point_count(::hdl_graph_slam::msg::ScanMatchingOdometryDebug & msg)
  : msg_(msg)
  {}
  Init_ScanMatchingOdometryDebug_downsampled_point_count input_point_count(::hdl_graph_slam::msg::ScanMatchingOdometryDebug::_input_point_count_type arg)
  {
    msg_.input_point_count = std::move(arg);
    return Init_ScanMatchingOdometryDebug_downsampled_point_count(msg_);
  }

private:
  ::hdl_graph_slam::msg::ScanMatchingOdometryDebug msg_;
};

class Init_ScanMatchingOdometryDebug_header
{
public:
  Init_ScanMatchingOdometryDebug_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ScanMatchingOdometryDebug_input_point_count header(::hdl_graph_slam::msg::ScanMatchingOdometryDebug::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_ScanMatchingOdometryDebug_input_point_count(msg_);
  }

private:
  ::hdl_graph_slam::msg::ScanMatchingOdometryDebug msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::hdl_graph_slam::msg::ScanMatchingOdometryDebug>()
{
  return hdl_graph_slam::msg::builder::Init_ScanMatchingOdometryDebug_header();
}

}  // namespace hdl_graph_slam

#endif  // HDL_GRAPH_SLAM__MSG__DETAIL__SCAN_MATCHING_ODOMETRY_DEBUG__BUILDER_HPP_
