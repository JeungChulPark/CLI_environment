// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from hdl_graph_slam:msg/ScanMatchingOdometryDebug.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "hdl_graph_slam/msg/scan_matching_odometry_debug.hpp"


#ifndef HDL_GRAPH_SLAM__MSG__DETAIL__SCAN_MATCHING_ODOMETRY_DEBUG__TRAITS_HPP_
#define HDL_GRAPH_SLAM__MSG__DETAIL__SCAN_MATCHING_ODOMETRY_DEBUG__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "hdl_graph_slam/msg/detail/scan_matching_odometry_debug__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace hdl_graph_slam
{

namespace msg
{

inline void to_flow_style_yaml(
  const ScanMatchingOdometryDebug & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: input_point_count
  {
    out << "input_point_count: ";
    rosidl_generator_traits::value_to_yaml(msg.input_point_count, out);
    out << ", ";
  }

  // member: downsampled_point_count
  {
    out << "downsampled_point_count: ";
    rosidl_generator_traits::value_to_yaml(msg.downsampled_point_count, out);
    out << ", ";
  }

  // member: registration_time_ms
  {
    out << "registration_time_ms: ";
    rosidl_generator_traits::value_to_yaml(msg.registration_time_ms, out);
    out << ", ";
  }

  // member: callback_time_ms
  {
    out << "callback_time_ms: ";
    rosidl_generator_traits::value_to_yaml(msg.callback_time_ms, out);
    out << ", ";
  }

  // member: odom_count
  {
    out << "odom_count: ";
    rosidl_generator_traits::value_to_yaml(msg.odom_count, out);
    out << ", ";
  }

  // member: registration_triggered
  {
    out << "registration_triggered: ";
    rosidl_generator_traits::value_to_yaml(msg.registration_triggered, out);
    out << ", ";
  }

  // member: has_converged
  {
    out << "has_converged: ";
    rosidl_generator_traits::value_to_yaml(msg.has_converged, out);
    out << ", ";
  }

  // member: matching_error
  {
    out << "matching_error: ";
    rosidl_generator_traits::value_to_yaml(msg.matching_error, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ScanMatchingOdometryDebug & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: input_point_count
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "input_point_count: ";
    rosidl_generator_traits::value_to_yaml(msg.input_point_count, out);
    out << "\n";
  }

  // member: downsampled_point_count
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "downsampled_point_count: ";
    rosidl_generator_traits::value_to_yaml(msg.downsampled_point_count, out);
    out << "\n";
  }

  // member: registration_time_ms
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "registration_time_ms: ";
    rosidl_generator_traits::value_to_yaml(msg.registration_time_ms, out);
    out << "\n";
  }

  // member: callback_time_ms
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "callback_time_ms: ";
    rosidl_generator_traits::value_to_yaml(msg.callback_time_ms, out);
    out << "\n";
  }

  // member: odom_count
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "odom_count: ";
    rosidl_generator_traits::value_to_yaml(msg.odom_count, out);
    out << "\n";
  }

  // member: registration_triggered
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "registration_triggered: ";
    rosidl_generator_traits::value_to_yaml(msg.registration_triggered, out);
    out << "\n";
  }

  // member: has_converged
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "has_converged: ";
    rosidl_generator_traits::value_to_yaml(msg.has_converged, out);
    out << "\n";
  }

  // member: matching_error
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "matching_error: ";
    rosidl_generator_traits::value_to_yaml(msg.matching_error, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ScanMatchingOdometryDebug & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace hdl_graph_slam

namespace rosidl_generator_traits
{

[[deprecated("use hdl_graph_slam::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const hdl_graph_slam::msg::ScanMatchingOdometryDebug & msg,
  std::ostream & out, size_t indentation = 0)
{
  hdl_graph_slam::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use hdl_graph_slam::msg::to_yaml() instead")]]
inline std::string to_yaml(const hdl_graph_slam::msg::ScanMatchingOdometryDebug & msg)
{
  return hdl_graph_slam::msg::to_yaml(msg);
}

template<>
inline const char * data_type<hdl_graph_slam::msg::ScanMatchingOdometryDebug>()
{
  return "hdl_graph_slam::msg::ScanMatchingOdometryDebug";
}

template<>
inline const char * name<hdl_graph_slam::msg::ScanMatchingOdometryDebug>()
{
  return "hdl_graph_slam/msg/ScanMatchingOdometryDebug";
}

template<>
struct has_fixed_size<hdl_graph_slam::msg::ScanMatchingOdometryDebug>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<hdl_graph_slam::msg::ScanMatchingOdometryDebug>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<hdl_graph_slam::msg::ScanMatchingOdometryDebug>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // HDL_GRAPH_SLAM__MSG__DETAIL__SCAN_MATCHING_ODOMETRY_DEBUG__TRAITS_HPP_
