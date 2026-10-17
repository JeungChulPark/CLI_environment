// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from hdl_graph_slam:msg/PrefilteringDebug.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "hdl_graph_slam/msg/prefiltering_debug.hpp"


#ifndef HDL_GRAPH_SLAM__MSG__DETAIL__PREFILTERING_DEBUG__TRAITS_HPP_
#define HDL_GRAPH_SLAM__MSG__DETAIL__PREFILTERING_DEBUG__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "hdl_graph_slam/msg/detail/prefiltering_debug__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace hdl_graph_slam
{

namespace msg
{

inline void to_flow_style_yaml(
  const PrefilteringDebug & msg,
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

  // member: transformed_point_count
  {
    out << "transformed_point_count: ";
    rosidl_generator_traits::value_to_yaml(msg.transformed_point_count, out);
    out << ", ";
  }

  // member: distance_filtered_point_count
  {
    out << "distance_filtered_point_count: ";
    rosidl_generator_traits::value_to_yaml(msg.distance_filtered_point_count, out);
    out << ", ";
  }

  // member: downsampled_point_count
  {
    out << "downsampled_point_count: ";
    rosidl_generator_traits::value_to_yaml(msg.downsampled_point_count, out);
    out << ", ";
  }

  // member: output_point_count
  {
    out << "output_point_count: ";
    rosidl_generator_traits::value_to_yaml(msg.output_point_count, out);
    out << ", ";
  }

  // member: callback_time_ms
  {
    out << "callback_time_ms: ";
    rosidl_generator_traits::value_to_yaml(msg.callback_time_ms, out);
    out << ", ";
  }

  // member: output_count
  {
    out << "output_count: ";
    rosidl_generator_traits::value_to_yaml(msg.output_count, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const PrefilteringDebug & msg,
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

  // member: transformed_point_count
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "transformed_point_count: ";
    rosidl_generator_traits::value_to_yaml(msg.transformed_point_count, out);
    out << "\n";
  }

  // member: distance_filtered_point_count
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "distance_filtered_point_count: ";
    rosidl_generator_traits::value_to_yaml(msg.distance_filtered_point_count, out);
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

  // member: output_point_count
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "output_point_count: ";
    rosidl_generator_traits::value_to_yaml(msg.output_point_count, out);
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

  // member: output_count
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "output_count: ";
    rosidl_generator_traits::value_to_yaml(msg.output_count, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const PrefilteringDebug & msg, bool use_flow_style = false)
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
  const hdl_graph_slam::msg::PrefilteringDebug & msg,
  std::ostream & out, size_t indentation = 0)
{
  hdl_graph_slam::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use hdl_graph_slam::msg::to_yaml() instead")]]
inline std::string to_yaml(const hdl_graph_slam::msg::PrefilteringDebug & msg)
{
  return hdl_graph_slam::msg::to_yaml(msg);
}

template<>
inline const char * data_type<hdl_graph_slam::msg::PrefilteringDebug>()
{
  return "hdl_graph_slam::msg::PrefilteringDebug";
}

template<>
inline const char * name<hdl_graph_slam::msg::PrefilteringDebug>()
{
  return "hdl_graph_slam/msg/PrefilteringDebug";
}

template<>
struct has_fixed_size<hdl_graph_slam::msg::PrefilteringDebug>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<hdl_graph_slam::msg::PrefilteringDebug>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<hdl_graph_slam::msg::PrefilteringDebug>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // HDL_GRAPH_SLAM__MSG__DETAIL__PREFILTERING_DEBUG__TRAITS_HPP_
