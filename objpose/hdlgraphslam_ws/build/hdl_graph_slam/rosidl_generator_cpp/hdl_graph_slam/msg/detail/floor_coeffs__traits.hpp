// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from hdl_graph_slam:msg/FloorCoeffs.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "hdl_graph_slam/msg/floor_coeffs.hpp"


#ifndef HDL_GRAPH_SLAM__MSG__DETAIL__FLOOR_COEFFS__TRAITS_HPP_
#define HDL_GRAPH_SLAM__MSG__DETAIL__FLOOR_COEFFS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "hdl_graph_slam/msg/detail/floor_coeffs__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace hdl_graph_slam
{

namespace msg
{

inline void to_flow_style_yaml(
  const FloorCoeffs & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: coeffs
  {
    if (msg.coeffs.size() == 0) {
      out << "coeffs: []";
    } else {
      out << "coeffs: [";
      size_t pending_items = msg.coeffs.size();
      for (auto item : msg.coeffs) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const FloorCoeffs & msg,
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

  // member: coeffs
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.coeffs.size() == 0) {
      out << "coeffs: []\n";
    } else {
      out << "coeffs:\n";
      for (auto item : msg.coeffs) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const FloorCoeffs & msg, bool use_flow_style = false)
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
  const hdl_graph_slam::msg::FloorCoeffs & msg,
  std::ostream & out, size_t indentation = 0)
{
  hdl_graph_slam::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use hdl_graph_slam::msg::to_yaml() instead")]]
inline std::string to_yaml(const hdl_graph_slam::msg::FloorCoeffs & msg)
{
  return hdl_graph_slam::msg::to_yaml(msg);
}

template<>
inline const char * data_type<hdl_graph_slam::msg::FloorCoeffs>()
{
  return "hdl_graph_slam::msg::FloorCoeffs";
}

template<>
inline const char * name<hdl_graph_slam::msg::FloorCoeffs>()
{
  return "hdl_graph_slam/msg/FloorCoeffs";
}

template<>
struct has_fixed_size<hdl_graph_slam::msg::FloorCoeffs>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<hdl_graph_slam::msg::FloorCoeffs>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<hdl_graph_slam::msg::FloorCoeffs>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // HDL_GRAPH_SLAM__MSG__DETAIL__FLOOR_COEFFS__TRAITS_HPP_
