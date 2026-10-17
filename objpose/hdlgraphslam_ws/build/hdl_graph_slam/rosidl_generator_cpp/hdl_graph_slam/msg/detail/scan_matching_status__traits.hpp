// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from hdl_graph_slam:msg/ScanMatchingStatus.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "hdl_graph_slam/msg/scan_matching_status.hpp"


#ifndef HDL_GRAPH_SLAM__MSG__DETAIL__SCAN_MATCHING_STATUS__TRAITS_HPP_
#define HDL_GRAPH_SLAM__MSG__DETAIL__SCAN_MATCHING_STATUS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "hdl_graph_slam/msg/detail/scan_matching_status__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'relative_pose'
// Member 'prediction_errors'
#include "geometry_msgs/msg/detail/pose__traits.hpp"
// Member 'prediction_labels'
#include "std_msgs/msg/detail/string__traits.hpp"

namespace hdl_graph_slam
{

namespace msg
{

inline void to_flow_style_yaml(
  const ScanMatchingStatus & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
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
    out << ", ";
  }

  // member: inlier_fraction
  {
    out << "inlier_fraction: ";
    rosidl_generator_traits::value_to_yaml(msg.inlier_fraction, out);
    out << ", ";
  }

  // member: relative_pose
  {
    out << "relative_pose: ";
    to_flow_style_yaml(msg.relative_pose, out);
    out << ", ";
  }

  // member: prediction_labels
  {
    if (msg.prediction_labels.size() == 0) {
      out << "prediction_labels: []";
    } else {
      out << "prediction_labels: [";
      size_t pending_items = msg.prediction_labels.size();
      for (auto item : msg.prediction_labels) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: prediction_errors
  {
    if (msg.prediction_errors.size() == 0) {
      out << "prediction_errors: []";
    } else {
      out << "prediction_errors: [";
      size_t pending_items = msg.prediction_errors.size();
      for (auto item : msg.prediction_errors) {
        to_flow_style_yaml(item, out);
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
  const ScanMatchingStatus & msg,
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

  // member: inlier_fraction
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "inlier_fraction: ";
    rosidl_generator_traits::value_to_yaml(msg.inlier_fraction, out);
    out << "\n";
  }

  // member: relative_pose
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "relative_pose:\n";
    to_block_style_yaml(msg.relative_pose, out, indentation + 2);
  }

  // member: prediction_labels
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.prediction_labels.size() == 0) {
      out << "prediction_labels: []\n";
    } else {
      out << "prediction_labels:\n";
      for (auto item : msg.prediction_labels) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: prediction_errors
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.prediction_errors.size() == 0) {
      out << "prediction_errors: []\n";
    } else {
      out << "prediction_errors:\n";
      for (auto item : msg.prediction_errors) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ScanMatchingStatus & msg, bool use_flow_style = false)
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
  const hdl_graph_slam::msg::ScanMatchingStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  hdl_graph_slam::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use hdl_graph_slam::msg::to_yaml() instead")]]
inline std::string to_yaml(const hdl_graph_slam::msg::ScanMatchingStatus & msg)
{
  return hdl_graph_slam::msg::to_yaml(msg);
}

template<>
inline const char * data_type<hdl_graph_slam::msg::ScanMatchingStatus>()
{
  return "hdl_graph_slam::msg::ScanMatchingStatus";
}

template<>
inline const char * name<hdl_graph_slam::msg::ScanMatchingStatus>()
{
  return "hdl_graph_slam/msg/ScanMatchingStatus";
}

template<>
struct has_fixed_size<hdl_graph_slam::msg::ScanMatchingStatus>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<hdl_graph_slam::msg::ScanMatchingStatus>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<hdl_graph_slam::msg::ScanMatchingStatus>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // HDL_GRAPH_SLAM__MSG__DETAIL__SCAN_MATCHING_STATUS__TRAITS_HPP_
