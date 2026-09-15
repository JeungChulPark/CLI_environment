// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from hdl_graph_slam:msg/FloorCoeffs.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "hdl_graph_slam/msg/floor_coeffs.hpp"


#ifndef HDL_GRAPH_SLAM__MSG__DETAIL__FLOOR_COEFFS__BUILDER_HPP_
#define HDL_GRAPH_SLAM__MSG__DETAIL__FLOOR_COEFFS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "hdl_graph_slam/msg/detail/floor_coeffs__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace hdl_graph_slam
{

namespace msg
{

namespace builder
{

class Init_FloorCoeffs_coeffs
{
public:
  explicit Init_FloorCoeffs_coeffs(::hdl_graph_slam::msg::FloorCoeffs & msg)
  : msg_(msg)
  {}
  ::hdl_graph_slam::msg::FloorCoeffs coeffs(::hdl_graph_slam::msg::FloorCoeffs::_coeffs_type arg)
  {
    msg_.coeffs = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hdl_graph_slam::msg::FloorCoeffs msg_;
};

class Init_FloorCoeffs_header
{
public:
  Init_FloorCoeffs_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FloorCoeffs_coeffs header(::hdl_graph_slam::msg::FloorCoeffs::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_FloorCoeffs_coeffs(msg_);
  }

private:
  ::hdl_graph_slam::msg::FloorCoeffs msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::hdl_graph_slam::msg::FloorCoeffs>()
{
  return hdl_graph_slam::msg::builder::Init_FloorCoeffs_header();
}

}  // namespace hdl_graph_slam

#endif  // HDL_GRAPH_SLAM__MSG__DETAIL__FLOOR_COEFFS__BUILDER_HPP_
