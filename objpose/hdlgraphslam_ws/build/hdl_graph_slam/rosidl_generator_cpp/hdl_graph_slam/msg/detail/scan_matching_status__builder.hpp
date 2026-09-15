// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from hdl_graph_slam:msg/ScanMatchingStatus.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "hdl_graph_slam/msg/scan_matching_status.hpp"


#ifndef HDL_GRAPH_SLAM__MSG__DETAIL__SCAN_MATCHING_STATUS__BUILDER_HPP_
#define HDL_GRAPH_SLAM__MSG__DETAIL__SCAN_MATCHING_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "hdl_graph_slam/msg/detail/scan_matching_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace hdl_graph_slam
{

namespace msg
{

namespace builder
{

class Init_ScanMatchingStatus_prediction_errors
{
public:
  explicit Init_ScanMatchingStatus_prediction_errors(::hdl_graph_slam::msg::ScanMatchingStatus & msg)
  : msg_(msg)
  {}
  ::hdl_graph_slam::msg::ScanMatchingStatus prediction_errors(::hdl_graph_slam::msg::ScanMatchingStatus::_prediction_errors_type arg)
  {
    msg_.prediction_errors = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hdl_graph_slam::msg::ScanMatchingStatus msg_;
};

class Init_ScanMatchingStatus_prediction_labels
{
public:
  explicit Init_ScanMatchingStatus_prediction_labels(::hdl_graph_slam::msg::ScanMatchingStatus & msg)
  : msg_(msg)
  {}
  Init_ScanMatchingStatus_prediction_errors prediction_labels(::hdl_graph_slam::msg::ScanMatchingStatus::_prediction_labels_type arg)
  {
    msg_.prediction_labels = std::move(arg);
    return Init_ScanMatchingStatus_prediction_errors(msg_);
  }

private:
  ::hdl_graph_slam::msg::ScanMatchingStatus msg_;
};

class Init_ScanMatchingStatus_relative_pose
{
public:
  explicit Init_ScanMatchingStatus_relative_pose(::hdl_graph_slam::msg::ScanMatchingStatus & msg)
  : msg_(msg)
  {}
  Init_ScanMatchingStatus_prediction_labels relative_pose(::hdl_graph_slam::msg::ScanMatchingStatus::_relative_pose_type arg)
  {
    msg_.relative_pose = std::move(arg);
    return Init_ScanMatchingStatus_prediction_labels(msg_);
  }

private:
  ::hdl_graph_slam::msg::ScanMatchingStatus msg_;
};

class Init_ScanMatchingStatus_inlier_fraction
{
public:
  explicit Init_ScanMatchingStatus_inlier_fraction(::hdl_graph_slam::msg::ScanMatchingStatus & msg)
  : msg_(msg)
  {}
  Init_ScanMatchingStatus_relative_pose inlier_fraction(::hdl_graph_slam::msg::ScanMatchingStatus::_inlier_fraction_type arg)
  {
    msg_.inlier_fraction = std::move(arg);
    return Init_ScanMatchingStatus_relative_pose(msg_);
  }

private:
  ::hdl_graph_slam::msg::ScanMatchingStatus msg_;
};

class Init_ScanMatchingStatus_matching_error
{
public:
  explicit Init_ScanMatchingStatus_matching_error(::hdl_graph_slam::msg::ScanMatchingStatus & msg)
  : msg_(msg)
  {}
  Init_ScanMatchingStatus_inlier_fraction matching_error(::hdl_graph_slam::msg::ScanMatchingStatus::_matching_error_type arg)
  {
    msg_.matching_error = std::move(arg);
    return Init_ScanMatchingStatus_inlier_fraction(msg_);
  }

private:
  ::hdl_graph_slam::msg::ScanMatchingStatus msg_;
};

class Init_ScanMatchingStatus_has_converged
{
public:
  explicit Init_ScanMatchingStatus_has_converged(::hdl_graph_slam::msg::ScanMatchingStatus & msg)
  : msg_(msg)
  {}
  Init_ScanMatchingStatus_matching_error has_converged(::hdl_graph_slam::msg::ScanMatchingStatus::_has_converged_type arg)
  {
    msg_.has_converged = std::move(arg);
    return Init_ScanMatchingStatus_matching_error(msg_);
  }

private:
  ::hdl_graph_slam::msg::ScanMatchingStatus msg_;
};

class Init_ScanMatchingStatus_header
{
public:
  Init_ScanMatchingStatus_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ScanMatchingStatus_has_converged header(::hdl_graph_slam::msg::ScanMatchingStatus::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_ScanMatchingStatus_has_converged(msg_);
  }

private:
  ::hdl_graph_slam::msg::ScanMatchingStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::hdl_graph_slam::msg::ScanMatchingStatus>()
{
  return hdl_graph_slam::msg::builder::Init_ScanMatchingStatus_header();
}

}  // namespace hdl_graph_slam

#endif  // HDL_GRAPH_SLAM__MSG__DETAIL__SCAN_MATCHING_STATUS__BUILDER_HPP_
