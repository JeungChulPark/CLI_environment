// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from hdl_graph_slam:msg/ScanMatchingStatus.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "hdl_graph_slam/msg/scan_matching_status.hpp"


#ifndef HDL_GRAPH_SLAM__MSG__DETAIL__SCAN_MATCHING_STATUS__STRUCT_HPP_
#define HDL_GRAPH_SLAM__MSG__DETAIL__SCAN_MATCHING_STATUS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"
// Member 'relative_pose'
// Member 'prediction_errors'
#include "geometry_msgs/msg/detail/pose__struct.hpp"
// Member 'prediction_labels'
#include "std_msgs/msg/detail/string__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__hdl_graph_slam__msg__ScanMatchingStatus __attribute__((deprecated))
#else
# define DEPRECATED__hdl_graph_slam__msg__ScanMatchingStatus __declspec(deprecated)
#endif

namespace hdl_graph_slam
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ScanMatchingStatus_
{
  using Type = ScanMatchingStatus_<ContainerAllocator>;

  explicit ScanMatchingStatus_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    relative_pose(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->has_converged = false;
      this->matching_error = 0.0f;
      this->inlier_fraction = 0.0f;
    }
  }

  explicit ScanMatchingStatus_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    relative_pose(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->has_converged = false;
      this->matching_error = 0.0f;
      this->inlier_fraction = 0.0f;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _has_converged_type =
    bool;
  _has_converged_type has_converged;
  using _matching_error_type =
    float;
  _matching_error_type matching_error;
  using _inlier_fraction_type =
    float;
  _inlier_fraction_type inlier_fraction;
  using _relative_pose_type =
    geometry_msgs::msg::Pose_<ContainerAllocator>;
  _relative_pose_type relative_pose;
  using _prediction_labels_type =
    std::vector<std_msgs::msg::String_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std_msgs::msg::String_<ContainerAllocator>>>;
  _prediction_labels_type prediction_labels;
  using _prediction_errors_type =
    std::vector<geometry_msgs::msg::Pose_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::Pose_<ContainerAllocator>>>;
  _prediction_errors_type prediction_errors;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__has_converged(
    const bool & _arg)
  {
    this->has_converged = _arg;
    return *this;
  }
  Type & set__matching_error(
    const float & _arg)
  {
    this->matching_error = _arg;
    return *this;
  }
  Type & set__inlier_fraction(
    const float & _arg)
  {
    this->inlier_fraction = _arg;
    return *this;
  }
  Type & set__relative_pose(
    const geometry_msgs::msg::Pose_<ContainerAllocator> & _arg)
  {
    this->relative_pose = _arg;
    return *this;
  }
  Type & set__prediction_labels(
    const std::vector<std_msgs::msg::String_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std_msgs::msg::String_<ContainerAllocator>>> & _arg)
  {
    this->prediction_labels = _arg;
    return *this;
  }
  Type & set__prediction_errors(
    const std::vector<geometry_msgs::msg::Pose_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::Pose_<ContainerAllocator>>> & _arg)
  {
    this->prediction_errors = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    hdl_graph_slam::msg::ScanMatchingStatus_<ContainerAllocator> *;
  using ConstRawPtr =
    const hdl_graph_slam::msg::ScanMatchingStatus_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<hdl_graph_slam::msg::ScanMatchingStatus_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<hdl_graph_slam::msg::ScanMatchingStatus_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      hdl_graph_slam::msg::ScanMatchingStatus_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<hdl_graph_slam::msg::ScanMatchingStatus_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      hdl_graph_slam::msg::ScanMatchingStatus_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<hdl_graph_slam::msg::ScanMatchingStatus_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<hdl_graph_slam::msg::ScanMatchingStatus_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<hdl_graph_slam::msg::ScanMatchingStatus_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__hdl_graph_slam__msg__ScanMatchingStatus
    std::shared_ptr<hdl_graph_slam::msg::ScanMatchingStatus_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__hdl_graph_slam__msg__ScanMatchingStatus
    std::shared_ptr<hdl_graph_slam::msg::ScanMatchingStatus_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ScanMatchingStatus_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->has_converged != other.has_converged) {
      return false;
    }
    if (this->matching_error != other.matching_error) {
      return false;
    }
    if (this->inlier_fraction != other.inlier_fraction) {
      return false;
    }
    if (this->relative_pose != other.relative_pose) {
      return false;
    }
    if (this->prediction_labels != other.prediction_labels) {
      return false;
    }
    if (this->prediction_errors != other.prediction_errors) {
      return false;
    }
    return true;
  }
  bool operator!=(const ScanMatchingStatus_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ScanMatchingStatus_

// alias to use template instance with default allocator
using ScanMatchingStatus =
  hdl_graph_slam::msg::ScanMatchingStatus_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace hdl_graph_slam

#endif  // HDL_GRAPH_SLAM__MSG__DETAIL__SCAN_MATCHING_STATUS__STRUCT_HPP_
