// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from hdl_graph_slam:msg/ScanMatchingOdometryDebug.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "hdl_graph_slam/msg/scan_matching_odometry_debug.hpp"


#ifndef HDL_GRAPH_SLAM__MSG__DETAIL__SCAN_MATCHING_ODOMETRY_DEBUG__STRUCT_HPP_
#define HDL_GRAPH_SLAM__MSG__DETAIL__SCAN_MATCHING_ODOMETRY_DEBUG__STRUCT_HPP_

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

#ifndef _WIN32
# define DEPRECATED__hdl_graph_slam__msg__ScanMatchingOdometryDebug __attribute__((deprecated))
#else
# define DEPRECATED__hdl_graph_slam__msg__ScanMatchingOdometryDebug __declspec(deprecated)
#endif

namespace hdl_graph_slam
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ScanMatchingOdometryDebug_
{
  using Type = ScanMatchingOdometryDebug_<ContainerAllocator>;

  explicit ScanMatchingOdometryDebug_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->input_point_count = 0ul;
      this->downsampled_point_count = 0ul;
      this->registration_time_ms = 0.0f;
      this->callback_time_ms = 0.0f;
      this->odom_count = 0ull;
      this->registration_triggered = false;
      this->has_converged = false;
      this->matching_error = 0.0f;
    }
  }

  explicit ScanMatchingOdometryDebug_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->input_point_count = 0ul;
      this->downsampled_point_count = 0ul;
      this->registration_time_ms = 0.0f;
      this->callback_time_ms = 0.0f;
      this->odom_count = 0ull;
      this->registration_triggered = false;
      this->has_converged = false;
      this->matching_error = 0.0f;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _input_point_count_type =
    uint32_t;
  _input_point_count_type input_point_count;
  using _downsampled_point_count_type =
    uint32_t;
  _downsampled_point_count_type downsampled_point_count;
  using _registration_time_ms_type =
    float;
  _registration_time_ms_type registration_time_ms;
  using _callback_time_ms_type =
    float;
  _callback_time_ms_type callback_time_ms;
  using _odom_count_type =
    uint64_t;
  _odom_count_type odom_count;
  using _registration_triggered_type =
    bool;
  _registration_triggered_type registration_triggered;
  using _has_converged_type =
    bool;
  _has_converged_type has_converged;
  using _matching_error_type =
    float;
  _matching_error_type matching_error;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__input_point_count(
    const uint32_t & _arg)
  {
    this->input_point_count = _arg;
    return *this;
  }
  Type & set__downsampled_point_count(
    const uint32_t & _arg)
  {
    this->downsampled_point_count = _arg;
    return *this;
  }
  Type & set__registration_time_ms(
    const float & _arg)
  {
    this->registration_time_ms = _arg;
    return *this;
  }
  Type & set__callback_time_ms(
    const float & _arg)
  {
    this->callback_time_ms = _arg;
    return *this;
  }
  Type & set__odom_count(
    const uint64_t & _arg)
  {
    this->odom_count = _arg;
    return *this;
  }
  Type & set__registration_triggered(
    const bool & _arg)
  {
    this->registration_triggered = _arg;
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

  // constant declarations

  // pointer types
  using RawPtr =
    hdl_graph_slam::msg::ScanMatchingOdometryDebug_<ContainerAllocator> *;
  using ConstRawPtr =
    const hdl_graph_slam::msg::ScanMatchingOdometryDebug_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<hdl_graph_slam::msg::ScanMatchingOdometryDebug_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<hdl_graph_slam::msg::ScanMatchingOdometryDebug_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      hdl_graph_slam::msg::ScanMatchingOdometryDebug_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<hdl_graph_slam::msg::ScanMatchingOdometryDebug_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      hdl_graph_slam::msg::ScanMatchingOdometryDebug_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<hdl_graph_slam::msg::ScanMatchingOdometryDebug_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<hdl_graph_slam::msg::ScanMatchingOdometryDebug_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<hdl_graph_slam::msg::ScanMatchingOdometryDebug_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__hdl_graph_slam__msg__ScanMatchingOdometryDebug
    std::shared_ptr<hdl_graph_slam::msg::ScanMatchingOdometryDebug_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__hdl_graph_slam__msg__ScanMatchingOdometryDebug
    std::shared_ptr<hdl_graph_slam::msg::ScanMatchingOdometryDebug_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ScanMatchingOdometryDebug_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->input_point_count != other.input_point_count) {
      return false;
    }
    if (this->downsampled_point_count != other.downsampled_point_count) {
      return false;
    }
    if (this->registration_time_ms != other.registration_time_ms) {
      return false;
    }
    if (this->callback_time_ms != other.callback_time_ms) {
      return false;
    }
    if (this->odom_count != other.odom_count) {
      return false;
    }
    if (this->registration_triggered != other.registration_triggered) {
      return false;
    }
    if (this->has_converged != other.has_converged) {
      return false;
    }
    if (this->matching_error != other.matching_error) {
      return false;
    }
    return true;
  }
  bool operator!=(const ScanMatchingOdometryDebug_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ScanMatchingOdometryDebug_

// alias to use template instance with default allocator
using ScanMatchingOdometryDebug =
  hdl_graph_slam::msg::ScanMatchingOdometryDebug_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace hdl_graph_slam

#endif  // HDL_GRAPH_SLAM__MSG__DETAIL__SCAN_MATCHING_ODOMETRY_DEBUG__STRUCT_HPP_
