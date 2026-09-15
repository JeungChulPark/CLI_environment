// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from hdl_graph_slam:msg/PrefilteringDebug.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "hdl_graph_slam/msg/prefiltering_debug.hpp"


#ifndef HDL_GRAPH_SLAM__MSG__DETAIL__PREFILTERING_DEBUG__STRUCT_HPP_
#define HDL_GRAPH_SLAM__MSG__DETAIL__PREFILTERING_DEBUG__STRUCT_HPP_

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
# define DEPRECATED__hdl_graph_slam__msg__PrefilteringDebug __attribute__((deprecated))
#else
# define DEPRECATED__hdl_graph_slam__msg__PrefilteringDebug __declspec(deprecated)
#endif

namespace hdl_graph_slam
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct PrefilteringDebug_
{
  using Type = PrefilteringDebug_<ContainerAllocator>;

  explicit PrefilteringDebug_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->input_point_count = 0ul;
      this->transformed_point_count = 0ul;
      this->distance_filtered_point_count = 0ul;
      this->downsampled_point_count = 0ul;
      this->output_point_count = 0ul;
      this->callback_time_ms = 0.0f;
      this->output_count = 0ull;
    }
  }

  explicit PrefilteringDebug_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->input_point_count = 0ul;
      this->transformed_point_count = 0ul;
      this->distance_filtered_point_count = 0ul;
      this->downsampled_point_count = 0ul;
      this->output_point_count = 0ul;
      this->callback_time_ms = 0.0f;
      this->output_count = 0ull;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _input_point_count_type =
    uint32_t;
  _input_point_count_type input_point_count;
  using _transformed_point_count_type =
    uint32_t;
  _transformed_point_count_type transformed_point_count;
  using _distance_filtered_point_count_type =
    uint32_t;
  _distance_filtered_point_count_type distance_filtered_point_count;
  using _downsampled_point_count_type =
    uint32_t;
  _downsampled_point_count_type downsampled_point_count;
  using _output_point_count_type =
    uint32_t;
  _output_point_count_type output_point_count;
  using _callback_time_ms_type =
    float;
  _callback_time_ms_type callback_time_ms;
  using _output_count_type =
    uint64_t;
  _output_count_type output_count;

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
  Type & set__transformed_point_count(
    const uint32_t & _arg)
  {
    this->transformed_point_count = _arg;
    return *this;
  }
  Type & set__distance_filtered_point_count(
    const uint32_t & _arg)
  {
    this->distance_filtered_point_count = _arg;
    return *this;
  }
  Type & set__downsampled_point_count(
    const uint32_t & _arg)
  {
    this->downsampled_point_count = _arg;
    return *this;
  }
  Type & set__output_point_count(
    const uint32_t & _arg)
  {
    this->output_point_count = _arg;
    return *this;
  }
  Type & set__callback_time_ms(
    const float & _arg)
  {
    this->callback_time_ms = _arg;
    return *this;
  }
  Type & set__output_count(
    const uint64_t & _arg)
  {
    this->output_count = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    hdl_graph_slam::msg::PrefilteringDebug_<ContainerAllocator> *;
  using ConstRawPtr =
    const hdl_graph_slam::msg::PrefilteringDebug_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<hdl_graph_slam::msg::PrefilteringDebug_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<hdl_graph_slam::msg::PrefilteringDebug_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      hdl_graph_slam::msg::PrefilteringDebug_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<hdl_graph_slam::msg::PrefilteringDebug_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      hdl_graph_slam::msg::PrefilteringDebug_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<hdl_graph_slam::msg::PrefilteringDebug_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<hdl_graph_slam::msg::PrefilteringDebug_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<hdl_graph_slam::msg::PrefilteringDebug_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__hdl_graph_slam__msg__PrefilteringDebug
    std::shared_ptr<hdl_graph_slam::msg::PrefilteringDebug_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__hdl_graph_slam__msg__PrefilteringDebug
    std::shared_ptr<hdl_graph_slam::msg::PrefilteringDebug_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PrefilteringDebug_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->input_point_count != other.input_point_count) {
      return false;
    }
    if (this->transformed_point_count != other.transformed_point_count) {
      return false;
    }
    if (this->distance_filtered_point_count != other.distance_filtered_point_count) {
      return false;
    }
    if (this->downsampled_point_count != other.downsampled_point_count) {
      return false;
    }
    if (this->output_point_count != other.output_point_count) {
      return false;
    }
    if (this->callback_time_ms != other.callback_time_ms) {
      return false;
    }
    if (this->output_count != other.output_count) {
      return false;
    }
    return true;
  }
  bool operator!=(const PrefilteringDebug_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PrefilteringDebug_

// alias to use template instance with default allocator
using PrefilteringDebug =
  hdl_graph_slam::msg::PrefilteringDebug_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace hdl_graph_slam

#endif  // HDL_GRAPH_SLAM__MSG__DETAIL__PREFILTERING_DEBUG__STRUCT_HPP_
