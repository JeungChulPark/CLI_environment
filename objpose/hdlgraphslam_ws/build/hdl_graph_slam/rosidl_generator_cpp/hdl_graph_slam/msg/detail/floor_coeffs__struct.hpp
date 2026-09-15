// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from hdl_graph_slam:msg/FloorCoeffs.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "hdl_graph_slam/msg/floor_coeffs.hpp"


#ifndef HDL_GRAPH_SLAM__MSG__DETAIL__FLOOR_COEFFS__STRUCT_HPP_
#define HDL_GRAPH_SLAM__MSG__DETAIL__FLOOR_COEFFS__STRUCT_HPP_

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
# define DEPRECATED__hdl_graph_slam__msg__FloorCoeffs __attribute__((deprecated))
#else
# define DEPRECATED__hdl_graph_slam__msg__FloorCoeffs __declspec(deprecated)
#endif

namespace hdl_graph_slam
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct FloorCoeffs_
{
  using Type = FloorCoeffs_<ContainerAllocator>;

  explicit FloorCoeffs_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    (void)_init;
  }

  explicit FloorCoeffs_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _coeffs_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _coeffs_type coeffs;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__coeffs(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->coeffs = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    hdl_graph_slam::msg::FloorCoeffs_<ContainerAllocator> *;
  using ConstRawPtr =
    const hdl_graph_slam::msg::FloorCoeffs_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<hdl_graph_slam::msg::FloorCoeffs_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<hdl_graph_slam::msg::FloorCoeffs_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      hdl_graph_slam::msg::FloorCoeffs_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<hdl_graph_slam::msg::FloorCoeffs_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      hdl_graph_slam::msg::FloorCoeffs_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<hdl_graph_slam::msg::FloorCoeffs_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<hdl_graph_slam::msg::FloorCoeffs_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<hdl_graph_slam::msg::FloorCoeffs_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__hdl_graph_slam__msg__FloorCoeffs
    std::shared_ptr<hdl_graph_slam::msg::FloorCoeffs_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__hdl_graph_slam__msg__FloorCoeffs
    std::shared_ptr<hdl_graph_slam::msg::FloorCoeffs_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const FloorCoeffs_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->coeffs != other.coeffs) {
      return false;
    }
    return true;
  }
  bool operator!=(const FloorCoeffs_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct FloorCoeffs_

// alias to use template instance with default allocator
using FloorCoeffs =
  hdl_graph_slam::msg::FloorCoeffs_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace hdl_graph_slam

#endif  // HDL_GRAPH_SLAM__MSG__DETAIL__FLOOR_COEFFS__STRUCT_HPP_
