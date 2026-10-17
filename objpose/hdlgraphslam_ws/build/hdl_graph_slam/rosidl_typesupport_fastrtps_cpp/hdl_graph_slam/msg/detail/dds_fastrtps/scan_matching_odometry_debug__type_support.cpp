// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from hdl_graph_slam:msg/ScanMatchingOdometryDebug.idl
// generated code does not contain a copyright notice
#include "hdl_graph_slam/msg/detail/scan_matching_odometry_debug__rosidl_typesupport_fastrtps_cpp.hpp"
#include "hdl_graph_slam/msg/detail/scan_matching_odometry_debug__functions.h"
#include "hdl_graph_slam/msg/detail/scan_matching_odometry_debug__struct.hpp"

#include <cstddef>
#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/serialization_helpers.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions
namespace std_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const std_msgs::msg::Header &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  std_msgs::msg::Header &);
size_t get_serialized_size(
  const std_msgs::msg::Header &,
  size_t current_alignment);
size_t
max_serialized_size_Header(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
bool cdr_serialize_key(
  const std_msgs::msg::Header &,
  eprosima::fastcdr::Cdr &);
size_t get_serialized_size_key(
  const std_msgs::msg::Header &,
  size_t current_alignment);
size_t
max_serialized_size_key_Header(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace std_msgs


namespace hdl_graph_slam
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{


bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_hdl_graph_slam
cdr_serialize(
  const hdl_graph_slam::msg::ScanMatchingOdometryDebug & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: header
  std_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.header,
    cdr);

  // Member: input_point_count
  cdr << ros_message.input_point_count;

  // Member: downsampled_point_count
  cdr << ros_message.downsampled_point_count;

  // Member: registration_time_ms
  cdr << ros_message.registration_time_ms;

  // Member: callback_time_ms
  cdr << ros_message.callback_time_ms;

  // Member: odom_count
  cdr << ros_message.odom_count;

  // Member: registration_triggered
  cdr << (ros_message.registration_triggered ? true : false);

  // Member: has_converged
  cdr << (ros_message.has_converged ? true : false);

  // Member: matching_error
  cdr << ros_message.matching_error;

  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_hdl_graph_slam
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  hdl_graph_slam::msg::ScanMatchingOdometryDebug & ros_message)
{
  // Member: header
  std_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.header);

  // Member: input_point_count
  cdr >> ros_message.input_point_count;

  // Member: downsampled_point_count
  cdr >> ros_message.downsampled_point_count;

  // Member: registration_time_ms
  cdr >> ros_message.registration_time_ms;

  // Member: callback_time_ms
  cdr >> ros_message.callback_time_ms;

  // Member: odom_count
  cdr >> ros_message.odom_count;

  // Member: registration_triggered
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.registration_triggered = tmp ? true : false;
  }

  // Member: has_converged
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.has_converged = tmp ? true : false;
  }

  // Member: matching_error
  cdr >> ros_message.matching_error;

  return true;
}  // NOLINT(readability/fn_size)


size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_hdl_graph_slam
get_serialized_size(
  const hdl_graph_slam::msg::ScanMatchingOdometryDebug & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: header
  current_alignment +=
    std_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.header, current_alignment);

  // Member: input_point_count
  {
    size_t item_size = sizeof(ros_message.input_point_count);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: downsampled_point_count
  {
    size_t item_size = sizeof(ros_message.downsampled_point_count);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: registration_time_ms
  {
    size_t item_size = sizeof(ros_message.registration_time_ms);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: callback_time_ms
  {
    size_t item_size = sizeof(ros_message.callback_time_ms);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: odom_count
  {
    size_t item_size = sizeof(ros_message.odom_count);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: registration_triggered
  {
    size_t item_size = sizeof(ros_message.registration_triggered);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: has_converged
  {
    size_t item_size = sizeof(ros_message.has_converged);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: matching_error
  {
    size_t item_size = sizeof(ros_message.matching_error);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}


size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_hdl_graph_slam
max_serialized_size_ScanMatchingOdometryDebug(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // Member: header
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        std_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_Header(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }
  // Member: input_point_count
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // Member: downsampled_point_count
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // Member: registration_time_ms
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // Member: callback_time_ms
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // Member: odom_count
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // Member: registration_triggered
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // Member: has_converged
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // Member: matching_error
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = hdl_graph_slam::msg::ScanMatchingOdometryDebug;
    is_plain =
      (
      offsetof(DataType, matching_error) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_hdl_graph_slam
cdr_serialize_key(
  const hdl_graph_slam::msg::ScanMatchingOdometryDebug & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: header
  std_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize_key(
    ros_message.header,
    cdr);

  // Member: input_point_count
  cdr << ros_message.input_point_count;

  // Member: downsampled_point_count
  cdr << ros_message.downsampled_point_count;

  // Member: registration_time_ms
  cdr << ros_message.registration_time_ms;

  // Member: callback_time_ms
  cdr << ros_message.callback_time_ms;

  // Member: odom_count
  cdr << ros_message.odom_count;

  // Member: registration_triggered
  cdr << (ros_message.registration_triggered ? true : false);

  // Member: has_converged
  cdr << (ros_message.has_converged ? true : false);

  // Member: matching_error
  cdr << ros_message.matching_error;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_hdl_graph_slam
get_serialized_size_key(
  const hdl_graph_slam::msg::ScanMatchingOdometryDebug & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: header
  current_alignment +=
    std_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size_key(
    ros_message.header, current_alignment);

  // Member: input_point_count
  {
    size_t item_size = sizeof(ros_message.input_point_count);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: downsampled_point_count
  {
    size_t item_size = sizeof(ros_message.downsampled_point_count);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: registration_time_ms
  {
    size_t item_size = sizeof(ros_message.registration_time_ms);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: callback_time_ms
  {
    size_t item_size = sizeof(ros_message.callback_time_ms);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: odom_count
  {
    size_t item_size = sizeof(ros_message.odom_count);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: registration_triggered
  {
    size_t item_size = sizeof(ros_message.registration_triggered);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: has_converged
  {
    size_t item_size = sizeof(ros_message.has_converged);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: matching_error
  {
    size_t item_size = sizeof(ros_message.matching_error);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_hdl_graph_slam
max_serialized_size_key_ScanMatchingOdometryDebug(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // Member: header
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        std_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_key_Header(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Member: input_point_count
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: downsampled_point_count
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: registration_time_ms
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: callback_time_ms
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: odom_count
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: registration_triggered
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: has_converged
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: matching_error
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = hdl_graph_slam::msg::ScanMatchingOdometryDebug;
    is_plain =
      (
      offsetof(DataType, matching_error) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}


static bool _ScanMatchingOdometryDebug__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const hdl_graph_slam::msg::ScanMatchingOdometryDebug *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _ScanMatchingOdometryDebug__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<hdl_graph_slam::msg::ScanMatchingOdometryDebug *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _ScanMatchingOdometryDebug__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const hdl_graph_slam::msg::ScanMatchingOdometryDebug *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _ScanMatchingOdometryDebug__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_ScanMatchingOdometryDebug(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _ScanMatchingOdometryDebug__callbacks = {
  "hdl_graph_slam::msg",
  "ScanMatchingOdometryDebug",
  _ScanMatchingOdometryDebug__cdr_serialize,
  _ScanMatchingOdometryDebug__cdr_deserialize,
  _ScanMatchingOdometryDebug__get_serialized_size,
  _ScanMatchingOdometryDebug__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _ScanMatchingOdometryDebug__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_ScanMatchingOdometryDebug__callbacks,
  get_message_typesupport_handle_function,
  &hdl_graph_slam__msg__ScanMatchingOdometryDebug__get_type_hash,
  &hdl_graph_slam__msg__ScanMatchingOdometryDebug__get_type_description,
  &hdl_graph_slam__msg__ScanMatchingOdometryDebug__get_type_description_sources,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace hdl_graph_slam

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_hdl_graph_slam
const rosidl_message_type_support_t *
get_message_type_support_handle<hdl_graph_slam::msg::ScanMatchingOdometryDebug>()
{
  return &hdl_graph_slam::msg::typesupport_fastrtps_cpp::_ScanMatchingOdometryDebug__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, hdl_graph_slam, msg, ScanMatchingOdometryDebug)() {
  return &hdl_graph_slam::msg::typesupport_fastrtps_cpp::_ScanMatchingOdometryDebug__handle;
}

#ifdef __cplusplus
}
#endif
