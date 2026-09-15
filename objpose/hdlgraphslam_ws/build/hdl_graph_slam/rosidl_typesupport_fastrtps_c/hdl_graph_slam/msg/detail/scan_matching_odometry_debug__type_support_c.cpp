// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from hdl_graph_slam:msg/ScanMatchingOdometryDebug.idl
// generated code does not contain a copyright notice
#include "hdl_graph_slam/msg/detail/scan_matching_odometry_debug__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <cstddef>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/serialization_helpers.hpp"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "hdl_graph_slam/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "hdl_graph_slam/msg/detail/scan_matching_odometry_debug__struct.h"
#include "hdl_graph_slam/msg/detail/scan_matching_odometry_debug__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "std_msgs/msg/detail/header__functions.h"  // header

// forward declare type support functions

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_hdl_graph_slam
bool cdr_serialize_std_msgs__msg__Header(
  const std_msgs__msg__Header * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_hdl_graph_slam
bool cdr_deserialize_std_msgs__msg__Header(
  eprosima::fastcdr::Cdr & cdr,
  std_msgs__msg__Header * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_hdl_graph_slam
size_t get_serialized_size_std_msgs__msg__Header(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_hdl_graph_slam
size_t max_serialized_size_std_msgs__msg__Header(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_hdl_graph_slam
bool cdr_serialize_key_std_msgs__msg__Header(
  const std_msgs__msg__Header * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_hdl_graph_slam
size_t get_serialized_size_key_std_msgs__msg__Header(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_hdl_graph_slam
size_t max_serialized_size_key_std_msgs__msg__Header(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_hdl_graph_slam
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, std_msgs, msg, Header)();


using _ScanMatchingOdometryDebug__ros_msg_type = hdl_graph_slam__msg__ScanMatchingOdometryDebug;


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_hdl_graph_slam
bool cdr_serialize_hdl_graph_slam__msg__ScanMatchingOdometryDebug(
  const hdl_graph_slam__msg__ScanMatchingOdometryDebug * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: header
  {
    cdr_serialize_std_msgs__msg__Header(
      &ros_message->header, cdr);
  }

  // Field name: input_point_count
  {
    cdr << ros_message->input_point_count;
  }

  // Field name: downsampled_point_count
  {
    cdr << ros_message->downsampled_point_count;
  }

  // Field name: registration_time_ms
  {
    cdr << ros_message->registration_time_ms;
  }

  // Field name: callback_time_ms
  {
    cdr << ros_message->callback_time_ms;
  }

  // Field name: odom_count
  {
    cdr << ros_message->odom_count;
  }

  // Field name: registration_triggered
  {
    cdr << (ros_message->registration_triggered ? true : false);
  }

  // Field name: has_converged
  {
    cdr << (ros_message->has_converged ? true : false);
  }

  // Field name: matching_error
  {
    cdr << ros_message->matching_error;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_hdl_graph_slam
bool cdr_deserialize_hdl_graph_slam__msg__ScanMatchingOdometryDebug(
  eprosima::fastcdr::Cdr & cdr,
  hdl_graph_slam__msg__ScanMatchingOdometryDebug * ros_message)
{
  // Field name: header
  {
    cdr_deserialize_std_msgs__msg__Header(cdr, &ros_message->header);
  }

  // Field name: input_point_count
  {
    cdr >> ros_message->input_point_count;
  }

  // Field name: downsampled_point_count
  {
    cdr >> ros_message->downsampled_point_count;
  }

  // Field name: registration_time_ms
  {
    cdr >> ros_message->registration_time_ms;
  }

  // Field name: callback_time_ms
  {
    cdr >> ros_message->callback_time_ms;
  }

  // Field name: odom_count
  {
    cdr >> ros_message->odom_count;
  }

  // Field name: registration_triggered
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->registration_triggered = tmp ? true : false;
  }

  // Field name: has_converged
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->has_converged = tmp ? true : false;
  }

  // Field name: matching_error
  {
    cdr >> ros_message->matching_error;
  }

  return true;
}  // NOLINT(readability/fn_size)


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_hdl_graph_slam
size_t get_serialized_size_hdl_graph_slam__msg__ScanMatchingOdometryDebug(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _ScanMatchingOdometryDebug__ros_msg_type * ros_message = static_cast<const _ScanMatchingOdometryDebug__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: header
  current_alignment += get_serialized_size_std_msgs__msg__Header(
    &(ros_message->header), current_alignment);

  // Field name: input_point_count
  {
    size_t item_size = sizeof(ros_message->input_point_count);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: downsampled_point_count
  {
    size_t item_size = sizeof(ros_message->downsampled_point_count);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: registration_time_ms
  {
    size_t item_size = sizeof(ros_message->registration_time_ms);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: callback_time_ms
  {
    size_t item_size = sizeof(ros_message->callback_time_ms);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: odom_count
  {
    size_t item_size = sizeof(ros_message->odom_count);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: registration_triggered
  {
    size_t item_size = sizeof(ros_message->registration_triggered);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: has_converged
  {
    size_t item_size = sizeof(ros_message->has_converged);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: matching_error
  {
    size_t item_size = sizeof(ros_message->matching_error);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_hdl_graph_slam
size_t max_serialized_size_hdl_graph_slam__msg__ScanMatchingOdometryDebug(
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

  // Field name: header
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_std_msgs__msg__Header(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: input_point_count
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: downsampled_point_count
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: registration_time_ms
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: callback_time_ms
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: odom_count
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Field name: registration_triggered
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: has_converged
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: matching_error
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
    using DataType = hdl_graph_slam__msg__ScanMatchingOdometryDebug;
    is_plain =
      (
      offsetof(DataType, matching_error) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_hdl_graph_slam
bool cdr_serialize_key_hdl_graph_slam__msg__ScanMatchingOdometryDebug(
  const hdl_graph_slam__msg__ScanMatchingOdometryDebug * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: header
  {
    cdr_serialize_key_std_msgs__msg__Header(
      &ros_message->header, cdr);
  }

  // Field name: input_point_count
  {
    cdr << ros_message->input_point_count;
  }

  // Field name: downsampled_point_count
  {
    cdr << ros_message->downsampled_point_count;
  }

  // Field name: registration_time_ms
  {
    cdr << ros_message->registration_time_ms;
  }

  // Field name: callback_time_ms
  {
    cdr << ros_message->callback_time_ms;
  }

  // Field name: odom_count
  {
    cdr << ros_message->odom_count;
  }

  // Field name: registration_triggered
  {
    cdr << (ros_message->registration_triggered ? true : false);
  }

  // Field name: has_converged
  {
    cdr << (ros_message->has_converged ? true : false);
  }

  // Field name: matching_error
  {
    cdr << ros_message->matching_error;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_hdl_graph_slam
size_t get_serialized_size_key_hdl_graph_slam__msg__ScanMatchingOdometryDebug(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _ScanMatchingOdometryDebug__ros_msg_type * ros_message = static_cast<const _ScanMatchingOdometryDebug__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;

  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: header
  current_alignment += get_serialized_size_key_std_msgs__msg__Header(
    &(ros_message->header), current_alignment);

  // Field name: input_point_count
  {
    size_t item_size = sizeof(ros_message->input_point_count);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: downsampled_point_count
  {
    size_t item_size = sizeof(ros_message->downsampled_point_count);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: registration_time_ms
  {
    size_t item_size = sizeof(ros_message->registration_time_ms);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: callback_time_ms
  {
    size_t item_size = sizeof(ros_message->callback_time_ms);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: odom_count
  {
    size_t item_size = sizeof(ros_message->odom_count);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: registration_triggered
  {
    size_t item_size = sizeof(ros_message->registration_triggered);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: has_converged
  {
    size_t item_size = sizeof(ros_message->has_converged);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: matching_error
  {
    size_t item_size = sizeof(ros_message->matching_error);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_hdl_graph_slam
size_t max_serialized_size_key_hdl_graph_slam__msg__ScanMatchingOdometryDebug(
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
  // Field name: header
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_std_msgs__msg__Header(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: input_point_count
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: downsampled_point_count
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: registration_time_ms
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: callback_time_ms
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: odom_count
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Field name: registration_triggered
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: has_converged
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: matching_error
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
    using DataType = hdl_graph_slam__msg__ScanMatchingOdometryDebug;
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
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const hdl_graph_slam__msg__ScanMatchingOdometryDebug * ros_message = static_cast<const hdl_graph_slam__msg__ScanMatchingOdometryDebug *>(untyped_ros_message);
  (void)ros_message;
  return cdr_serialize_hdl_graph_slam__msg__ScanMatchingOdometryDebug(ros_message, cdr);
}

static bool _ScanMatchingOdometryDebug__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  hdl_graph_slam__msg__ScanMatchingOdometryDebug * ros_message = static_cast<hdl_graph_slam__msg__ScanMatchingOdometryDebug *>(untyped_ros_message);
  (void)ros_message;
  return cdr_deserialize_hdl_graph_slam__msg__ScanMatchingOdometryDebug(cdr, ros_message);
}

static uint32_t _ScanMatchingOdometryDebug__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_hdl_graph_slam__msg__ScanMatchingOdometryDebug(
      untyped_ros_message, 0));
}

static size_t _ScanMatchingOdometryDebug__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_hdl_graph_slam__msg__ScanMatchingOdometryDebug(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_ScanMatchingOdometryDebug = {
  "hdl_graph_slam::msg",
  "ScanMatchingOdometryDebug",
  _ScanMatchingOdometryDebug__cdr_serialize,
  _ScanMatchingOdometryDebug__cdr_deserialize,
  _ScanMatchingOdometryDebug__get_serialized_size,
  _ScanMatchingOdometryDebug__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _ScanMatchingOdometryDebug__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_ScanMatchingOdometryDebug,
  get_message_typesupport_handle_function,
  &hdl_graph_slam__msg__ScanMatchingOdometryDebug__get_type_hash,
  &hdl_graph_slam__msg__ScanMatchingOdometryDebug__get_type_description,
  &hdl_graph_slam__msg__ScanMatchingOdometryDebug__get_type_description_sources,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, hdl_graph_slam, msg, ScanMatchingOdometryDebug)() {
  return &_ScanMatchingOdometryDebug__type_support;
}

#if defined(__cplusplus)
}
#endif
