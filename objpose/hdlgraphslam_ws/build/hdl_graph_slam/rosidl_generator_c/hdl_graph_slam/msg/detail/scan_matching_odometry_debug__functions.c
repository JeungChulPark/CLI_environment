// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from hdl_graph_slam:msg/ScanMatchingOdometryDebug.idl
// generated code does not contain a copyright notice
#include "hdl_graph_slam/msg/detail/scan_matching_odometry_debug__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
hdl_graph_slam__msg__ScanMatchingOdometryDebug__init(hdl_graph_slam__msg__ScanMatchingOdometryDebug * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    hdl_graph_slam__msg__ScanMatchingOdometryDebug__fini(msg);
    return false;
  }
  // input_point_count
  // downsampled_point_count
  // registration_time_ms
  // callback_time_ms
  // odom_count
  // registration_triggered
  // has_converged
  // matching_error
  return true;
}

void
hdl_graph_slam__msg__ScanMatchingOdometryDebug__fini(hdl_graph_slam__msg__ScanMatchingOdometryDebug * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // input_point_count
  // downsampled_point_count
  // registration_time_ms
  // callback_time_ms
  // odom_count
  // registration_triggered
  // has_converged
  // matching_error
}

bool
hdl_graph_slam__msg__ScanMatchingOdometryDebug__are_equal(const hdl_graph_slam__msg__ScanMatchingOdometryDebug * lhs, const hdl_graph_slam__msg__ScanMatchingOdometryDebug * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // input_point_count
  if (lhs->input_point_count != rhs->input_point_count) {
    return false;
  }
  // downsampled_point_count
  if (lhs->downsampled_point_count != rhs->downsampled_point_count) {
    return false;
  }
  // registration_time_ms
  if (lhs->registration_time_ms != rhs->registration_time_ms) {
    return false;
  }
  // callback_time_ms
  if (lhs->callback_time_ms != rhs->callback_time_ms) {
    return false;
  }
  // odom_count
  if (lhs->odom_count != rhs->odom_count) {
    return false;
  }
  // registration_triggered
  if (lhs->registration_triggered != rhs->registration_triggered) {
    return false;
  }
  // has_converged
  if (lhs->has_converged != rhs->has_converged) {
    return false;
  }
  // matching_error
  if (lhs->matching_error != rhs->matching_error) {
    return false;
  }
  return true;
}

bool
hdl_graph_slam__msg__ScanMatchingOdometryDebug__copy(
  const hdl_graph_slam__msg__ScanMatchingOdometryDebug * input,
  hdl_graph_slam__msg__ScanMatchingOdometryDebug * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // input_point_count
  output->input_point_count = input->input_point_count;
  // downsampled_point_count
  output->downsampled_point_count = input->downsampled_point_count;
  // registration_time_ms
  output->registration_time_ms = input->registration_time_ms;
  // callback_time_ms
  output->callback_time_ms = input->callback_time_ms;
  // odom_count
  output->odom_count = input->odom_count;
  // registration_triggered
  output->registration_triggered = input->registration_triggered;
  // has_converged
  output->has_converged = input->has_converged;
  // matching_error
  output->matching_error = input->matching_error;
  return true;
}

hdl_graph_slam__msg__ScanMatchingOdometryDebug *
hdl_graph_slam__msg__ScanMatchingOdometryDebug__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  hdl_graph_slam__msg__ScanMatchingOdometryDebug * msg = (hdl_graph_slam__msg__ScanMatchingOdometryDebug *)allocator.allocate(sizeof(hdl_graph_slam__msg__ScanMatchingOdometryDebug), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(hdl_graph_slam__msg__ScanMatchingOdometryDebug));
  bool success = hdl_graph_slam__msg__ScanMatchingOdometryDebug__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
hdl_graph_slam__msg__ScanMatchingOdometryDebug__destroy(hdl_graph_slam__msg__ScanMatchingOdometryDebug * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    hdl_graph_slam__msg__ScanMatchingOdometryDebug__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
hdl_graph_slam__msg__ScanMatchingOdometryDebug__Sequence__init(hdl_graph_slam__msg__ScanMatchingOdometryDebug__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  hdl_graph_slam__msg__ScanMatchingOdometryDebug * data = NULL;

  if (size) {
    data = (hdl_graph_slam__msg__ScanMatchingOdometryDebug *)allocator.zero_allocate(size, sizeof(hdl_graph_slam__msg__ScanMatchingOdometryDebug), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = hdl_graph_slam__msg__ScanMatchingOdometryDebug__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        hdl_graph_slam__msg__ScanMatchingOdometryDebug__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
hdl_graph_slam__msg__ScanMatchingOdometryDebug__Sequence__fini(hdl_graph_slam__msg__ScanMatchingOdometryDebug__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      hdl_graph_slam__msg__ScanMatchingOdometryDebug__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

hdl_graph_slam__msg__ScanMatchingOdometryDebug__Sequence *
hdl_graph_slam__msg__ScanMatchingOdometryDebug__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  hdl_graph_slam__msg__ScanMatchingOdometryDebug__Sequence * array = (hdl_graph_slam__msg__ScanMatchingOdometryDebug__Sequence *)allocator.allocate(sizeof(hdl_graph_slam__msg__ScanMatchingOdometryDebug__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = hdl_graph_slam__msg__ScanMatchingOdometryDebug__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
hdl_graph_slam__msg__ScanMatchingOdometryDebug__Sequence__destroy(hdl_graph_slam__msg__ScanMatchingOdometryDebug__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    hdl_graph_slam__msg__ScanMatchingOdometryDebug__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
hdl_graph_slam__msg__ScanMatchingOdometryDebug__Sequence__are_equal(const hdl_graph_slam__msg__ScanMatchingOdometryDebug__Sequence * lhs, const hdl_graph_slam__msg__ScanMatchingOdometryDebug__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!hdl_graph_slam__msg__ScanMatchingOdometryDebug__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
hdl_graph_slam__msg__ScanMatchingOdometryDebug__Sequence__copy(
  const hdl_graph_slam__msg__ScanMatchingOdometryDebug__Sequence * input,
  hdl_graph_slam__msg__ScanMatchingOdometryDebug__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(hdl_graph_slam__msg__ScanMatchingOdometryDebug);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    hdl_graph_slam__msg__ScanMatchingOdometryDebug * data =
      (hdl_graph_slam__msg__ScanMatchingOdometryDebug *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!hdl_graph_slam__msg__ScanMatchingOdometryDebug__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          hdl_graph_slam__msg__ScanMatchingOdometryDebug__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!hdl_graph_slam__msg__ScanMatchingOdometryDebug__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
