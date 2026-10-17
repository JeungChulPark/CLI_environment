// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from hdl_graph_slam:msg/ScanMatchingStatus.idl
// generated code does not contain a copyright notice
#include "hdl_graph_slam/msg/detail/scan_matching_status__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `relative_pose`
// Member `prediction_errors`
#include "geometry_msgs/msg/detail/pose__functions.h"
// Member `prediction_labels`
#include "std_msgs/msg/detail/string__functions.h"

bool
hdl_graph_slam__msg__ScanMatchingStatus__init(hdl_graph_slam__msg__ScanMatchingStatus * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    hdl_graph_slam__msg__ScanMatchingStatus__fini(msg);
    return false;
  }
  // has_converged
  // matching_error
  // inlier_fraction
  // relative_pose
  if (!geometry_msgs__msg__Pose__init(&msg->relative_pose)) {
    hdl_graph_slam__msg__ScanMatchingStatus__fini(msg);
    return false;
  }
  // prediction_labels
  if (!std_msgs__msg__String__Sequence__init(&msg->prediction_labels, 0)) {
    hdl_graph_slam__msg__ScanMatchingStatus__fini(msg);
    return false;
  }
  // prediction_errors
  if (!geometry_msgs__msg__Pose__Sequence__init(&msg->prediction_errors, 0)) {
    hdl_graph_slam__msg__ScanMatchingStatus__fini(msg);
    return false;
  }
  return true;
}

void
hdl_graph_slam__msg__ScanMatchingStatus__fini(hdl_graph_slam__msg__ScanMatchingStatus * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // has_converged
  // matching_error
  // inlier_fraction
  // relative_pose
  geometry_msgs__msg__Pose__fini(&msg->relative_pose);
  // prediction_labels
  std_msgs__msg__String__Sequence__fini(&msg->prediction_labels);
  // prediction_errors
  geometry_msgs__msg__Pose__Sequence__fini(&msg->prediction_errors);
}

bool
hdl_graph_slam__msg__ScanMatchingStatus__are_equal(const hdl_graph_slam__msg__ScanMatchingStatus * lhs, const hdl_graph_slam__msg__ScanMatchingStatus * rhs)
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
  // has_converged
  if (lhs->has_converged != rhs->has_converged) {
    return false;
  }
  // matching_error
  if (lhs->matching_error != rhs->matching_error) {
    return false;
  }
  // inlier_fraction
  if (lhs->inlier_fraction != rhs->inlier_fraction) {
    return false;
  }
  // relative_pose
  if (!geometry_msgs__msg__Pose__are_equal(
      &(lhs->relative_pose), &(rhs->relative_pose)))
  {
    return false;
  }
  // prediction_labels
  if (!std_msgs__msg__String__Sequence__are_equal(
      &(lhs->prediction_labels), &(rhs->prediction_labels)))
  {
    return false;
  }
  // prediction_errors
  if (!geometry_msgs__msg__Pose__Sequence__are_equal(
      &(lhs->prediction_errors), &(rhs->prediction_errors)))
  {
    return false;
  }
  return true;
}

bool
hdl_graph_slam__msg__ScanMatchingStatus__copy(
  const hdl_graph_slam__msg__ScanMatchingStatus * input,
  hdl_graph_slam__msg__ScanMatchingStatus * output)
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
  // has_converged
  output->has_converged = input->has_converged;
  // matching_error
  output->matching_error = input->matching_error;
  // inlier_fraction
  output->inlier_fraction = input->inlier_fraction;
  // relative_pose
  if (!geometry_msgs__msg__Pose__copy(
      &(input->relative_pose), &(output->relative_pose)))
  {
    return false;
  }
  // prediction_labels
  if (!std_msgs__msg__String__Sequence__copy(
      &(input->prediction_labels), &(output->prediction_labels)))
  {
    return false;
  }
  // prediction_errors
  if (!geometry_msgs__msg__Pose__Sequence__copy(
      &(input->prediction_errors), &(output->prediction_errors)))
  {
    return false;
  }
  return true;
}

hdl_graph_slam__msg__ScanMatchingStatus *
hdl_graph_slam__msg__ScanMatchingStatus__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  hdl_graph_slam__msg__ScanMatchingStatus * msg = (hdl_graph_slam__msg__ScanMatchingStatus *)allocator.allocate(sizeof(hdl_graph_slam__msg__ScanMatchingStatus), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(hdl_graph_slam__msg__ScanMatchingStatus));
  bool success = hdl_graph_slam__msg__ScanMatchingStatus__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
hdl_graph_slam__msg__ScanMatchingStatus__destroy(hdl_graph_slam__msg__ScanMatchingStatus * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    hdl_graph_slam__msg__ScanMatchingStatus__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
hdl_graph_slam__msg__ScanMatchingStatus__Sequence__init(hdl_graph_slam__msg__ScanMatchingStatus__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  hdl_graph_slam__msg__ScanMatchingStatus * data = NULL;

  if (size) {
    data = (hdl_graph_slam__msg__ScanMatchingStatus *)allocator.zero_allocate(size, sizeof(hdl_graph_slam__msg__ScanMatchingStatus), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = hdl_graph_slam__msg__ScanMatchingStatus__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        hdl_graph_slam__msg__ScanMatchingStatus__fini(&data[i - 1]);
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
hdl_graph_slam__msg__ScanMatchingStatus__Sequence__fini(hdl_graph_slam__msg__ScanMatchingStatus__Sequence * array)
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
      hdl_graph_slam__msg__ScanMatchingStatus__fini(&array->data[i]);
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

hdl_graph_slam__msg__ScanMatchingStatus__Sequence *
hdl_graph_slam__msg__ScanMatchingStatus__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  hdl_graph_slam__msg__ScanMatchingStatus__Sequence * array = (hdl_graph_slam__msg__ScanMatchingStatus__Sequence *)allocator.allocate(sizeof(hdl_graph_slam__msg__ScanMatchingStatus__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = hdl_graph_slam__msg__ScanMatchingStatus__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
hdl_graph_slam__msg__ScanMatchingStatus__Sequence__destroy(hdl_graph_slam__msg__ScanMatchingStatus__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    hdl_graph_slam__msg__ScanMatchingStatus__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
hdl_graph_slam__msg__ScanMatchingStatus__Sequence__are_equal(const hdl_graph_slam__msg__ScanMatchingStatus__Sequence * lhs, const hdl_graph_slam__msg__ScanMatchingStatus__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!hdl_graph_slam__msg__ScanMatchingStatus__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
hdl_graph_slam__msg__ScanMatchingStatus__Sequence__copy(
  const hdl_graph_slam__msg__ScanMatchingStatus__Sequence * input,
  hdl_graph_slam__msg__ScanMatchingStatus__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(hdl_graph_slam__msg__ScanMatchingStatus);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    hdl_graph_slam__msg__ScanMatchingStatus * data =
      (hdl_graph_slam__msg__ScanMatchingStatus *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!hdl_graph_slam__msg__ScanMatchingStatus__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          hdl_graph_slam__msg__ScanMatchingStatus__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!hdl_graph_slam__msg__ScanMatchingStatus__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
