// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from hdl_graph_slam:msg/FloorCoeffs.idl
// generated code does not contain a copyright notice
#include "hdl_graph_slam/msg/detail/floor_coeffs__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `coeffs`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
hdl_graph_slam__msg__FloorCoeffs__init(hdl_graph_slam__msg__FloorCoeffs * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    hdl_graph_slam__msg__FloorCoeffs__fini(msg);
    return false;
  }
  // coeffs
  if (!rosidl_runtime_c__float__Sequence__init(&msg->coeffs, 0)) {
    hdl_graph_slam__msg__FloorCoeffs__fini(msg);
    return false;
  }
  return true;
}

void
hdl_graph_slam__msg__FloorCoeffs__fini(hdl_graph_slam__msg__FloorCoeffs * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // coeffs
  rosidl_runtime_c__float__Sequence__fini(&msg->coeffs);
}

bool
hdl_graph_slam__msg__FloorCoeffs__are_equal(const hdl_graph_slam__msg__FloorCoeffs * lhs, const hdl_graph_slam__msg__FloorCoeffs * rhs)
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
  // coeffs
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->coeffs), &(rhs->coeffs)))
  {
    return false;
  }
  return true;
}

bool
hdl_graph_slam__msg__FloorCoeffs__copy(
  const hdl_graph_slam__msg__FloorCoeffs * input,
  hdl_graph_slam__msg__FloorCoeffs * output)
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
  // coeffs
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->coeffs), &(output->coeffs)))
  {
    return false;
  }
  return true;
}

hdl_graph_slam__msg__FloorCoeffs *
hdl_graph_slam__msg__FloorCoeffs__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  hdl_graph_slam__msg__FloorCoeffs * msg = (hdl_graph_slam__msg__FloorCoeffs *)allocator.allocate(sizeof(hdl_graph_slam__msg__FloorCoeffs), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(hdl_graph_slam__msg__FloorCoeffs));
  bool success = hdl_graph_slam__msg__FloorCoeffs__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
hdl_graph_slam__msg__FloorCoeffs__destroy(hdl_graph_slam__msg__FloorCoeffs * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    hdl_graph_slam__msg__FloorCoeffs__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
hdl_graph_slam__msg__FloorCoeffs__Sequence__init(hdl_graph_slam__msg__FloorCoeffs__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  hdl_graph_slam__msg__FloorCoeffs * data = NULL;

  if (size) {
    data = (hdl_graph_slam__msg__FloorCoeffs *)allocator.zero_allocate(size, sizeof(hdl_graph_slam__msg__FloorCoeffs), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = hdl_graph_slam__msg__FloorCoeffs__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        hdl_graph_slam__msg__FloorCoeffs__fini(&data[i - 1]);
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
hdl_graph_slam__msg__FloorCoeffs__Sequence__fini(hdl_graph_slam__msg__FloorCoeffs__Sequence * array)
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
      hdl_graph_slam__msg__FloorCoeffs__fini(&array->data[i]);
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

hdl_graph_slam__msg__FloorCoeffs__Sequence *
hdl_graph_slam__msg__FloorCoeffs__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  hdl_graph_slam__msg__FloorCoeffs__Sequence * array = (hdl_graph_slam__msg__FloorCoeffs__Sequence *)allocator.allocate(sizeof(hdl_graph_slam__msg__FloorCoeffs__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = hdl_graph_slam__msg__FloorCoeffs__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
hdl_graph_slam__msg__FloorCoeffs__Sequence__destroy(hdl_graph_slam__msg__FloorCoeffs__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    hdl_graph_slam__msg__FloorCoeffs__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
hdl_graph_slam__msg__FloorCoeffs__Sequence__are_equal(const hdl_graph_slam__msg__FloorCoeffs__Sequence * lhs, const hdl_graph_slam__msg__FloorCoeffs__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!hdl_graph_slam__msg__FloorCoeffs__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
hdl_graph_slam__msg__FloorCoeffs__Sequence__copy(
  const hdl_graph_slam__msg__FloorCoeffs__Sequence * input,
  hdl_graph_slam__msg__FloorCoeffs__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(hdl_graph_slam__msg__FloorCoeffs);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    hdl_graph_slam__msg__FloorCoeffs * data =
      (hdl_graph_slam__msg__FloorCoeffs *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!hdl_graph_slam__msg__FloorCoeffs__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          hdl_graph_slam__msg__FloorCoeffs__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!hdl_graph_slam__msg__FloorCoeffs__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
