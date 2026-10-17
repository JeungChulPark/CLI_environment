// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from hdl_graph_slam:msg/FloorCoeffs.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "hdl_graph_slam/msg/floor_coeffs.h"


#ifndef HDL_GRAPH_SLAM__MSG__DETAIL__FLOOR_COEFFS__FUNCTIONS_H_
#define HDL_GRAPH_SLAM__MSG__DETAIL__FLOOR_COEFFS__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/action_type_support_struct.h"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_runtime_c/service_type_support_struct.h"
#include "rosidl_runtime_c/type_description/type_description__struct.h"
#include "rosidl_runtime_c/type_description/type_source__struct.h"
#include "rosidl_runtime_c/type_hash.h"
#include "rosidl_runtime_c/visibility_control.h"
#include "hdl_graph_slam/msg/rosidl_generator_c__visibility_control.h"

#include "hdl_graph_slam/msg/detail/floor_coeffs__struct.h"

/// Initialize msg/FloorCoeffs message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * hdl_graph_slam__msg__FloorCoeffs
 * )) before or use
 * hdl_graph_slam__msg__FloorCoeffs__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_hdl_graph_slam
bool
hdl_graph_slam__msg__FloorCoeffs__init(hdl_graph_slam__msg__FloorCoeffs * msg);

/// Finalize msg/FloorCoeffs message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hdl_graph_slam
void
hdl_graph_slam__msg__FloorCoeffs__fini(hdl_graph_slam__msg__FloorCoeffs * msg);

/// Create msg/FloorCoeffs message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * hdl_graph_slam__msg__FloorCoeffs__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_hdl_graph_slam
hdl_graph_slam__msg__FloorCoeffs *
hdl_graph_slam__msg__FloorCoeffs__create(void);

/// Destroy msg/FloorCoeffs message.
/**
 * It calls
 * hdl_graph_slam__msg__FloorCoeffs__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hdl_graph_slam
void
hdl_graph_slam__msg__FloorCoeffs__destroy(hdl_graph_slam__msg__FloorCoeffs * msg);

/// Check for msg/FloorCoeffs message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_hdl_graph_slam
bool
hdl_graph_slam__msg__FloorCoeffs__are_equal(const hdl_graph_slam__msg__FloorCoeffs * lhs, const hdl_graph_slam__msg__FloorCoeffs * rhs);

/// Copy a msg/FloorCoeffs message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_hdl_graph_slam
bool
hdl_graph_slam__msg__FloorCoeffs__copy(
  const hdl_graph_slam__msg__FloorCoeffs * input,
  hdl_graph_slam__msg__FloorCoeffs * output);

/// Retrieve pointer to the hash of the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_hdl_graph_slam
const rosidl_type_hash_t *
hdl_graph_slam__msg__FloorCoeffs__get_type_hash(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_hdl_graph_slam
const rosidl_runtime_c__type_description__TypeDescription *
hdl_graph_slam__msg__FloorCoeffs__get_type_description(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the single raw source text that defined this type.
ROSIDL_GENERATOR_C_PUBLIC_hdl_graph_slam
const rosidl_runtime_c__type_description__TypeSource *
hdl_graph_slam__msg__FloorCoeffs__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the recursive raw sources that defined the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_hdl_graph_slam
const rosidl_runtime_c__type_description__TypeSource__Sequence *
hdl_graph_slam__msg__FloorCoeffs__get_type_description_sources(
  const rosidl_message_type_support_t * type_support);

/// Initialize array of msg/FloorCoeffs messages.
/**
 * It allocates the memory for the number of elements and calls
 * hdl_graph_slam__msg__FloorCoeffs__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_hdl_graph_slam
bool
hdl_graph_slam__msg__FloorCoeffs__Sequence__init(hdl_graph_slam__msg__FloorCoeffs__Sequence * array, size_t size);

/// Finalize array of msg/FloorCoeffs messages.
/**
 * It calls
 * hdl_graph_slam__msg__FloorCoeffs__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hdl_graph_slam
void
hdl_graph_slam__msg__FloorCoeffs__Sequence__fini(hdl_graph_slam__msg__FloorCoeffs__Sequence * array);

/// Create array of msg/FloorCoeffs messages.
/**
 * It allocates the memory for the array and calls
 * hdl_graph_slam__msg__FloorCoeffs__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_hdl_graph_slam
hdl_graph_slam__msg__FloorCoeffs__Sequence *
hdl_graph_slam__msg__FloorCoeffs__Sequence__create(size_t size);

/// Destroy array of msg/FloorCoeffs messages.
/**
 * It calls
 * hdl_graph_slam__msg__FloorCoeffs__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hdl_graph_slam
void
hdl_graph_slam__msg__FloorCoeffs__Sequence__destroy(hdl_graph_slam__msg__FloorCoeffs__Sequence * array);

/// Check for msg/FloorCoeffs message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_hdl_graph_slam
bool
hdl_graph_slam__msg__FloorCoeffs__Sequence__are_equal(const hdl_graph_slam__msg__FloorCoeffs__Sequence * lhs, const hdl_graph_slam__msg__FloorCoeffs__Sequence * rhs);

/// Copy an array of msg/FloorCoeffs messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_hdl_graph_slam
bool
hdl_graph_slam__msg__FloorCoeffs__Sequence__copy(
  const hdl_graph_slam__msg__FloorCoeffs__Sequence * input,
  hdl_graph_slam__msg__FloorCoeffs__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // HDL_GRAPH_SLAM__MSG__DETAIL__FLOOR_COEFFS__FUNCTIONS_H_
