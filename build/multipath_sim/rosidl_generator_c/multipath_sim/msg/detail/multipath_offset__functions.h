// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from multipath_sim:msg/MultipathOffset.idl
// generated code does not contain a copyright notice

#ifndef MULTIPATH_SIM__MSG__DETAIL__MULTIPATH_OFFSET__FUNCTIONS_H_
#define MULTIPATH_SIM__MSG__DETAIL__MULTIPATH_OFFSET__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "multipath_sim/msg/rosidl_generator_c__visibility_control.h"

#include "multipath_sim/msg/detail/multipath_offset__struct.h"

/// Initialize msg/MultipathOffset message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * multipath_sim__msg__MultipathOffset
 * )) before or use
 * multipath_sim__msg__MultipathOffset__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_multipath_sim
bool
multipath_sim__msg__MultipathOffset__init(multipath_sim__msg__MultipathOffset * msg);

/// Finalize msg/MultipathOffset message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_multipath_sim
void
multipath_sim__msg__MultipathOffset__fini(multipath_sim__msg__MultipathOffset * msg);

/// Create msg/MultipathOffset message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * multipath_sim__msg__MultipathOffset__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_multipath_sim
multipath_sim__msg__MultipathOffset *
multipath_sim__msg__MultipathOffset__create();

/// Destroy msg/MultipathOffset message.
/**
 * It calls
 * multipath_sim__msg__MultipathOffset__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_multipath_sim
void
multipath_sim__msg__MultipathOffset__destroy(multipath_sim__msg__MultipathOffset * msg);

/// Check for msg/MultipathOffset message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_multipath_sim
bool
multipath_sim__msg__MultipathOffset__are_equal(const multipath_sim__msg__MultipathOffset * lhs, const multipath_sim__msg__MultipathOffset * rhs);

/// Copy a msg/MultipathOffset message.
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
ROSIDL_GENERATOR_C_PUBLIC_multipath_sim
bool
multipath_sim__msg__MultipathOffset__copy(
  const multipath_sim__msg__MultipathOffset * input,
  multipath_sim__msg__MultipathOffset * output);

/// Initialize array of msg/MultipathOffset messages.
/**
 * It allocates the memory for the number of elements and calls
 * multipath_sim__msg__MultipathOffset__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_multipath_sim
bool
multipath_sim__msg__MultipathOffset__Sequence__init(multipath_sim__msg__MultipathOffset__Sequence * array, size_t size);

/// Finalize array of msg/MultipathOffset messages.
/**
 * It calls
 * multipath_sim__msg__MultipathOffset__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_multipath_sim
void
multipath_sim__msg__MultipathOffset__Sequence__fini(multipath_sim__msg__MultipathOffset__Sequence * array);

/// Create array of msg/MultipathOffset messages.
/**
 * It allocates the memory for the array and calls
 * multipath_sim__msg__MultipathOffset__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_multipath_sim
multipath_sim__msg__MultipathOffset__Sequence *
multipath_sim__msg__MultipathOffset__Sequence__create(size_t size);

/// Destroy array of msg/MultipathOffset messages.
/**
 * It calls
 * multipath_sim__msg__MultipathOffset__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_multipath_sim
void
multipath_sim__msg__MultipathOffset__Sequence__destroy(multipath_sim__msg__MultipathOffset__Sequence * array);

/// Check for msg/MultipathOffset message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_multipath_sim
bool
multipath_sim__msg__MultipathOffset__Sequence__are_equal(const multipath_sim__msg__MultipathOffset__Sequence * lhs, const multipath_sim__msg__MultipathOffset__Sequence * rhs);

/// Copy an array of msg/MultipathOffset messages.
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
ROSIDL_GENERATOR_C_PUBLIC_multipath_sim
bool
multipath_sim__msg__MultipathOffset__Sequence__copy(
  const multipath_sim__msg__MultipathOffset__Sequence * input,
  multipath_sim__msg__MultipathOffset__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // MULTIPATH_SIM__MSG__DETAIL__MULTIPATH_OFFSET__FUNCTIONS_H_
