// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from gnss_multipath_plugin:msg/GNSSMultipathFix.idl
// generated code does not contain a copyright notice

#ifndef GNSS_MULTIPATH_PLUGIN__MSG__DETAIL__GNSS_MULTIPATH_FIX__FUNCTIONS_H_
#define GNSS_MULTIPATH_PLUGIN__MSG__DETAIL__GNSS_MULTIPATH_FIX__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "gnss_multipath_plugin/msg/rosidl_generator_c__visibility_control.h"

#include "gnss_multipath_plugin/msg/detail/gnss_multipath_fix__struct.h"

/// Initialize msg/GNSSMultipathFix message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * gnss_multipath_plugin__msg__GNSSMultipathFix
 * )) before or use
 * gnss_multipath_plugin__msg__GNSSMultipathFix__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_gnss_multipath_plugin
bool
gnss_multipath_plugin__msg__GNSSMultipathFix__init(gnss_multipath_plugin__msg__GNSSMultipathFix * msg);

/// Finalize msg/GNSSMultipathFix message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_gnss_multipath_plugin
void
gnss_multipath_plugin__msg__GNSSMultipathFix__fini(gnss_multipath_plugin__msg__GNSSMultipathFix * msg);

/// Create msg/GNSSMultipathFix message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * gnss_multipath_plugin__msg__GNSSMultipathFix__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_gnss_multipath_plugin
gnss_multipath_plugin__msg__GNSSMultipathFix *
gnss_multipath_plugin__msg__GNSSMultipathFix__create();

/// Destroy msg/GNSSMultipathFix message.
/**
 * It calls
 * gnss_multipath_plugin__msg__GNSSMultipathFix__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_gnss_multipath_plugin
void
gnss_multipath_plugin__msg__GNSSMultipathFix__destroy(gnss_multipath_plugin__msg__GNSSMultipathFix * msg);

/// Check for msg/GNSSMultipathFix message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_gnss_multipath_plugin
bool
gnss_multipath_plugin__msg__GNSSMultipathFix__are_equal(const gnss_multipath_plugin__msg__GNSSMultipathFix * lhs, const gnss_multipath_plugin__msg__GNSSMultipathFix * rhs);

/// Copy a msg/GNSSMultipathFix message.
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
ROSIDL_GENERATOR_C_PUBLIC_gnss_multipath_plugin
bool
gnss_multipath_plugin__msg__GNSSMultipathFix__copy(
  const gnss_multipath_plugin__msg__GNSSMultipathFix * input,
  gnss_multipath_plugin__msg__GNSSMultipathFix * output);

/// Initialize array of msg/GNSSMultipathFix messages.
/**
 * It allocates the memory for the number of elements and calls
 * gnss_multipath_plugin__msg__GNSSMultipathFix__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_gnss_multipath_plugin
bool
gnss_multipath_plugin__msg__GNSSMultipathFix__Sequence__init(gnss_multipath_plugin__msg__GNSSMultipathFix__Sequence * array, size_t size);

/// Finalize array of msg/GNSSMultipathFix messages.
/**
 * It calls
 * gnss_multipath_plugin__msg__GNSSMultipathFix__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_gnss_multipath_plugin
void
gnss_multipath_plugin__msg__GNSSMultipathFix__Sequence__fini(gnss_multipath_plugin__msg__GNSSMultipathFix__Sequence * array);

/// Create array of msg/GNSSMultipathFix messages.
/**
 * It allocates the memory for the array and calls
 * gnss_multipath_plugin__msg__GNSSMultipathFix__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_gnss_multipath_plugin
gnss_multipath_plugin__msg__GNSSMultipathFix__Sequence *
gnss_multipath_plugin__msg__GNSSMultipathFix__Sequence__create(size_t size);

/// Destroy array of msg/GNSSMultipathFix messages.
/**
 * It calls
 * gnss_multipath_plugin__msg__GNSSMultipathFix__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_gnss_multipath_plugin
void
gnss_multipath_plugin__msg__GNSSMultipathFix__Sequence__destroy(gnss_multipath_plugin__msg__GNSSMultipathFix__Sequence * array);

/// Check for msg/GNSSMultipathFix message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_gnss_multipath_plugin
bool
gnss_multipath_plugin__msg__GNSSMultipathFix__Sequence__are_equal(const gnss_multipath_plugin__msg__GNSSMultipathFix__Sequence * lhs, const gnss_multipath_plugin__msg__GNSSMultipathFix__Sequence * rhs);

/// Copy an array of msg/GNSSMultipathFix messages.
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
ROSIDL_GENERATOR_C_PUBLIC_gnss_multipath_plugin
bool
gnss_multipath_plugin__msg__GNSSMultipathFix__Sequence__copy(
  const gnss_multipath_plugin__msg__GNSSMultipathFix__Sequence * input,
  gnss_multipath_plugin__msg__GNSSMultipathFix__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // GNSS_MULTIPATH_PLUGIN__MSG__DETAIL__GNSS_MULTIPATH_FIX__FUNCTIONS_H_
