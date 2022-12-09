// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from multipath_sim:msg/MultipathOffset.idl
// generated code does not contain a copyright notice

#ifndef MULTIPATH_SIM__MSG__DETAIL__MULTIPATH_OFFSET__STRUCT_H_
#define MULTIPATH_SIM__MSG__DETAIL__MULTIPATH_OFFSET__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'range_offset'
// Member 'sats_blocked'
#include "rosidl_runtime_c/primitives_sequence.h"

// Struct defined in msg/MultipathOffset in the package multipath_sim.
typedef struct multipath_sim__msg__MultipathOffset
{
  std_msgs__msg__Header header;
  rosidl_runtime_c__float__Sequence range_offset;
  rosidl_runtime_c__int32__Sequence sats_blocked;
  int32_t num_vis_sat;
  int32_t num_block_sat;
} multipath_sim__msg__MultipathOffset;

// Struct for a sequence of multipath_sim__msg__MultipathOffset.
typedef struct multipath_sim__msg__MultipathOffset__Sequence
{
  multipath_sim__msg__MultipathOffset * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} multipath_sim__msg__MultipathOffset__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MULTIPATH_SIM__MSG__DETAIL__MULTIPATH_OFFSET__STRUCT_H_
