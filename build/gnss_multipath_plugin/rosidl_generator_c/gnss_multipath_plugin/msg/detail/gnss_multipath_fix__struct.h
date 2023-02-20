// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from gnss_multipath_plugin:msg/GNSSMultipathFix.idl
// generated code does not contain a copyright notice

#ifndef GNSS_MULTIPATH_PLUGIN__MSG__DETAIL__GNSS_MULTIPATH_FIX__STRUCT_H_
#define GNSS_MULTIPATH_PLUGIN__MSG__DETAIL__GNSS_MULTIPATH_FIX__STRUCT_H_

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
// Member 'navsatfix'
#include "sensor_msgs/msg/detail/nav_sat_fix__struct.h"
// Member 'range_offset'
// Member 'sats_blocked'
#include "rosidl_runtime_c/primitives_sequence.h"

// Struct defined in msg/GNSSMultipathFix in the package gnss_multipath_plugin.
typedef struct gnss_multipath_plugin__msg__GNSSMultipathFix
{
  std_msgs__msg__Header header;
  sensor_msgs__msg__NavSatFix navsatfix;
  rosidl_runtime_c__float__Sequence range_offset;
  rosidl_runtime_c__int32__Sequence sats_blocked;
  int32_t num_vis_sat;
  int32_t num_block_sat;
} gnss_multipath_plugin__msg__GNSSMultipathFix;

// Struct for a sequence of gnss_multipath_plugin__msg__GNSSMultipathFix.
typedef struct gnss_multipath_plugin__msg__GNSSMultipathFix__Sequence
{
  gnss_multipath_plugin__msg__GNSSMultipathFix * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} gnss_multipath_plugin__msg__GNSSMultipathFix__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // GNSS_MULTIPATH_PLUGIN__MSG__DETAIL__GNSS_MULTIPATH_FIX__STRUCT_H_
