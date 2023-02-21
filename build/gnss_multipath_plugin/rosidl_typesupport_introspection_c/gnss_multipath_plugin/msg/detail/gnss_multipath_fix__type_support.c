// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from gnss_multipath_plugin:msg/GNSSMultipathFix.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "gnss_multipath_plugin/msg/detail/gnss_multipath_fix__rosidl_typesupport_introspection_c.h"
#include "gnss_multipath_plugin/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "gnss_multipath_plugin/msg/detail/gnss_multipath_fix__functions.h"
#include "gnss_multipath_plugin/msg/detail/gnss_multipath_fix__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `navsatfix`
#include "sensor_msgs/msg/nav_sat_fix.h"
// Member `navsatfix`
#include "sensor_msgs/msg/detail/nav_sat_fix__rosidl_typesupport_introspection_c.h"
// Member `enu_true`
// Member `enu_gnss_fix`
// Member `range_offset`
// Member `sats_blocked`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void GNSSMultipathFix__rosidl_typesupport_introspection_c__GNSSMultipathFix_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  gnss_multipath_plugin__msg__GNSSMultipathFix__init(message_memory);
}

void GNSSMultipathFix__rosidl_typesupport_introspection_c__GNSSMultipathFix_fini_function(void * message_memory)
{
  gnss_multipath_plugin__msg__GNSSMultipathFix__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember GNSSMultipathFix__rosidl_typesupport_introspection_c__GNSSMultipathFix_message_member_array[8] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(gnss_multipath_plugin__msg__GNSSMultipathFix, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "navsatfix",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(gnss_multipath_plugin__msg__GNSSMultipathFix, navsatfix),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "enu_true",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(gnss_multipath_plugin__msg__GNSSMultipathFix, enu_true),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "enu_gnss_fix",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(gnss_multipath_plugin__msg__GNSSMultipathFix, enu_gnss_fix),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "range_offset",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(gnss_multipath_plugin__msg__GNSSMultipathFix, range_offset),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "sats_blocked",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(gnss_multipath_plugin__msg__GNSSMultipathFix, sats_blocked),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "num_vis_sat",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(gnss_multipath_plugin__msg__GNSSMultipathFix, num_vis_sat),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "num_block_sat",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(gnss_multipath_plugin__msg__GNSSMultipathFix, num_block_sat),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers GNSSMultipathFix__rosidl_typesupport_introspection_c__GNSSMultipathFix_message_members = {
  "gnss_multipath_plugin__msg",  // message namespace
  "GNSSMultipathFix",  // message name
  8,  // number of fields
  sizeof(gnss_multipath_plugin__msg__GNSSMultipathFix),
  GNSSMultipathFix__rosidl_typesupport_introspection_c__GNSSMultipathFix_message_member_array,  // message members
  GNSSMultipathFix__rosidl_typesupport_introspection_c__GNSSMultipathFix_init_function,  // function to initialize message memory (memory has to be allocated)
  GNSSMultipathFix__rosidl_typesupport_introspection_c__GNSSMultipathFix_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t GNSSMultipathFix__rosidl_typesupport_introspection_c__GNSSMultipathFix_message_type_support_handle = {
  0,
  &GNSSMultipathFix__rosidl_typesupport_introspection_c__GNSSMultipathFix_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_gnss_multipath_plugin
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, gnss_multipath_plugin, msg, GNSSMultipathFix)() {
  GNSSMultipathFix__rosidl_typesupport_introspection_c__GNSSMultipathFix_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  GNSSMultipathFix__rosidl_typesupport_introspection_c__GNSSMultipathFix_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sensor_msgs, msg, NavSatFix)();
  if (!GNSSMultipathFix__rosidl_typesupport_introspection_c__GNSSMultipathFix_message_type_support_handle.typesupport_identifier) {
    GNSSMultipathFix__rosidl_typesupport_introspection_c__GNSSMultipathFix_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &GNSSMultipathFix__rosidl_typesupport_introspection_c__GNSSMultipathFix_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
