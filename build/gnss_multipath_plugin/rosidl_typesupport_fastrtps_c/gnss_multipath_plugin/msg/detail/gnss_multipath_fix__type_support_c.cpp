// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from gnss_multipath_plugin:msg/GNSSMultipathFix.idl
// generated code does not contain a copyright notice
#include "gnss_multipath_plugin/msg/detail/gnss_multipath_fix__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "gnss_multipath_plugin/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "gnss_multipath_plugin/msg/detail/gnss_multipath_fix__struct.h"
#include "gnss_multipath_plugin/msg/detail/gnss_multipath_fix__functions.h"
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

#include "rosidl_runtime_c/primitives_sequence.h"  // enu_gnss_fix, enu_true, range_offset, sats_blocked
#include "rosidl_runtime_c/primitives_sequence_functions.h"  // enu_gnss_fix, enu_true, range_offset, sats_blocked
#include "sensor_msgs/msg/detail/nav_sat_fix__functions.h"  // navsatfix
#include "std_msgs/msg/detail/header__functions.h"  // header

// forward declare type support functions
ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_gnss_multipath_plugin
size_t get_serialized_size_sensor_msgs__msg__NavSatFix(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_gnss_multipath_plugin
size_t max_serialized_size_sensor_msgs__msg__NavSatFix(
  bool & full_bounded,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_gnss_multipath_plugin
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, sensor_msgs, msg, NavSatFix)();
ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_gnss_multipath_plugin
size_t get_serialized_size_std_msgs__msg__Header(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_gnss_multipath_plugin
size_t max_serialized_size_std_msgs__msg__Header(
  bool & full_bounded,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_gnss_multipath_plugin
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, std_msgs, msg, Header)();


using _GNSSMultipathFix__ros_msg_type = gnss_multipath_plugin__msg__GNSSMultipathFix;

static bool _GNSSMultipathFix__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _GNSSMultipathFix__ros_msg_type * ros_message = static_cast<const _GNSSMultipathFix__ros_msg_type *>(untyped_ros_message);
  // Field name: header
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, std_msgs, msg, Header
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->header, cdr))
    {
      return false;
    }
  }

  // Field name: navsatfix
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, sensor_msgs, msg, NavSatFix
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->navsatfix, cdr))
    {
      return false;
    }
  }

  // Field name: enu_true
  {
    size_t size = ros_message->enu_true.size;
    auto array_ptr = ros_message->enu_true.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: enu_gnss_fix
  {
    size_t size = ros_message->enu_gnss_fix.size;
    auto array_ptr = ros_message->enu_gnss_fix.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: range_offset
  {
    size_t size = ros_message->range_offset.size;
    auto array_ptr = ros_message->range_offset.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: sats_blocked
  {
    size_t size = ros_message->sats_blocked.size;
    auto array_ptr = ros_message->sats_blocked.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: num_vis_sat
  {
    cdr << ros_message->num_vis_sat;
  }

  // Field name: num_block_sat
  {
    cdr << ros_message->num_block_sat;
  }

  return true;
}

static bool _GNSSMultipathFix__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _GNSSMultipathFix__ros_msg_type * ros_message = static_cast<_GNSSMultipathFix__ros_msg_type *>(untyped_ros_message);
  // Field name: header
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, std_msgs, msg, Header
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->header))
    {
      return false;
    }
  }

  // Field name: navsatfix
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, sensor_msgs, msg, NavSatFix
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->navsatfix))
    {
      return false;
    }
  }

  // Field name: enu_true
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->enu_true.data) {
      rosidl_runtime_c__float__Sequence__fini(&ros_message->enu_true);
    }
    if (!rosidl_runtime_c__float__Sequence__init(&ros_message->enu_true, size)) {
      return "failed to create array for field 'enu_true'";
    }
    auto array_ptr = ros_message->enu_true.data;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: enu_gnss_fix
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->enu_gnss_fix.data) {
      rosidl_runtime_c__float__Sequence__fini(&ros_message->enu_gnss_fix);
    }
    if (!rosidl_runtime_c__float__Sequence__init(&ros_message->enu_gnss_fix, size)) {
      return "failed to create array for field 'enu_gnss_fix'";
    }
    auto array_ptr = ros_message->enu_gnss_fix.data;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: range_offset
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->range_offset.data) {
      rosidl_runtime_c__float__Sequence__fini(&ros_message->range_offset);
    }
    if (!rosidl_runtime_c__float__Sequence__init(&ros_message->range_offset, size)) {
      return "failed to create array for field 'range_offset'";
    }
    auto array_ptr = ros_message->range_offset.data;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: sats_blocked
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->sats_blocked.data) {
      rosidl_runtime_c__int32__Sequence__fini(&ros_message->sats_blocked);
    }
    if (!rosidl_runtime_c__int32__Sequence__init(&ros_message->sats_blocked, size)) {
      return "failed to create array for field 'sats_blocked'";
    }
    auto array_ptr = ros_message->sats_blocked.data;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: num_vis_sat
  {
    cdr >> ros_message->num_vis_sat;
  }

  // Field name: num_block_sat
  {
    cdr >> ros_message->num_block_sat;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_gnss_multipath_plugin
size_t get_serialized_size_gnss_multipath_plugin__msg__GNSSMultipathFix(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _GNSSMultipathFix__ros_msg_type * ros_message = static_cast<const _GNSSMultipathFix__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name header

  current_alignment += get_serialized_size_std_msgs__msg__Header(
    &(ros_message->header), current_alignment);
  // field.name navsatfix

  current_alignment += get_serialized_size_sensor_msgs__msg__NavSatFix(
    &(ros_message->navsatfix), current_alignment);
  // field.name enu_true
  {
    size_t array_size = ros_message->enu_true.size;
    auto array_ptr = ros_message->enu_true.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name enu_gnss_fix
  {
    size_t array_size = ros_message->enu_gnss_fix.size;
    auto array_ptr = ros_message->enu_gnss_fix.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name range_offset
  {
    size_t array_size = ros_message->range_offset.size;
    auto array_ptr = ros_message->range_offset.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name sats_blocked
  {
    size_t array_size = ros_message->sats_blocked.size;
    auto array_ptr = ros_message->sats_blocked.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name num_vis_sat
  {
    size_t item_size = sizeof(ros_message->num_vis_sat);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name num_block_sat
  {
    size_t item_size = sizeof(ros_message->num_block_sat);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _GNSSMultipathFix__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_gnss_multipath_plugin__msg__GNSSMultipathFix(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_gnss_multipath_plugin
size_t max_serialized_size_gnss_multipath_plugin__msg__GNSSMultipathFix(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

  // member: header
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        max_serialized_size_std_msgs__msg__Header(
        full_bounded, current_alignment);
    }
  }
  // member: navsatfix
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        max_serialized_size_sensor_msgs__msg__NavSatFix(
        full_bounded, current_alignment);
    }
  }
  // member: enu_true
  {
    size_t array_size = 0;
    full_bounded = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: enu_gnss_fix
  {
    size_t array_size = 0;
    full_bounded = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: range_offset
  {
    size_t array_size = 0;
    full_bounded = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: sats_blocked
  {
    size_t array_size = 0;
    full_bounded = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: num_vis_sat
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: num_block_sat
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  return current_alignment - initial_alignment;
}

static size_t _GNSSMultipathFix__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_gnss_multipath_plugin__msg__GNSSMultipathFix(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_GNSSMultipathFix = {
  "gnss_multipath_plugin::msg",
  "GNSSMultipathFix",
  _GNSSMultipathFix__cdr_serialize,
  _GNSSMultipathFix__cdr_deserialize,
  _GNSSMultipathFix__get_serialized_size,
  _GNSSMultipathFix__max_serialized_size
};

static rosidl_message_type_support_t _GNSSMultipathFix__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_GNSSMultipathFix,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, gnss_multipath_plugin, msg, GNSSMultipathFix)() {
  return &_GNSSMultipathFix__type_support;
}

#if defined(__cplusplus)
}
#endif
