// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from gnss_multipath_plugin:msg/GNSSMultipathFix.idl
// generated code does not contain a copyright notice
#include "gnss_multipath_plugin/msg/detail/gnss_multipath_fix__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `navsatfix`
#include "sensor_msgs/msg/detail/nav_sat_fix__functions.h"
// Member `enu_true`
// Member `enu_gnss_fix`
// Member `range_offset`
// Member `sats_blocked`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
gnss_multipath_plugin__msg__GNSSMultipathFix__init(gnss_multipath_plugin__msg__GNSSMultipathFix * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    gnss_multipath_plugin__msg__GNSSMultipathFix__fini(msg);
    return false;
  }
  // navsatfix
  if (!sensor_msgs__msg__NavSatFix__init(&msg->navsatfix)) {
    gnss_multipath_plugin__msg__GNSSMultipathFix__fini(msg);
    return false;
  }
  // enu_true
  if (!rosidl_runtime_c__float__Sequence__init(&msg->enu_true, 0)) {
    gnss_multipath_plugin__msg__GNSSMultipathFix__fini(msg);
    return false;
  }
  // enu_gnss_fix
  if (!rosidl_runtime_c__float__Sequence__init(&msg->enu_gnss_fix, 0)) {
    gnss_multipath_plugin__msg__GNSSMultipathFix__fini(msg);
    return false;
  }
  // range_offset
  if (!rosidl_runtime_c__float__Sequence__init(&msg->range_offset, 0)) {
    gnss_multipath_plugin__msg__GNSSMultipathFix__fini(msg);
    return false;
  }
  // sats_blocked
  if (!rosidl_runtime_c__int32__Sequence__init(&msg->sats_blocked, 0)) {
    gnss_multipath_plugin__msg__GNSSMultipathFix__fini(msg);
    return false;
  }
  // num_vis_sat
  // num_block_sat
  return true;
}

void
gnss_multipath_plugin__msg__GNSSMultipathFix__fini(gnss_multipath_plugin__msg__GNSSMultipathFix * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // navsatfix
  sensor_msgs__msg__NavSatFix__fini(&msg->navsatfix);
  // enu_true
  rosidl_runtime_c__float__Sequence__fini(&msg->enu_true);
  // enu_gnss_fix
  rosidl_runtime_c__float__Sequence__fini(&msg->enu_gnss_fix);
  // range_offset
  rosidl_runtime_c__float__Sequence__fini(&msg->range_offset);
  // sats_blocked
  rosidl_runtime_c__int32__Sequence__fini(&msg->sats_blocked);
  // num_vis_sat
  // num_block_sat
}

bool
gnss_multipath_plugin__msg__GNSSMultipathFix__are_equal(const gnss_multipath_plugin__msg__GNSSMultipathFix * lhs, const gnss_multipath_plugin__msg__GNSSMultipathFix * rhs)
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
  // navsatfix
  if (!sensor_msgs__msg__NavSatFix__are_equal(
      &(lhs->navsatfix), &(rhs->navsatfix)))
  {
    return false;
  }
  // enu_true
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->enu_true), &(rhs->enu_true)))
  {
    return false;
  }
  // enu_gnss_fix
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->enu_gnss_fix), &(rhs->enu_gnss_fix)))
  {
    return false;
  }
  // range_offset
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->range_offset), &(rhs->range_offset)))
  {
    return false;
  }
  // sats_blocked
  if (!rosidl_runtime_c__int32__Sequence__are_equal(
      &(lhs->sats_blocked), &(rhs->sats_blocked)))
  {
    return false;
  }
  // num_vis_sat
  if (lhs->num_vis_sat != rhs->num_vis_sat) {
    return false;
  }
  // num_block_sat
  if (lhs->num_block_sat != rhs->num_block_sat) {
    return false;
  }
  return true;
}

bool
gnss_multipath_plugin__msg__GNSSMultipathFix__copy(
  const gnss_multipath_plugin__msg__GNSSMultipathFix * input,
  gnss_multipath_plugin__msg__GNSSMultipathFix * output)
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
  // navsatfix
  if (!sensor_msgs__msg__NavSatFix__copy(
      &(input->navsatfix), &(output->navsatfix)))
  {
    return false;
  }
  // enu_true
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->enu_true), &(output->enu_true)))
  {
    return false;
  }
  // enu_gnss_fix
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->enu_gnss_fix), &(output->enu_gnss_fix)))
  {
    return false;
  }
  // range_offset
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->range_offset), &(output->range_offset)))
  {
    return false;
  }
  // sats_blocked
  if (!rosidl_runtime_c__int32__Sequence__copy(
      &(input->sats_blocked), &(output->sats_blocked)))
  {
    return false;
  }
  // num_vis_sat
  output->num_vis_sat = input->num_vis_sat;
  // num_block_sat
  output->num_block_sat = input->num_block_sat;
  return true;
}

gnss_multipath_plugin__msg__GNSSMultipathFix *
gnss_multipath_plugin__msg__GNSSMultipathFix__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  gnss_multipath_plugin__msg__GNSSMultipathFix * msg = (gnss_multipath_plugin__msg__GNSSMultipathFix *)allocator.allocate(sizeof(gnss_multipath_plugin__msg__GNSSMultipathFix), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(gnss_multipath_plugin__msg__GNSSMultipathFix));
  bool success = gnss_multipath_plugin__msg__GNSSMultipathFix__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
gnss_multipath_plugin__msg__GNSSMultipathFix__destroy(gnss_multipath_plugin__msg__GNSSMultipathFix * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    gnss_multipath_plugin__msg__GNSSMultipathFix__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
gnss_multipath_plugin__msg__GNSSMultipathFix__Sequence__init(gnss_multipath_plugin__msg__GNSSMultipathFix__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  gnss_multipath_plugin__msg__GNSSMultipathFix * data = NULL;

  if (size) {
    data = (gnss_multipath_plugin__msg__GNSSMultipathFix *)allocator.zero_allocate(size, sizeof(gnss_multipath_plugin__msg__GNSSMultipathFix), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = gnss_multipath_plugin__msg__GNSSMultipathFix__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        gnss_multipath_plugin__msg__GNSSMultipathFix__fini(&data[i - 1]);
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
gnss_multipath_plugin__msg__GNSSMultipathFix__Sequence__fini(gnss_multipath_plugin__msg__GNSSMultipathFix__Sequence * array)
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
      gnss_multipath_plugin__msg__GNSSMultipathFix__fini(&array->data[i]);
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

gnss_multipath_plugin__msg__GNSSMultipathFix__Sequence *
gnss_multipath_plugin__msg__GNSSMultipathFix__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  gnss_multipath_plugin__msg__GNSSMultipathFix__Sequence * array = (gnss_multipath_plugin__msg__GNSSMultipathFix__Sequence *)allocator.allocate(sizeof(gnss_multipath_plugin__msg__GNSSMultipathFix__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = gnss_multipath_plugin__msg__GNSSMultipathFix__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
gnss_multipath_plugin__msg__GNSSMultipathFix__Sequence__destroy(gnss_multipath_plugin__msg__GNSSMultipathFix__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    gnss_multipath_plugin__msg__GNSSMultipathFix__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
gnss_multipath_plugin__msg__GNSSMultipathFix__Sequence__are_equal(const gnss_multipath_plugin__msg__GNSSMultipathFix__Sequence * lhs, const gnss_multipath_plugin__msg__GNSSMultipathFix__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!gnss_multipath_plugin__msg__GNSSMultipathFix__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
gnss_multipath_plugin__msg__GNSSMultipathFix__Sequence__copy(
  const gnss_multipath_plugin__msg__GNSSMultipathFix__Sequence * input,
  gnss_multipath_plugin__msg__GNSSMultipathFix__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(gnss_multipath_plugin__msg__GNSSMultipathFix);
    gnss_multipath_plugin__msg__GNSSMultipathFix * data =
      (gnss_multipath_plugin__msg__GNSSMultipathFix *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!gnss_multipath_plugin__msg__GNSSMultipathFix__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          gnss_multipath_plugin__msg__GNSSMultipathFix__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!gnss_multipath_plugin__msg__GNSSMultipathFix__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
