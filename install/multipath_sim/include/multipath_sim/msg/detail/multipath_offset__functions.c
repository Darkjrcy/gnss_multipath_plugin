// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from multipath_sim:msg/MultipathOffset.idl
// generated code does not contain a copyright notice
#include "multipath_sim/msg/detail/multipath_offset__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `range_offset`
// Member `sats_blocked`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
multipath_sim__msg__MultipathOffset__init(multipath_sim__msg__MultipathOffset * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    multipath_sim__msg__MultipathOffset__fini(msg);
    return false;
  }
  // range_offset
  if (!rosidl_runtime_c__float__Sequence__init(&msg->range_offset, 0)) {
    multipath_sim__msg__MultipathOffset__fini(msg);
    return false;
  }
  // sats_blocked
  if (!rosidl_runtime_c__int32__Sequence__init(&msg->sats_blocked, 0)) {
    multipath_sim__msg__MultipathOffset__fini(msg);
    return false;
  }
  // num_vis_sat
  // num_block_sat
  return true;
}

void
multipath_sim__msg__MultipathOffset__fini(multipath_sim__msg__MultipathOffset * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // range_offset
  rosidl_runtime_c__float__Sequence__fini(&msg->range_offset);
  // sats_blocked
  rosidl_runtime_c__int32__Sequence__fini(&msg->sats_blocked);
  // num_vis_sat
  // num_block_sat
}

bool
multipath_sim__msg__MultipathOffset__are_equal(const multipath_sim__msg__MultipathOffset * lhs, const multipath_sim__msg__MultipathOffset * rhs)
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
multipath_sim__msg__MultipathOffset__copy(
  const multipath_sim__msg__MultipathOffset * input,
  multipath_sim__msg__MultipathOffset * output)
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

multipath_sim__msg__MultipathOffset *
multipath_sim__msg__MultipathOffset__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  multipath_sim__msg__MultipathOffset * msg = (multipath_sim__msg__MultipathOffset *)allocator.allocate(sizeof(multipath_sim__msg__MultipathOffset), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(multipath_sim__msg__MultipathOffset));
  bool success = multipath_sim__msg__MultipathOffset__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
multipath_sim__msg__MultipathOffset__destroy(multipath_sim__msg__MultipathOffset * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    multipath_sim__msg__MultipathOffset__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
multipath_sim__msg__MultipathOffset__Sequence__init(multipath_sim__msg__MultipathOffset__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  multipath_sim__msg__MultipathOffset * data = NULL;

  if (size) {
    data = (multipath_sim__msg__MultipathOffset *)allocator.zero_allocate(size, sizeof(multipath_sim__msg__MultipathOffset), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = multipath_sim__msg__MultipathOffset__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        multipath_sim__msg__MultipathOffset__fini(&data[i - 1]);
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
multipath_sim__msg__MultipathOffset__Sequence__fini(multipath_sim__msg__MultipathOffset__Sequence * array)
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
      multipath_sim__msg__MultipathOffset__fini(&array->data[i]);
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

multipath_sim__msg__MultipathOffset__Sequence *
multipath_sim__msg__MultipathOffset__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  multipath_sim__msg__MultipathOffset__Sequence * array = (multipath_sim__msg__MultipathOffset__Sequence *)allocator.allocate(sizeof(multipath_sim__msg__MultipathOffset__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = multipath_sim__msg__MultipathOffset__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
multipath_sim__msg__MultipathOffset__Sequence__destroy(multipath_sim__msg__MultipathOffset__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    multipath_sim__msg__MultipathOffset__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
multipath_sim__msg__MultipathOffset__Sequence__are_equal(const multipath_sim__msg__MultipathOffset__Sequence * lhs, const multipath_sim__msg__MultipathOffset__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!multipath_sim__msg__MultipathOffset__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
multipath_sim__msg__MultipathOffset__Sequence__copy(
  const multipath_sim__msg__MultipathOffset__Sequence * input,
  multipath_sim__msg__MultipathOffset__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(multipath_sim__msg__MultipathOffset);
    multipath_sim__msg__MultipathOffset * data =
      (multipath_sim__msg__MultipathOffset *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!multipath_sim__msg__MultipathOffset__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          multipath_sim__msg__MultipathOffset__fini(&data[i]);
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
    if (!multipath_sim__msg__MultipathOffset__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
