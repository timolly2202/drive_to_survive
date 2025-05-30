// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from yolo_msg:msg/YoloDetections.idl
// generated code does not contain a copyright notice
#include "yolo_msg/msg/detail/yolo_detections__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `bounding_boxes`
#include "yolo_msg/msg/detail/bounding_box__functions.h"

bool
yolo_msg__msg__YoloDetections__init(yolo_msg__msg__YoloDetections * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    yolo_msg__msg__YoloDetections__fini(msg);
    return false;
  }
  // bounding_boxes
  if (!yolo_msg__msg__BoundingBox__Sequence__init(&msg->bounding_boxes, 0)) {
    yolo_msg__msg__YoloDetections__fini(msg);
    return false;
  }
  return true;
}

void
yolo_msg__msg__YoloDetections__fini(yolo_msg__msg__YoloDetections * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // bounding_boxes
  yolo_msg__msg__BoundingBox__Sequence__fini(&msg->bounding_boxes);
}

bool
yolo_msg__msg__YoloDetections__are_equal(const yolo_msg__msg__YoloDetections * lhs, const yolo_msg__msg__YoloDetections * rhs)
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
  // bounding_boxes
  if (!yolo_msg__msg__BoundingBox__Sequence__are_equal(
      &(lhs->bounding_boxes), &(rhs->bounding_boxes)))
  {
    return false;
  }
  return true;
}

bool
yolo_msg__msg__YoloDetections__copy(
  const yolo_msg__msg__YoloDetections * input,
  yolo_msg__msg__YoloDetections * output)
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
  // bounding_boxes
  if (!yolo_msg__msg__BoundingBox__Sequence__copy(
      &(input->bounding_boxes), &(output->bounding_boxes)))
  {
    return false;
  }
  return true;
}

yolo_msg__msg__YoloDetections *
yolo_msg__msg__YoloDetections__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  yolo_msg__msg__YoloDetections * msg = (yolo_msg__msg__YoloDetections *)allocator.allocate(sizeof(yolo_msg__msg__YoloDetections), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(yolo_msg__msg__YoloDetections));
  bool success = yolo_msg__msg__YoloDetections__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
yolo_msg__msg__YoloDetections__destroy(yolo_msg__msg__YoloDetections * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    yolo_msg__msg__YoloDetections__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
yolo_msg__msg__YoloDetections__Sequence__init(yolo_msg__msg__YoloDetections__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  yolo_msg__msg__YoloDetections * data = NULL;

  if (size) {
    data = (yolo_msg__msg__YoloDetections *)allocator.zero_allocate(size, sizeof(yolo_msg__msg__YoloDetections), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = yolo_msg__msg__YoloDetections__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        yolo_msg__msg__YoloDetections__fini(&data[i - 1]);
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
yolo_msg__msg__YoloDetections__Sequence__fini(yolo_msg__msg__YoloDetections__Sequence * array)
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
      yolo_msg__msg__YoloDetections__fini(&array->data[i]);
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

yolo_msg__msg__YoloDetections__Sequence *
yolo_msg__msg__YoloDetections__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  yolo_msg__msg__YoloDetections__Sequence * array = (yolo_msg__msg__YoloDetections__Sequence *)allocator.allocate(sizeof(yolo_msg__msg__YoloDetections__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = yolo_msg__msg__YoloDetections__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
yolo_msg__msg__YoloDetections__Sequence__destroy(yolo_msg__msg__YoloDetections__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    yolo_msg__msg__YoloDetections__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
yolo_msg__msg__YoloDetections__Sequence__are_equal(const yolo_msg__msg__YoloDetections__Sequence * lhs, const yolo_msg__msg__YoloDetections__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!yolo_msg__msg__YoloDetections__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
yolo_msg__msg__YoloDetections__Sequence__copy(
  const yolo_msg__msg__YoloDetections__Sequence * input,
  yolo_msg__msg__YoloDetections__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(yolo_msg__msg__YoloDetections);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    yolo_msg__msg__YoloDetections * data =
      (yolo_msg__msg__YoloDetections *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!yolo_msg__msg__YoloDetections__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          yolo_msg__msg__YoloDetections__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!yolo_msg__msg__YoloDetections__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
