// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from yolo_msg:msg/BoundingBox.idl
// generated code does not contain a copyright notice

#ifndef YOLO_MSG__MSG__DETAIL__BOUNDING_BOX__STRUCT_H_
#define YOLO_MSG__MSG__DETAIL__BOUNDING_BOX__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'class_name'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/BoundingBox in the package yolo_msg.
typedef struct yolo_msg__msg__BoundingBox
{
  rosidl_runtime_c__String class_name;
  double score;
  int64_t top;
  int64_t left;
  int64_t bottom;
  int64_t right;
} yolo_msg__msg__BoundingBox;

// Struct for a sequence of yolo_msg__msg__BoundingBox.
typedef struct yolo_msg__msg__BoundingBox__Sequence
{
  yolo_msg__msg__BoundingBox * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} yolo_msg__msg__BoundingBox__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // YOLO_MSG__MSG__DETAIL__BOUNDING_BOX__STRUCT_H_
