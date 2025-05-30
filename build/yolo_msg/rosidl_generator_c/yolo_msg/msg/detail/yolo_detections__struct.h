// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from yolo_msg:msg/YoloDetections.idl
// generated code does not contain a copyright notice

#ifndef YOLO_MSG__MSG__DETAIL__YOLO_DETECTIONS__STRUCT_H_
#define YOLO_MSG__MSG__DETAIL__YOLO_DETECTIONS__STRUCT_H_

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
// Member 'bounding_boxes'
#include "yolo_msg/msg/detail/bounding_box__struct.h"

/// Struct defined in msg/YoloDetections in the package yolo_msg.
typedef struct yolo_msg__msg__YoloDetections
{
  std_msgs__msg__Header header;
  yolo_msg__msg__BoundingBox__Sequence bounding_boxes;
} yolo_msg__msg__YoloDetections;

// Struct for a sequence of yolo_msg__msg__YoloDetections.
typedef struct yolo_msg__msg__YoloDetections__Sequence
{
  yolo_msg__msg__YoloDetections * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} yolo_msg__msg__YoloDetections__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // YOLO_MSG__MSG__DETAIL__YOLO_DETECTIONS__STRUCT_H_
