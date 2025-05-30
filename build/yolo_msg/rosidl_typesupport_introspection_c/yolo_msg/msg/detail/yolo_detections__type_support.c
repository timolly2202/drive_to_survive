// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from yolo_msg:msg/YoloDetections.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "yolo_msg/msg/detail/yolo_detections__rosidl_typesupport_introspection_c.h"
#include "yolo_msg/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "yolo_msg/msg/detail/yolo_detections__functions.h"
#include "yolo_msg/msg/detail/yolo_detections__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `bounding_boxes`
#include "yolo_msg/msg/bounding_box.h"
// Member `bounding_boxes`
#include "yolo_msg/msg/detail/bounding_box__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void yolo_msg__msg__YoloDetections__rosidl_typesupport_introspection_c__YoloDetections_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  yolo_msg__msg__YoloDetections__init(message_memory);
}

void yolo_msg__msg__YoloDetections__rosidl_typesupport_introspection_c__YoloDetections_fini_function(void * message_memory)
{
  yolo_msg__msg__YoloDetections__fini(message_memory);
}

size_t yolo_msg__msg__YoloDetections__rosidl_typesupport_introspection_c__size_function__YoloDetections__bounding_boxes(
  const void * untyped_member)
{
  const yolo_msg__msg__BoundingBox__Sequence * member =
    (const yolo_msg__msg__BoundingBox__Sequence *)(untyped_member);
  return member->size;
}

const void * yolo_msg__msg__YoloDetections__rosidl_typesupport_introspection_c__get_const_function__YoloDetections__bounding_boxes(
  const void * untyped_member, size_t index)
{
  const yolo_msg__msg__BoundingBox__Sequence * member =
    (const yolo_msg__msg__BoundingBox__Sequence *)(untyped_member);
  return &member->data[index];
}

void * yolo_msg__msg__YoloDetections__rosidl_typesupport_introspection_c__get_function__YoloDetections__bounding_boxes(
  void * untyped_member, size_t index)
{
  yolo_msg__msg__BoundingBox__Sequence * member =
    (yolo_msg__msg__BoundingBox__Sequence *)(untyped_member);
  return &member->data[index];
}

void yolo_msg__msg__YoloDetections__rosidl_typesupport_introspection_c__fetch_function__YoloDetections__bounding_boxes(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const yolo_msg__msg__BoundingBox * item =
    ((const yolo_msg__msg__BoundingBox *)
    yolo_msg__msg__YoloDetections__rosidl_typesupport_introspection_c__get_const_function__YoloDetections__bounding_boxes(untyped_member, index));
  yolo_msg__msg__BoundingBox * value =
    (yolo_msg__msg__BoundingBox *)(untyped_value);
  *value = *item;
}

void yolo_msg__msg__YoloDetections__rosidl_typesupport_introspection_c__assign_function__YoloDetections__bounding_boxes(
  void * untyped_member, size_t index, const void * untyped_value)
{
  yolo_msg__msg__BoundingBox * item =
    ((yolo_msg__msg__BoundingBox *)
    yolo_msg__msg__YoloDetections__rosidl_typesupport_introspection_c__get_function__YoloDetections__bounding_boxes(untyped_member, index));
  const yolo_msg__msg__BoundingBox * value =
    (const yolo_msg__msg__BoundingBox *)(untyped_value);
  *item = *value;
}

bool yolo_msg__msg__YoloDetections__rosidl_typesupport_introspection_c__resize_function__YoloDetections__bounding_boxes(
  void * untyped_member, size_t size)
{
  yolo_msg__msg__BoundingBox__Sequence * member =
    (yolo_msg__msg__BoundingBox__Sequence *)(untyped_member);
  yolo_msg__msg__BoundingBox__Sequence__fini(member);
  return yolo_msg__msg__BoundingBox__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember yolo_msg__msg__YoloDetections__rosidl_typesupport_introspection_c__YoloDetections_message_member_array[2] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(yolo_msg__msg__YoloDetections, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "bounding_boxes",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(yolo_msg__msg__YoloDetections, bounding_boxes),  // bytes offset in struct
    NULL,  // default value
    yolo_msg__msg__YoloDetections__rosidl_typesupport_introspection_c__size_function__YoloDetections__bounding_boxes,  // size() function pointer
    yolo_msg__msg__YoloDetections__rosidl_typesupport_introspection_c__get_const_function__YoloDetections__bounding_boxes,  // get_const(index) function pointer
    yolo_msg__msg__YoloDetections__rosidl_typesupport_introspection_c__get_function__YoloDetections__bounding_boxes,  // get(index) function pointer
    yolo_msg__msg__YoloDetections__rosidl_typesupport_introspection_c__fetch_function__YoloDetections__bounding_boxes,  // fetch(index, &value) function pointer
    yolo_msg__msg__YoloDetections__rosidl_typesupport_introspection_c__assign_function__YoloDetections__bounding_boxes,  // assign(index, value) function pointer
    yolo_msg__msg__YoloDetections__rosidl_typesupport_introspection_c__resize_function__YoloDetections__bounding_boxes  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers yolo_msg__msg__YoloDetections__rosidl_typesupport_introspection_c__YoloDetections_message_members = {
  "yolo_msg__msg",  // message namespace
  "YoloDetections",  // message name
  2,  // number of fields
  sizeof(yolo_msg__msg__YoloDetections),
  yolo_msg__msg__YoloDetections__rosidl_typesupport_introspection_c__YoloDetections_message_member_array,  // message members
  yolo_msg__msg__YoloDetections__rosidl_typesupport_introspection_c__YoloDetections_init_function,  // function to initialize message memory (memory has to be allocated)
  yolo_msg__msg__YoloDetections__rosidl_typesupport_introspection_c__YoloDetections_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t yolo_msg__msg__YoloDetections__rosidl_typesupport_introspection_c__YoloDetections_message_type_support_handle = {
  0,
  &yolo_msg__msg__YoloDetections__rosidl_typesupport_introspection_c__YoloDetections_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_yolo_msg
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, yolo_msg, msg, YoloDetections)() {
  yolo_msg__msg__YoloDetections__rosidl_typesupport_introspection_c__YoloDetections_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  yolo_msg__msg__YoloDetections__rosidl_typesupport_introspection_c__YoloDetections_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, yolo_msg, msg, BoundingBox)();
  if (!yolo_msg__msg__YoloDetections__rosidl_typesupport_introspection_c__YoloDetections_message_type_support_handle.typesupport_identifier) {
    yolo_msg__msg__YoloDetections__rosidl_typesupport_introspection_c__YoloDetections_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &yolo_msg__msg__YoloDetections__rosidl_typesupport_introspection_c__YoloDetections_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
