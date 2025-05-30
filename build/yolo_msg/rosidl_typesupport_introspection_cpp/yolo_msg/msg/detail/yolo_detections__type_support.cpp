// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from yolo_msg:msg/YoloDetections.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "yolo_msg/msg/detail/yolo_detections__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace yolo_msg
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void YoloDetections_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) yolo_msg::msg::YoloDetections(_init);
}

void YoloDetections_fini_function(void * message_memory)
{
  auto typed_message = static_cast<yolo_msg::msg::YoloDetections *>(message_memory);
  typed_message->~YoloDetections();
}

size_t size_function__YoloDetections__bounding_boxes(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<yolo_msg::msg::BoundingBox> *>(untyped_member);
  return member->size();
}

const void * get_const_function__YoloDetections__bounding_boxes(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<yolo_msg::msg::BoundingBox> *>(untyped_member);
  return &member[index];
}

void * get_function__YoloDetections__bounding_boxes(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<yolo_msg::msg::BoundingBox> *>(untyped_member);
  return &member[index];
}

void fetch_function__YoloDetections__bounding_boxes(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const yolo_msg::msg::BoundingBox *>(
    get_const_function__YoloDetections__bounding_boxes(untyped_member, index));
  auto & value = *reinterpret_cast<yolo_msg::msg::BoundingBox *>(untyped_value);
  value = item;
}

void assign_function__YoloDetections__bounding_boxes(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<yolo_msg::msg::BoundingBox *>(
    get_function__YoloDetections__bounding_boxes(untyped_member, index));
  const auto & value = *reinterpret_cast<const yolo_msg::msg::BoundingBox *>(untyped_value);
  item = value;
}

void resize_function__YoloDetections__bounding_boxes(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<yolo_msg::msg::BoundingBox> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember YoloDetections_message_member_array[2] = {
  {
    "header",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Header>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(yolo_msg::msg::YoloDetections, header),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "bounding_boxes",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<yolo_msg::msg::BoundingBox>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(yolo_msg::msg::YoloDetections, bounding_boxes),  // bytes offset in struct
    nullptr,  // default value
    size_function__YoloDetections__bounding_boxes,  // size() function pointer
    get_const_function__YoloDetections__bounding_boxes,  // get_const(index) function pointer
    get_function__YoloDetections__bounding_boxes,  // get(index) function pointer
    fetch_function__YoloDetections__bounding_boxes,  // fetch(index, &value) function pointer
    assign_function__YoloDetections__bounding_boxes,  // assign(index, value) function pointer
    resize_function__YoloDetections__bounding_boxes  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers YoloDetections_message_members = {
  "yolo_msg::msg",  // message namespace
  "YoloDetections",  // message name
  2,  // number of fields
  sizeof(yolo_msg::msg::YoloDetections),
  YoloDetections_message_member_array,  // message members
  YoloDetections_init_function,  // function to initialize message memory (memory has to be allocated)
  YoloDetections_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t YoloDetections_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &YoloDetections_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace yolo_msg


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<yolo_msg::msg::YoloDetections>()
{
  return &::yolo_msg::msg::rosidl_typesupport_introspection_cpp::YoloDetections_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, yolo_msg, msg, YoloDetections)() {
  return &::yolo_msg::msg::rosidl_typesupport_introspection_cpp::YoloDetections_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
