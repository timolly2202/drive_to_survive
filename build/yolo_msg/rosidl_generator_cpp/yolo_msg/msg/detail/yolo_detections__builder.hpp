// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from yolo_msg:msg/YoloDetections.idl
// generated code does not contain a copyright notice

#ifndef YOLO_MSG__MSG__DETAIL__YOLO_DETECTIONS__BUILDER_HPP_
#define YOLO_MSG__MSG__DETAIL__YOLO_DETECTIONS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "yolo_msg/msg/detail/yolo_detections__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace yolo_msg
{

namespace msg
{

namespace builder
{

class Init_YoloDetections_bounding_boxes
{
public:
  explicit Init_YoloDetections_bounding_boxes(::yolo_msg::msg::YoloDetections & msg)
  : msg_(msg)
  {}
  ::yolo_msg::msg::YoloDetections bounding_boxes(::yolo_msg::msg::YoloDetections::_bounding_boxes_type arg)
  {
    msg_.bounding_boxes = std::move(arg);
    return std::move(msg_);
  }

private:
  ::yolo_msg::msg::YoloDetections msg_;
};

class Init_YoloDetections_header
{
public:
  Init_YoloDetections_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_YoloDetections_bounding_boxes header(::yolo_msg::msg::YoloDetections::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_YoloDetections_bounding_boxes(msg_);
  }

private:
  ::yolo_msg::msg::YoloDetections msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::yolo_msg::msg::YoloDetections>()
{
  return yolo_msg::msg::builder::Init_YoloDetections_header();
}

}  // namespace yolo_msg

#endif  // YOLO_MSG__MSG__DETAIL__YOLO_DETECTIONS__BUILDER_HPP_
