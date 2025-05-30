// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from yolo_msg:msg/BoundingBox.idl
// generated code does not contain a copyright notice

#ifndef YOLO_MSG__MSG__DETAIL__BOUNDING_BOX__BUILDER_HPP_
#define YOLO_MSG__MSG__DETAIL__BOUNDING_BOX__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "yolo_msg/msg/detail/bounding_box__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace yolo_msg
{

namespace msg
{

namespace builder
{

class Init_BoundingBox_right
{
public:
  explicit Init_BoundingBox_right(::yolo_msg::msg::BoundingBox & msg)
  : msg_(msg)
  {}
  ::yolo_msg::msg::BoundingBox right(::yolo_msg::msg::BoundingBox::_right_type arg)
  {
    msg_.right = std::move(arg);
    return std::move(msg_);
  }

private:
  ::yolo_msg::msg::BoundingBox msg_;
};

class Init_BoundingBox_bottom
{
public:
  explicit Init_BoundingBox_bottom(::yolo_msg::msg::BoundingBox & msg)
  : msg_(msg)
  {}
  Init_BoundingBox_right bottom(::yolo_msg::msg::BoundingBox::_bottom_type arg)
  {
    msg_.bottom = std::move(arg);
    return Init_BoundingBox_right(msg_);
  }

private:
  ::yolo_msg::msg::BoundingBox msg_;
};

class Init_BoundingBox_left
{
public:
  explicit Init_BoundingBox_left(::yolo_msg::msg::BoundingBox & msg)
  : msg_(msg)
  {}
  Init_BoundingBox_bottom left(::yolo_msg::msg::BoundingBox::_left_type arg)
  {
    msg_.left = std::move(arg);
    return Init_BoundingBox_bottom(msg_);
  }

private:
  ::yolo_msg::msg::BoundingBox msg_;
};

class Init_BoundingBox_top
{
public:
  explicit Init_BoundingBox_top(::yolo_msg::msg::BoundingBox & msg)
  : msg_(msg)
  {}
  Init_BoundingBox_left top(::yolo_msg::msg::BoundingBox::_top_type arg)
  {
    msg_.top = std::move(arg);
    return Init_BoundingBox_left(msg_);
  }

private:
  ::yolo_msg::msg::BoundingBox msg_;
};

class Init_BoundingBox_score
{
public:
  explicit Init_BoundingBox_score(::yolo_msg::msg::BoundingBox & msg)
  : msg_(msg)
  {}
  Init_BoundingBox_top score(::yolo_msg::msg::BoundingBox::_score_type arg)
  {
    msg_.score = std::move(arg);
    return Init_BoundingBox_top(msg_);
  }

private:
  ::yolo_msg::msg::BoundingBox msg_;
};

class Init_BoundingBox_class_name
{
public:
  Init_BoundingBox_class_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_BoundingBox_score class_name(::yolo_msg::msg::BoundingBox::_class_name_type arg)
  {
    msg_.class_name = std::move(arg);
    return Init_BoundingBox_score(msg_);
  }

private:
  ::yolo_msg::msg::BoundingBox msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::yolo_msg::msg::BoundingBox>()
{
  return yolo_msg::msg::builder::Init_BoundingBox_class_name();
}

}  // namespace yolo_msg

#endif  // YOLO_MSG__MSG__DETAIL__BOUNDING_BOX__BUILDER_HPP_
