// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from yolo_msg:msg/BoundingBox.idl
// generated code does not contain a copyright notice

#ifndef YOLO_MSG__MSG__DETAIL__BOUNDING_BOX__STRUCT_HPP_
#define YOLO_MSG__MSG__DETAIL__BOUNDING_BOX__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__yolo_msg__msg__BoundingBox __attribute__((deprecated))
#else
# define DEPRECATED__yolo_msg__msg__BoundingBox __declspec(deprecated)
#endif

namespace yolo_msg
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct BoundingBox_
{
  using Type = BoundingBox_<ContainerAllocator>;

  explicit BoundingBox_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->class_name = "";
      this->score = 0.0;
      this->top = 0ll;
      this->left = 0ll;
      this->bottom = 0ll;
      this->right = 0ll;
    }
  }

  explicit BoundingBox_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : class_name(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->class_name = "";
      this->score = 0.0;
      this->top = 0ll;
      this->left = 0ll;
      this->bottom = 0ll;
      this->right = 0ll;
    }
  }

  // field types and members
  using _class_name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _class_name_type class_name;
  using _score_type =
    double;
  _score_type score;
  using _top_type =
    int64_t;
  _top_type top;
  using _left_type =
    int64_t;
  _left_type left;
  using _bottom_type =
    int64_t;
  _bottom_type bottom;
  using _right_type =
    int64_t;
  _right_type right;

  // setters for named parameter idiom
  Type & set__class_name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->class_name = _arg;
    return *this;
  }
  Type & set__score(
    const double & _arg)
  {
    this->score = _arg;
    return *this;
  }
  Type & set__top(
    const int64_t & _arg)
  {
    this->top = _arg;
    return *this;
  }
  Type & set__left(
    const int64_t & _arg)
  {
    this->left = _arg;
    return *this;
  }
  Type & set__bottom(
    const int64_t & _arg)
  {
    this->bottom = _arg;
    return *this;
  }
  Type & set__right(
    const int64_t & _arg)
  {
    this->right = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    yolo_msg::msg::BoundingBox_<ContainerAllocator> *;
  using ConstRawPtr =
    const yolo_msg::msg::BoundingBox_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<yolo_msg::msg::BoundingBox_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<yolo_msg::msg::BoundingBox_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      yolo_msg::msg::BoundingBox_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<yolo_msg::msg::BoundingBox_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      yolo_msg::msg::BoundingBox_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<yolo_msg::msg::BoundingBox_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<yolo_msg::msg::BoundingBox_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<yolo_msg::msg::BoundingBox_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__yolo_msg__msg__BoundingBox
    std::shared_ptr<yolo_msg::msg::BoundingBox_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__yolo_msg__msg__BoundingBox
    std::shared_ptr<yolo_msg::msg::BoundingBox_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const BoundingBox_ & other) const
  {
    if (this->class_name != other.class_name) {
      return false;
    }
    if (this->score != other.score) {
      return false;
    }
    if (this->top != other.top) {
      return false;
    }
    if (this->left != other.left) {
      return false;
    }
    if (this->bottom != other.bottom) {
      return false;
    }
    if (this->right != other.right) {
      return false;
    }
    return true;
  }
  bool operator!=(const BoundingBox_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct BoundingBox_

// alias to use template instance with default allocator
using BoundingBox =
  yolo_msg::msg::BoundingBox_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace yolo_msg

#endif  // YOLO_MSG__MSG__DETAIL__BOUNDING_BOX__STRUCT_HPP_
