// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from wheel_robot:msg/ObstacleStatus.idl
// generated code does not contain a copyright notice

#ifndef WHEEL_ROBOT__MSG__DETAIL__OBSTACLE_STATUS__STRUCT_HPP_
#define WHEEL_ROBOT__MSG__DETAIL__OBSTACLE_STATUS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__wheel_robot__msg__ObstacleStatus __attribute__((deprecated))
#else
# define DEPRECATED__wheel_robot__msg__ObstacleStatus __declspec(deprecated)
#endif

namespace wheel_robot
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ObstacleStatus_
{
  using Type = ObstacleStatus_<ContainerAllocator>;

  explicit ObstacleStatus_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
      this->left_status = 0;
      this->center_status = 0;
      this->right_status = 0;
      this->left_distance = 0.0f;
      this->center_distance = 0.0f;
      this->right_distance = 0.0f;
    }
  }

  explicit ObstacleStatus_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
      this->left_status = 0;
      this->center_status = 0;
      this->right_status = 0;
      this->left_distance = 0.0f;
      this->center_distance = 0.0f;
      this->right_distance = 0.0f;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _status_type =
    uint8_t;
  _status_type status;
  using _left_status_type =
    uint8_t;
  _left_status_type left_status;
  using _center_status_type =
    uint8_t;
  _center_status_type center_status;
  using _right_status_type =
    uint8_t;
  _right_status_type right_status;
  using _left_distance_type =
    float;
  _left_distance_type left_distance;
  using _center_distance_type =
    float;
  _center_distance_type center_distance;
  using _right_distance_type =
    float;
  _right_distance_type right_distance;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__status(
    const uint8_t & _arg)
  {
    this->status = _arg;
    return *this;
  }
  Type & set__left_status(
    const uint8_t & _arg)
  {
    this->left_status = _arg;
    return *this;
  }
  Type & set__center_status(
    const uint8_t & _arg)
  {
    this->center_status = _arg;
    return *this;
  }
  Type & set__right_status(
    const uint8_t & _arg)
  {
    this->right_status = _arg;
    return *this;
  }
  Type & set__left_distance(
    const float & _arg)
  {
    this->left_distance = _arg;
    return *this;
  }
  Type & set__center_distance(
    const float & _arg)
  {
    this->center_distance = _arg;
    return *this;
  }
  Type & set__right_distance(
    const float & _arg)
  {
    this->right_distance = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t UNKNOWN =
    0u;
  static constexpr uint8_t CLEAR =
    1u;
  static constexpr uint8_t CAUTION =
    2u;
  static constexpr uint8_t STOP =
    3u;

  // pointer types
  using RawPtr =
    wheel_robot::msg::ObstacleStatus_<ContainerAllocator> *;
  using ConstRawPtr =
    const wheel_robot::msg::ObstacleStatus_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<wheel_robot::msg::ObstacleStatus_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<wheel_robot::msg::ObstacleStatus_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      wheel_robot::msg::ObstacleStatus_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<wheel_robot::msg::ObstacleStatus_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      wheel_robot::msg::ObstacleStatus_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<wheel_robot::msg::ObstacleStatus_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<wheel_robot::msg::ObstacleStatus_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<wheel_robot::msg::ObstacleStatus_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__wheel_robot__msg__ObstacleStatus
    std::shared_ptr<wheel_robot::msg::ObstacleStatus_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__wheel_robot__msg__ObstacleStatus
    std::shared_ptr<wheel_robot::msg::ObstacleStatus_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ObstacleStatus_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->status != other.status) {
      return false;
    }
    if (this->left_status != other.left_status) {
      return false;
    }
    if (this->center_status != other.center_status) {
      return false;
    }
    if (this->right_status != other.right_status) {
      return false;
    }
    if (this->left_distance != other.left_distance) {
      return false;
    }
    if (this->center_distance != other.center_distance) {
      return false;
    }
    if (this->right_distance != other.right_distance) {
      return false;
    }
    return true;
  }
  bool operator!=(const ObstacleStatus_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ObstacleStatus_

// alias to use template instance with default allocator
using ObstacleStatus =
  wheel_robot::msg::ObstacleStatus_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ObstacleStatus_<ContainerAllocator>::UNKNOWN;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ObstacleStatus_<ContainerAllocator>::CLEAR;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ObstacleStatus_<ContainerAllocator>::CAUTION;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ObstacleStatus_<ContainerAllocator>::STOP;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace wheel_robot

#endif  // WHEEL_ROBOT__MSG__DETAIL__OBSTACLE_STATUS__STRUCT_HPP_
