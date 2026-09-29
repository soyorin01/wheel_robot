// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from wheel_robot:msg/StepApproachStatus.idl
// generated code does not contain a copyright notice

#ifndef WHEEL_ROBOT__MSG__DETAIL__STEP_APPROACH_STATUS__STRUCT_HPP_
#define WHEEL_ROBOT__MSG__DETAIL__STEP_APPROACH_STATUS__STRUCT_HPP_

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
# define DEPRECATED__wheel_robot__msg__StepApproachStatus __attribute__((deprecated))
#else
# define DEPRECATED__wheel_robot__msg__StepApproachStatus __declspec(deprecated)
#endif

namespace wheel_robot
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct StepApproachStatus_
{
  using Type = StepApproachStatus_<ContainerAllocator>;

  explicit StepApproachStatus_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->state = "";
      this->locked = false;
      this->step_detected = false;
      this->detected_distance = 0.0f;
      this->height = 0.0f;
      this->confidence = 0.0f;
      this->stop_distance = 0.0f;
      this->target_travel = 0.0f;
      this->traveled = 0.0f;
      this->remaining = 0.0f;
    }
  }

  explicit StepApproachStatus_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    state(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->state = "";
      this->locked = false;
      this->step_detected = false;
      this->detected_distance = 0.0f;
      this->height = 0.0f;
      this->confidence = 0.0f;
      this->stop_distance = 0.0f;
      this->target_travel = 0.0f;
      this->traveled = 0.0f;
      this->remaining = 0.0f;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _state_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _state_type state;
  using _locked_type =
    bool;
  _locked_type locked;
  using _step_detected_type =
    bool;
  _step_detected_type step_detected;
  using _detected_distance_type =
    float;
  _detected_distance_type detected_distance;
  using _height_type =
    float;
  _height_type height;
  using _confidence_type =
    float;
  _confidence_type confidence;
  using _stop_distance_type =
    float;
  _stop_distance_type stop_distance;
  using _target_travel_type =
    float;
  _target_travel_type target_travel;
  using _traveled_type =
    float;
  _traveled_type traveled;
  using _remaining_type =
    float;
  _remaining_type remaining;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__state(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->state = _arg;
    return *this;
  }
  Type & set__locked(
    const bool & _arg)
  {
    this->locked = _arg;
    return *this;
  }
  Type & set__step_detected(
    const bool & _arg)
  {
    this->step_detected = _arg;
    return *this;
  }
  Type & set__detected_distance(
    const float & _arg)
  {
    this->detected_distance = _arg;
    return *this;
  }
  Type & set__height(
    const float & _arg)
  {
    this->height = _arg;
    return *this;
  }
  Type & set__confidence(
    const float & _arg)
  {
    this->confidence = _arg;
    return *this;
  }
  Type & set__stop_distance(
    const float & _arg)
  {
    this->stop_distance = _arg;
    return *this;
  }
  Type & set__target_travel(
    const float & _arg)
  {
    this->target_travel = _arg;
    return *this;
  }
  Type & set__traveled(
    const float & _arg)
  {
    this->traveled = _arg;
    return *this;
  }
  Type & set__remaining(
    const float & _arg)
  {
    this->remaining = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    wheel_robot::msg::StepApproachStatus_<ContainerAllocator> *;
  using ConstRawPtr =
    const wheel_robot::msg::StepApproachStatus_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<wheel_robot::msg::StepApproachStatus_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<wheel_robot::msg::StepApproachStatus_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      wheel_robot::msg::StepApproachStatus_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<wheel_robot::msg::StepApproachStatus_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      wheel_robot::msg::StepApproachStatus_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<wheel_robot::msg::StepApproachStatus_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<wheel_robot::msg::StepApproachStatus_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<wheel_robot::msg::StepApproachStatus_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__wheel_robot__msg__StepApproachStatus
    std::shared_ptr<wheel_robot::msg::StepApproachStatus_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__wheel_robot__msg__StepApproachStatus
    std::shared_ptr<wheel_robot::msg::StepApproachStatus_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const StepApproachStatus_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->state != other.state) {
      return false;
    }
    if (this->locked != other.locked) {
      return false;
    }
    if (this->step_detected != other.step_detected) {
      return false;
    }
    if (this->detected_distance != other.detected_distance) {
      return false;
    }
    if (this->height != other.height) {
      return false;
    }
    if (this->confidence != other.confidence) {
      return false;
    }
    if (this->stop_distance != other.stop_distance) {
      return false;
    }
    if (this->target_travel != other.target_travel) {
      return false;
    }
    if (this->traveled != other.traveled) {
      return false;
    }
    if (this->remaining != other.remaining) {
      return false;
    }
    return true;
  }
  bool operator!=(const StepApproachStatus_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct StepApproachStatus_

// alias to use template instance with default allocator
using StepApproachStatus =
  wheel_robot::msg::StepApproachStatus_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace wheel_robot

#endif  // WHEEL_ROBOT__MSG__DETAIL__STEP_APPROACH_STATUS__STRUCT_HPP_
