// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from wheel_robot:msg/StepDetection.idl
// generated code does not contain a copyright notice

#ifndef WHEEL_ROBOT__MSG__DETAIL__STEP_DETECTION__STRUCT_HPP_
#define WHEEL_ROBOT__MSG__DETAIL__STEP_DETECTION__STRUCT_HPP_

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
# define DEPRECATED__wheel_robot__msg__StepDetection __attribute__((deprecated))
#else
# define DEPRECATED__wheel_robot__msg__StepDetection __declspec(deprecated)
#endif

namespace wheel_robot
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct StepDetection_
{
  using Type = StepDetection_<ContainerAllocator>;

  explicit StepDetection_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->detected = false;
      this->distance = 0.0f;
      this->height = 0.0f;
      this->confidence = 0.0f;
      this->pose_valid = false;
      this->edge_angle_rad = 0.0f;
      this->lateral_offset_m = 0.0f;
      this->confirm_frames = 0ul;
      this->required_confirm_frames = 0ul;
    }
  }

  explicit StepDetection_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->detected = false;
      this->distance = 0.0f;
      this->height = 0.0f;
      this->confidence = 0.0f;
      this->pose_valid = false;
      this->edge_angle_rad = 0.0f;
      this->lateral_offset_m = 0.0f;
      this->confirm_frames = 0ul;
      this->required_confirm_frames = 0ul;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _detected_type =
    bool;
  _detected_type detected;
  using _distance_type =
    float;
  _distance_type distance;
  using _height_type =
    float;
  _height_type height;
  using _confidence_type =
    float;
  _confidence_type confidence;
  using _pose_valid_type =
    bool;
  _pose_valid_type pose_valid;
  using _edge_angle_rad_type =
    float;
  _edge_angle_rad_type edge_angle_rad;
  using _lateral_offset_m_type =
    float;
  _lateral_offset_m_type lateral_offset_m;
  using _confirm_frames_type =
    uint32_t;
  _confirm_frames_type confirm_frames;
  using _required_confirm_frames_type =
    uint32_t;
  _required_confirm_frames_type required_confirm_frames;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__detected(
    const bool & _arg)
  {
    this->detected = _arg;
    return *this;
  }
  Type & set__distance(
    const float & _arg)
  {
    this->distance = _arg;
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
  Type & set__pose_valid(
    const bool & _arg)
  {
    this->pose_valid = _arg;
    return *this;
  }
  Type & set__edge_angle_rad(
    const float & _arg)
  {
    this->edge_angle_rad = _arg;
    return *this;
  }
  Type & set__lateral_offset_m(
    const float & _arg)
  {
    this->lateral_offset_m = _arg;
    return *this;
  }
  Type & set__confirm_frames(
    const uint32_t & _arg)
  {
    this->confirm_frames = _arg;
    return *this;
  }
  Type & set__required_confirm_frames(
    const uint32_t & _arg)
  {
    this->required_confirm_frames = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    wheel_robot::msg::StepDetection_<ContainerAllocator> *;
  using ConstRawPtr =
    const wheel_robot::msg::StepDetection_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<wheel_robot::msg::StepDetection_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<wheel_robot::msg::StepDetection_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      wheel_robot::msg::StepDetection_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<wheel_robot::msg::StepDetection_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      wheel_robot::msg::StepDetection_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<wheel_robot::msg::StepDetection_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<wheel_robot::msg::StepDetection_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<wheel_robot::msg::StepDetection_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__wheel_robot__msg__StepDetection
    std::shared_ptr<wheel_robot::msg::StepDetection_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__wheel_robot__msg__StepDetection
    std::shared_ptr<wheel_robot::msg::StepDetection_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const StepDetection_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->detected != other.detected) {
      return false;
    }
    if (this->distance != other.distance) {
      return false;
    }
    if (this->height != other.height) {
      return false;
    }
    if (this->confidence != other.confidence) {
      return false;
    }
    if (this->pose_valid != other.pose_valid) {
      return false;
    }
    if (this->edge_angle_rad != other.edge_angle_rad) {
      return false;
    }
    if (this->lateral_offset_m != other.lateral_offset_m) {
      return false;
    }
    if (this->confirm_frames != other.confirm_frames) {
      return false;
    }
    if (this->required_confirm_frames != other.required_confirm_frames) {
      return false;
    }
    return true;
  }
  bool operator!=(const StepDetection_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct StepDetection_

// alias to use template instance with default allocator
using StepDetection =
  wheel_robot::msg::StepDetection_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace wheel_robot

#endif  // WHEEL_ROBOT__MSG__DETAIL__STEP_DETECTION__STRUCT_HPP_
