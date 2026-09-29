// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from motor_temperature_pkg:msg/MotorTemperatures.idl
// generated code does not contain a copyright notice

#ifndef MOTOR_TEMPERATURE_PKG__MSG__DETAIL__MOTOR_TEMPERATURES__STRUCT_HPP_
#define MOTOR_TEMPERATURE_PKG__MSG__DETAIL__MOTOR_TEMPERATURES__STRUCT_HPP_

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
# define DEPRECATED__motor_temperature_pkg__msg__MotorTemperatures __attribute__((deprecated))
#else
# define DEPRECATED__motor_temperature_pkg__msg__MotorTemperatures __declspec(deprecated)
#endif

namespace motor_temperature_pkg
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MotorTemperatures_
{
  using Type = MotorTemperatures_<ContainerAllocator>;

  explicit MotorTemperatures_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->warning_threshold_c = 0.0f;
      this->overheated = false;
    }
  }

  explicit MotorTemperatures_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->warning_threshold_c = 0.0f;
      this->overheated = false;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _name_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _name_type name;
  using _temperature_c_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _temperature_c_type temperature_c;
  using _warning_threshold_c_type =
    float;
  _warning_threshold_c_type warning_threshold_c;
  using _overheated_index_type =
    std::vector<uint8_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint8_t>>;
  _overheated_index_type overheated_index;
  using _overheated_type =
    bool;
  _overheated_type overheated;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__name(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->name = _arg;
    return *this;
  }
  Type & set__temperature_c(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->temperature_c = _arg;
    return *this;
  }
  Type & set__warning_threshold_c(
    const float & _arg)
  {
    this->warning_threshold_c = _arg;
    return *this;
  }
  Type & set__overheated_index(
    const std::vector<uint8_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint8_t>> & _arg)
  {
    this->overheated_index = _arg;
    return *this;
  }
  Type & set__overheated(
    const bool & _arg)
  {
    this->overheated = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    motor_temperature_pkg::msg::MotorTemperatures_<ContainerAllocator> *;
  using ConstRawPtr =
    const motor_temperature_pkg::msg::MotorTemperatures_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<motor_temperature_pkg::msg::MotorTemperatures_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<motor_temperature_pkg::msg::MotorTemperatures_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      motor_temperature_pkg::msg::MotorTemperatures_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<motor_temperature_pkg::msg::MotorTemperatures_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      motor_temperature_pkg::msg::MotorTemperatures_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<motor_temperature_pkg::msg::MotorTemperatures_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<motor_temperature_pkg::msg::MotorTemperatures_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<motor_temperature_pkg::msg::MotorTemperatures_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__motor_temperature_pkg__msg__MotorTemperatures
    std::shared_ptr<motor_temperature_pkg::msg::MotorTemperatures_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__motor_temperature_pkg__msg__MotorTemperatures
    std::shared_ptr<motor_temperature_pkg::msg::MotorTemperatures_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MotorTemperatures_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->name != other.name) {
      return false;
    }
    if (this->temperature_c != other.temperature_c) {
      return false;
    }
    if (this->warning_threshold_c != other.warning_threshold_c) {
      return false;
    }
    if (this->overheated_index != other.overheated_index) {
      return false;
    }
    if (this->overheated != other.overheated) {
      return false;
    }
    return true;
  }
  bool operator!=(const MotorTemperatures_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MotorTemperatures_

// alias to use template instance with default allocator
using MotorTemperatures =
  motor_temperature_pkg::msg::MotorTemperatures_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace motor_temperature_pkg

#endif  // MOTOR_TEMPERATURE_PKG__MSG__DETAIL__MOTOR_TEMPERATURES__STRUCT_HPP_
