// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from robot_control:msg/VelocityData.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "robot_control/msg/velocity_data.hpp"


#ifndef ROBOT_CONTROL__MSG__DETAIL__VELOCITY_DATA__STRUCT_HPP_
#define ROBOT_CONTROL__MSG__DETAIL__VELOCITY_DATA__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__robot_control__msg__VelocityData __attribute__((deprecated))
#else
# define DEPRECATED__robot_control__msg__VelocityData __declspec(deprecated)
#endif

namespace robot_control
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct VelocityData_
{
  using Type = VelocityData_<ContainerAllocator>;

  explicit VelocityData_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->angular_speed_left = 0.0f;
      this->angular_speed_right = 0.0f;
      this->servo_angle = 0.0f;
    }
  }

  explicit VelocityData_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->angular_speed_left = 0.0f;
      this->angular_speed_right = 0.0f;
      this->servo_angle = 0.0f;
    }
  }

  // field types and members
  using _angular_speed_left_type =
    float;
  _angular_speed_left_type angular_speed_left;
  using _angular_speed_right_type =
    float;
  _angular_speed_right_type angular_speed_right;
  using _servo_angle_type =
    float;
  _servo_angle_type servo_angle;

  // setters for named parameter idiom
  Type & set__angular_speed_left(
    const float & _arg)
  {
    this->angular_speed_left = _arg;
    return *this;
  }
  Type & set__angular_speed_right(
    const float & _arg)
  {
    this->angular_speed_right = _arg;
    return *this;
  }
  Type & set__servo_angle(
    const float & _arg)
  {
    this->servo_angle = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robot_control::msg::VelocityData_<ContainerAllocator> *;
  using ConstRawPtr =
    const robot_control::msg::VelocityData_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robot_control::msg::VelocityData_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robot_control::msg::VelocityData_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robot_control::msg::VelocityData_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robot_control::msg::VelocityData_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robot_control::msg::VelocityData_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robot_control::msg::VelocityData_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robot_control::msg::VelocityData_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robot_control::msg::VelocityData_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robot_control__msg__VelocityData
    std::shared_ptr<robot_control::msg::VelocityData_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robot_control__msg__VelocityData
    std::shared_ptr<robot_control::msg::VelocityData_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const VelocityData_ & other) const
  {
    if (this->angular_speed_left != other.angular_speed_left) {
      return false;
    }
    if (this->angular_speed_right != other.angular_speed_right) {
      return false;
    }
    if (this->servo_angle != other.servo_angle) {
      return false;
    }
    return true;
  }
  bool operator!=(const VelocityData_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct VelocityData_

// alias to use template instance with default allocator
using VelocityData =
  robot_control::msg::VelocityData_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace robot_control

#endif  // ROBOT_CONTROL__MSG__DETAIL__VELOCITY_DATA__STRUCT_HPP_
