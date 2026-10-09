// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from robot_control:msg/I2cData.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "robot_control/msg/i2c_data.hpp"


#ifndef ROBOT_CONTROL__MSG__DETAIL__I2C_DATA__STRUCT_HPP_
#define ROBOT_CONTROL__MSG__DETAIL__I2C_DATA__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__robot_control__msg__I2cData __attribute__((deprecated))
#else
# define DEPRECATED__robot_control__msg__I2cData __declspec(deprecated)
#endif

namespace robot_control
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct I2cData_
{
  using Type = I2cData_<ContainerAllocator>;

  explicit I2cData_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x = 0.0f;
      this->y = 0.0f;
      this->z = 0.0f;
      this->timestamp = 0ul;
    }
  }

  explicit I2cData_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x = 0.0f;
      this->y = 0.0f;
      this->z = 0.0f;
      this->timestamp = 0ul;
    }
  }

  // field types and members
  using _x_type =
    float;
  _x_type x;
  using _y_type =
    float;
  _y_type y;
  using _z_type =
    float;
  _z_type z;
  using _timestamp_type =
    uint32_t;
  _timestamp_type timestamp;

  // setters for named parameter idiom
  Type & set__x(
    const float & _arg)
  {
    this->x = _arg;
    return *this;
  }
  Type & set__y(
    const float & _arg)
  {
    this->y = _arg;
    return *this;
  }
  Type & set__z(
    const float & _arg)
  {
    this->z = _arg;
    return *this;
  }
  Type & set__timestamp(
    const uint32_t & _arg)
  {
    this->timestamp = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robot_control::msg::I2cData_<ContainerAllocator> *;
  using ConstRawPtr =
    const robot_control::msg::I2cData_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robot_control::msg::I2cData_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robot_control::msg::I2cData_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robot_control::msg::I2cData_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robot_control::msg::I2cData_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robot_control::msg::I2cData_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robot_control::msg::I2cData_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robot_control::msg::I2cData_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robot_control::msg::I2cData_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robot_control__msg__I2cData
    std::shared_ptr<robot_control::msg::I2cData_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robot_control__msg__I2cData
    std::shared_ptr<robot_control::msg::I2cData_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const I2cData_ & other) const
  {
    if (this->x != other.x) {
      return false;
    }
    if (this->y != other.y) {
      return false;
    }
    if (this->z != other.z) {
      return false;
    }
    if (this->timestamp != other.timestamp) {
      return false;
    }
    return true;
  }
  bool operator!=(const I2cData_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct I2cData_

// alias to use template instance with default allocator
using I2cData =
  robot_control::msg::I2cData_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace robot_control

#endif  // ROBOT_CONTROL__MSG__DETAIL__I2C_DATA__STRUCT_HPP_
