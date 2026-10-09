// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robot_control:msg/VelocityData.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "robot_control/msg/velocity_data.hpp"


#ifndef ROBOT_CONTROL__MSG__DETAIL__VELOCITY_DATA__BUILDER_HPP_
#define ROBOT_CONTROL__MSG__DETAIL__VELOCITY_DATA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robot_control/msg/detail/velocity_data__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robot_control
{

namespace msg
{

namespace builder
{

class Init_VelocityData_servo_angle
{
public:
  explicit Init_VelocityData_servo_angle(::robot_control::msg::VelocityData & msg)
  : msg_(msg)
  {}
  ::robot_control::msg::VelocityData servo_angle(::robot_control::msg::VelocityData::_servo_angle_type arg)
  {
    msg_.servo_angle = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_control::msg::VelocityData msg_;
};

class Init_VelocityData_angular_speed_right
{
public:
  explicit Init_VelocityData_angular_speed_right(::robot_control::msg::VelocityData & msg)
  : msg_(msg)
  {}
  Init_VelocityData_servo_angle angular_speed_right(::robot_control::msg::VelocityData::_angular_speed_right_type arg)
  {
    msg_.angular_speed_right = std::move(arg);
    return Init_VelocityData_servo_angle(msg_);
  }

private:
  ::robot_control::msg::VelocityData msg_;
};

class Init_VelocityData_angular_speed_left
{
public:
  Init_VelocityData_angular_speed_left()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_VelocityData_angular_speed_right angular_speed_left(::robot_control::msg::VelocityData::_angular_speed_left_type arg)
  {
    msg_.angular_speed_left = std::move(arg);
    return Init_VelocityData_angular_speed_right(msg_);
  }

private:
  ::robot_control::msg::VelocityData msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_control::msg::VelocityData>()
{
  return robot_control::msg::builder::Init_VelocityData_angular_speed_left();
}

}  // namespace robot_control

#endif  // ROBOT_CONTROL__MSG__DETAIL__VELOCITY_DATA__BUILDER_HPP_
