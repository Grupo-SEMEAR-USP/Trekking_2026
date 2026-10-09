// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robot_control:msg/I2cData.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "robot_control/msg/i2c_data.hpp"


#ifndef ROBOT_CONTROL__MSG__DETAIL__I2C_DATA__BUILDER_HPP_
#define ROBOT_CONTROL__MSG__DETAIL__I2C_DATA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robot_control/msg/detail/i2c_data__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robot_control
{

namespace msg
{

namespace builder
{

class Init_I2cData_timestamp
{
public:
  explicit Init_I2cData_timestamp(::robot_control::msg::I2cData & msg)
  : msg_(msg)
  {}
  ::robot_control::msg::I2cData timestamp(::robot_control::msg::I2cData::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_control::msg::I2cData msg_;
};

class Init_I2cData_z
{
public:
  explicit Init_I2cData_z(::robot_control::msg::I2cData & msg)
  : msg_(msg)
  {}
  Init_I2cData_timestamp z(::robot_control::msg::I2cData::_z_type arg)
  {
    msg_.z = std::move(arg);
    return Init_I2cData_timestamp(msg_);
  }

private:
  ::robot_control::msg::I2cData msg_;
};

class Init_I2cData_y
{
public:
  explicit Init_I2cData_y(::robot_control::msg::I2cData & msg)
  : msg_(msg)
  {}
  Init_I2cData_z y(::robot_control::msg::I2cData::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_I2cData_z(msg_);
  }

private:
  ::robot_control::msg::I2cData msg_;
};

class Init_I2cData_x
{
public:
  Init_I2cData_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_I2cData_y x(::robot_control::msg::I2cData::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_I2cData_y(msg_);
  }

private:
  ::robot_control::msg::I2cData msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_control::msg::I2cData>()
{
  return robot_control::msg::builder::Init_I2cData_x();
}

}  // namespace robot_control

#endif  // ROBOT_CONTROL__MSG__DETAIL__I2C_DATA__BUILDER_HPP_
