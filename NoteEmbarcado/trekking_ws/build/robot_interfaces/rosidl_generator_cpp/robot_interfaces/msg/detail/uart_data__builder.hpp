// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robot_interfaces:msg/UARTData.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "robot_interfaces/msg/uart_data.hpp"


#ifndef ROBOT_INTERFACES__MSG__DETAIL__UART_DATA__BUILDER_HPP_
#define ROBOT_INTERFACES__MSG__DETAIL__UART_DATA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robot_interfaces/msg/detail/uart_data__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robot_interfaces
{

namespace msg
{

namespace builder
{

class Init_UARTData_timestamp
{
public:
  explicit Init_UARTData_timestamp(::robot_interfaces::msg::UARTData & msg)
  : msg_(msg)
  {}
  ::robot_interfaces::msg::UARTData timestamp(::robot_interfaces::msg::UARTData::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_interfaces::msg::UARTData msg_;
};

class Init_UARTData_z
{
public:
  explicit Init_UARTData_z(::robot_interfaces::msg::UARTData & msg)
  : msg_(msg)
  {}
  Init_UARTData_timestamp z(::robot_interfaces::msg::UARTData::_z_type arg)
  {
    msg_.z = std::move(arg);
    return Init_UARTData_timestamp(msg_);
  }

private:
  ::robot_interfaces::msg::UARTData msg_;
};

class Init_UARTData_y
{
public:
  explicit Init_UARTData_y(::robot_interfaces::msg::UARTData & msg)
  : msg_(msg)
  {}
  Init_UARTData_z y(::robot_interfaces::msg::UARTData::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_UARTData_z(msg_);
  }

private:
  ::robot_interfaces::msg::UARTData msg_;
};

class Init_UARTData_x
{
public:
  Init_UARTData_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_UARTData_y x(::robot_interfaces::msg::UARTData::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_UARTData_y(msg_);
  }

private:
  ::robot_interfaces::msg::UARTData msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_interfaces::msg::UARTData>()
{
  return robot_interfaces::msg::builder::Init_UARTData_x();
}

}  // namespace robot_interfaces

#endif  // ROBOT_INTERFACES__MSG__DETAIL__UART_DATA__BUILDER_HPP_
