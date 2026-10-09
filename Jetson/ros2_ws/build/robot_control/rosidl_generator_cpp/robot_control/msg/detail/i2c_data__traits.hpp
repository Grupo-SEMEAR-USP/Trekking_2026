// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from robot_control:msg/I2cData.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "robot_control/msg/i2c_data.hpp"


#ifndef ROBOT_CONTROL__MSG__DETAIL__I2C_DATA__TRAITS_HPP_
#define ROBOT_CONTROL__MSG__DETAIL__I2C_DATA__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "robot_control/msg/detail/i2c_data__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace robot_control
{

namespace msg
{

inline void to_flow_style_yaml(
  const I2cData & msg,
  std::ostream & out)
{
  out << "{";
  // member: x
  {
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << ", ";
  }

  // member: y
  {
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << ", ";
  }

  // member: z
  {
    out << "z: ";
    rosidl_generator_traits::value_to_yaml(msg.z, out);
    out << ", ";
  }

  // member: timestamp
  {
    out << "timestamp: ";
    rosidl_generator_traits::value_to_yaml(msg.timestamp, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const I2cData & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << "\n";
  }

  // member: y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << "\n";
  }

  // member: z
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "z: ";
    rosidl_generator_traits::value_to_yaml(msg.z, out);
    out << "\n";
  }

  // member: timestamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "timestamp: ";
    rosidl_generator_traits::value_to_yaml(msg.timestamp, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const I2cData & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace robot_control

namespace rosidl_generator_traits
{

[[deprecated("use robot_control::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const robot_control::msg::I2cData & msg,
  std::ostream & out, size_t indentation = 0)
{
  robot_control::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use robot_control::msg::to_yaml() instead")]]
inline std::string to_yaml(const robot_control::msg::I2cData & msg)
{
  return robot_control::msg::to_yaml(msg);
}

template<>
inline const char * data_type<robot_control::msg::I2cData>()
{
  return "robot_control::msg::I2cData";
}

template<>
inline const char * name<robot_control::msg::I2cData>()
{
  return "robot_control/msg/I2cData";
}

template<>
struct has_fixed_size<robot_control::msg::I2cData>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<robot_control::msg::I2cData>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<robot_control::msg::I2cData>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROBOT_CONTROL__MSG__DETAIL__I2C_DATA__TRAITS_HPP_
