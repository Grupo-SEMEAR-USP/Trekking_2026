// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from robot_interfaces:msg/VelocityData.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "robot_interfaces/msg/velocity_data.hpp"


#ifndef ROBOT_INTERFACES__MSG__DETAIL__VELOCITY_DATA__TRAITS_HPP_
#define ROBOT_INTERFACES__MSG__DETAIL__VELOCITY_DATA__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "robot_interfaces/msg/detail/velocity_data__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace robot_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const VelocityData & msg,
  std::ostream & out)
{
  out << "{";
  // member: angular_speed_left
  {
    out << "angular_speed_left: ";
    rosidl_generator_traits::value_to_yaml(msg.angular_speed_left, out);
    out << ", ";
  }

  // member: angular_speed_right
  {
    out << "angular_speed_right: ";
    rosidl_generator_traits::value_to_yaml(msg.angular_speed_right, out);
    out << ", ";
  }

  // member: servo_angle
  {
    out << "servo_angle: ";
    rosidl_generator_traits::value_to_yaml(msg.servo_angle, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const VelocityData & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: angular_speed_left
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "angular_speed_left: ";
    rosidl_generator_traits::value_to_yaml(msg.angular_speed_left, out);
    out << "\n";
  }

  // member: angular_speed_right
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "angular_speed_right: ";
    rosidl_generator_traits::value_to_yaml(msg.angular_speed_right, out);
    out << "\n";
  }

  // member: servo_angle
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "servo_angle: ";
    rosidl_generator_traits::value_to_yaml(msg.servo_angle, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const VelocityData & msg, bool use_flow_style = false)
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

}  // namespace robot_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use robot_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const robot_interfaces::msg::VelocityData & msg,
  std::ostream & out, size_t indentation = 0)
{
  robot_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use robot_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const robot_interfaces::msg::VelocityData & msg)
{
  return robot_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<robot_interfaces::msg::VelocityData>()
{
  return "robot_interfaces::msg::VelocityData";
}

template<>
inline const char * name<robot_interfaces::msg::VelocityData>()
{
  return "robot_interfaces/msg/VelocityData";
}

template<>
struct has_fixed_size<robot_interfaces::msg::VelocityData>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<robot_interfaces::msg::VelocityData>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<robot_interfaces::msg::VelocityData>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROBOT_INTERFACES__MSG__DETAIL__VELOCITY_DATA__TRAITS_HPP_
