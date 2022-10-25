// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from costmap_converter_msgs:msg/ObstacleMsg.idl
// generated code does not contain a copyright notice

#ifndef COSTMAP_CONVERTER_MSGS__MSG__DETAIL__OBSTACLE_MSG__TRAITS_HPP_
#define COSTMAP_CONVERTER_MSGS__MSG__DETAIL__OBSTACLE_MSG__TRAITS_HPP_

#include "costmap_converter_msgs/msg/detail/obstacle_msg__struct.hpp"
#include <stdint.h>
#include <rosidl_runtime_cpp/traits.hpp>
#include <sstream>
#include <string>
#include <type_traits>

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'polygon'
#include "geometry_msgs/msg/detail/polygon__traits.hpp"
// Member 'orientation'
#include "geometry_msgs/msg/detail/quaternion__traits.hpp"
// Member 'velocities'
#include "geometry_msgs/msg/detail/twist_with_covariance__traits.hpp"

namespace rosidl_generator_traits
{

inline void to_yaml(
  const costmap_converter_msgs::msg::ObstacleMsg & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_yaml(msg.header, out, indentation + 2);
  }

  // member: polygon
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "polygon:\n";
    to_yaml(msg.polygon, out, indentation + 2);
  }

  // member: radius
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "radius: ";
    value_to_yaml(msg.radius, out);
    out << "\n";
  }

  // member: id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "id: ";
    value_to_yaml(msg.id, out);
    out << "\n";
  }

  // member: orientation
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "orientation:\n";
    to_yaml(msg.orientation, out, indentation + 2);
  }

  // member: velocities
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "velocities:\n";
    to_yaml(msg.velocities, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const costmap_converter_msgs::msg::ObstacleMsg & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<costmap_converter_msgs::msg::ObstacleMsg>()
{
  return "costmap_converter_msgs::msg::ObstacleMsg";
}

template<>
inline const char * name<costmap_converter_msgs::msg::ObstacleMsg>()
{
  return "costmap_converter_msgs/msg/ObstacleMsg";
}

template<>
struct has_fixed_size<costmap_converter_msgs::msg::ObstacleMsg>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Polygon>::value && has_fixed_size<geometry_msgs::msg::Quaternion>::value && has_fixed_size<geometry_msgs::msg::TwistWithCovariance>::value && has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<costmap_converter_msgs::msg::ObstacleMsg>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Polygon>::value && has_bounded_size<geometry_msgs::msg::Quaternion>::value && has_bounded_size<geometry_msgs::msg::TwistWithCovariance>::value && has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<costmap_converter_msgs::msg::ObstacleMsg>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // COSTMAP_CONVERTER_MSGS__MSG__DETAIL__OBSTACLE_MSG__TRAITS_HPP_
