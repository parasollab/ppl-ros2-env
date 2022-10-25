// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from costmap_converter_msgs:msg/ObstacleArrayMsg.idl
// generated code does not contain a copyright notice

#ifndef COSTMAP_CONVERTER_MSGS__MSG__DETAIL__OBSTACLE_ARRAY_MSG__TRAITS_HPP_
#define COSTMAP_CONVERTER_MSGS__MSG__DETAIL__OBSTACLE_ARRAY_MSG__TRAITS_HPP_

#include "costmap_converter_msgs/msg/detail/obstacle_array_msg__struct.hpp"
#include <stdint.h>
#include <rosidl_runtime_cpp/traits.hpp>
#include <sstream>
#include <string>
#include <type_traits>

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'obstacles'
#include "costmap_converter_msgs/msg/detail/obstacle_msg__traits.hpp"

namespace rosidl_generator_traits
{

inline void to_yaml(
  const costmap_converter_msgs::msg::ObstacleArrayMsg & msg,
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

  // member: obstacles
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.obstacles.size() == 0) {
      out << "obstacles: []\n";
    } else {
      out << "obstacles:\n";
      for (auto item : msg.obstacles) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const costmap_converter_msgs::msg::ObstacleArrayMsg & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<costmap_converter_msgs::msg::ObstacleArrayMsg>()
{
  return "costmap_converter_msgs::msg::ObstacleArrayMsg";
}

template<>
inline const char * name<costmap_converter_msgs::msg::ObstacleArrayMsg>()
{
  return "costmap_converter_msgs/msg/ObstacleArrayMsg";
}

template<>
struct has_fixed_size<costmap_converter_msgs::msg::ObstacleArrayMsg>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<costmap_converter_msgs::msg::ObstacleArrayMsg>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<costmap_converter_msgs::msg::ObstacleArrayMsg>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // COSTMAP_CONVERTER_MSGS__MSG__DETAIL__OBSTACLE_ARRAY_MSG__TRAITS_HPP_
