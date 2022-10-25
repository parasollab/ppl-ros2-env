// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from teb_msgs:msg/TrajectoryPointMsg.idl
// generated code does not contain a copyright notice

#ifndef TEB_MSGS__MSG__DETAIL__TRAJECTORY_POINT_MSG__TRAITS_HPP_
#define TEB_MSGS__MSG__DETAIL__TRAJECTORY_POINT_MSG__TRAITS_HPP_

#include "teb_msgs/msg/detail/trajectory_point_msg__struct.hpp"
#include <stdint.h>
#include <rosidl_runtime_cpp/traits.hpp>
#include <sstream>
#include <string>
#include <type_traits>

// Include directives for member types
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__traits.hpp"
// Member 'velocity'
// Member 'acceleration'
#include "geometry_msgs/msg/detail/twist__traits.hpp"
// Member 'time_from_start'
#include "builtin_interfaces/msg/detail/duration__traits.hpp"

namespace rosidl_generator_traits
{

inline void to_yaml(
  const teb_msgs::msg::TrajectoryPointMsg & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: pose
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pose:\n";
    to_yaml(msg.pose, out, indentation + 2);
  }

  // member: velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "velocity:\n";
    to_yaml(msg.velocity, out, indentation + 2);
  }

  // member: acceleration
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "acceleration:\n";
    to_yaml(msg.acceleration, out, indentation + 2);
  }

  // member: time_from_start
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "time_from_start:\n";
    to_yaml(msg.time_from_start, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const teb_msgs::msg::TrajectoryPointMsg & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<teb_msgs::msg::TrajectoryPointMsg>()
{
  return "teb_msgs::msg::TrajectoryPointMsg";
}

template<>
inline const char * name<teb_msgs::msg::TrajectoryPointMsg>()
{
  return "teb_msgs/msg/TrajectoryPointMsg";
}

template<>
struct has_fixed_size<teb_msgs::msg::TrajectoryPointMsg>
  : std::integral_constant<bool, has_fixed_size<builtin_interfaces::msg::Duration>::value && has_fixed_size<geometry_msgs::msg::Pose>::value && has_fixed_size<geometry_msgs::msg::Twist>::value> {};

template<>
struct has_bounded_size<teb_msgs::msg::TrajectoryPointMsg>
  : std::integral_constant<bool, has_bounded_size<builtin_interfaces::msg::Duration>::value && has_bounded_size<geometry_msgs::msg::Pose>::value && has_bounded_size<geometry_msgs::msg::Twist>::value> {};

template<>
struct is_message<teb_msgs::msg::TrajectoryPointMsg>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // TEB_MSGS__MSG__DETAIL__TRAJECTORY_POINT_MSG__TRAITS_HPP_
