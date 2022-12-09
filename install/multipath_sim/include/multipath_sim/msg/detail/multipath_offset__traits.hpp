// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from multipath_sim:msg/MultipathOffset.idl
// generated code does not contain a copyright notice

#ifndef MULTIPATH_SIM__MSG__DETAIL__MULTIPATH_OFFSET__TRAITS_HPP_
#define MULTIPATH_SIM__MSG__DETAIL__MULTIPATH_OFFSET__TRAITS_HPP_

#include "multipath_sim/msg/detail/multipath_offset__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<multipath_sim::msg::MultipathOffset>()
{
  return "multipath_sim::msg::MultipathOffset";
}

template<>
inline const char * name<multipath_sim::msg::MultipathOffset>()
{
  return "multipath_sim/msg/MultipathOffset";
}

template<>
struct has_fixed_size<multipath_sim::msg::MultipathOffset>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<multipath_sim::msg::MultipathOffset>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<multipath_sim::msg::MultipathOffset>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MULTIPATH_SIM__MSG__DETAIL__MULTIPATH_OFFSET__TRAITS_HPP_
