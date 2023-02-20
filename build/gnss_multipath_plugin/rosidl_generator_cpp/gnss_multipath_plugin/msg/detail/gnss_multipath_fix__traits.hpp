// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from gnss_multipath_plugin:msg/GNSSMultipathFix.idl
// generated code does not contain a copyright notice

#ifndef GNSS_MULTIPATH_PLUGIN__MSG__DETAIL__GNSS_MULTIPATH_FIX__TRAITS_HPP_
#define GNSS_MULTIPATH_PLUGIN__MSG__DETAIL__GNSS_MULTIPATH_FIX__TRAITS_HPP_

#include "gnss_multipath_plugin/msg/detail/gnss_multipath_fix__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'navsatfix'
#include "sensor_msgs/msg/detail/nav_sat_fix__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<gnss_multipath_plugin::msg::GNSSMultipathFix>()
{
  return "gnss_multipath_plugin::msg::GNSSMultipathFix";
}

template<>
inline const char * name<gnss_multipath_plugin::msg::GNSSMultipathFix>()
{
  return "gnss_multipath_plugin/msg/GNSSMultipathFix";
}

template<>
struct has_fixed_size<gnss_multipath_plugin::msg::GNSSMultipathFix>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<gnss_multipath_plugin::msg::GNSSMultipathFix>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<gnss_multipath_plugin::msg::GNSSMultipathFix>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // GNSS_MULTIPATH_PLUGIN__MSG__DETAIL__GNSS_MULTIPATH_FIX__TRAITS_HPP_
