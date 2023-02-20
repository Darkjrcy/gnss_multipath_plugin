// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from gnss_multipath_plugin:msg/GNSSMultipathFix.idl
// generated code does not contain a copyright notice

#ifndef GNSS_MULTIPATH_PLUGIN__MSG__DETAIL__GNSS_MULTIPATH_FIX__BUILDER_HPP_
#define GNSS_MULTIPATH_PLUGIN__MSG__DETAIL__GNSS_MULTIPATH_FIX__BUILDER_HPP_

#include "gnss_multipath_plugin/msg/detail/gnss_multipath_fix__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace gnss_multipath_plugin
{

namespace msg
{

namespace builder
{

class Init_GNSSMultipathFix_num_block_sat
{
public:
  explicit Init_GNSSMultipathFix_num_block_sat(::gnss_multipath_plugin::msg::GNSSMultipathFix & msg)
  : msg_(msg)
  {}
  ::gnss_multipath_plugin::msg::GNSSMultipathFix num_block_sat(::gnss_multipath_plugin::msg::GNSSMultipathFix::_num_block_sat_type arg)
  {
    msg_.num_block_sat = std::move(arg);
    return std::move(msg_);
  }

private:
  ::gnss_multipath_plugin::msg::GNSSMultipathFix msg_;
};

class Init_GNSSMultipathFix_num_vis_sat
{
public:
  explicit Init_GNSSMultipathFix_num_vis_sat(::gnss_multipath_plugin::msg::GNSSMultipathFix & msg)
  : msg_(msg)
  {}
  Init_GNSSMultipathFix_num_block_sat num_vis_sat(::gnss_multipath_plugin::msg::GNSSMultipathFix::_num_vis_sat_type arg)
  {
    msg_.num_vis_sat = std::move(arg);
    return Init_GNSSMultipathFix_num_block_sat(msg_);
  }

private:
  ::gnss_multipath_plugin::msg::GNSSMultipathFix msg_;
};

class Init_GNSSMultipathFix_sats_blocked
{
public:
  explicit Init_GNSSMultipathFix_sats_blocked(::gnss_multipath_plugin::msg::GNSSMultipathFix & msg)
  : msg_(msg)
  {}
  Init_GNSSMultipathFix_num_vis_sat sats_blocked(::gnss_multipath_plugin::msg::GNSSMultipathFix::_sats_blocked_type arg)
  {
    msg_.sats_blocked = std::move(arg);
    return Init_GNSSMultipathFix_num_vis_sat(msg_);
  }

private:
  ::gnss_multipath_plugin::msg::GNSSMultipathFix msg_;
};

class Init_GNSSMultipathFix_range_offset
{
public:
  explicit Init_GNSSMultipathFix_range_offset(::gnss_multipath_plugin::msg::GNSSMultipathFix & msg)
  : msg_(msg)
  {}
  Init_GNSSMultipathFix_sats_blocked range_offset(::gnss_multipath_plugin::msg::GNSSMultipathFix::_range_offset_type arg)
  {
    msg_.range_offset = std::move(arg);
    return Init_GNSSMultipathFix_sats_blocked(msg_);
  }

private:
  ::gnss_multipath_plugin::msg::GNSSMultipathFix msg_;
};

class Init_GNSSMultipathFix_navsatfix
{
public:
  explicit Init_GNSSMultipathFix_navsatfix(::gnss_multipath_plugin::msg::GNSSMultipathFix & msg)
  : msg_(msg)
  {}
  Init_GNSSMultipathFix_range_offset navsatfix(::gnss_multipath_plugin::msg::GNSSMultipathFix::_navsatfix_type arg)
  {
    msg_.navsatfix = std::move(arg);
    return Init_GNSSMultipathFix_range_offset(msg_);
  }

private:
  ::gnss_multipath_plugin::msg::GNSSMultipathFix msg_;
};

class Init_GNSSMultipathFix_header
{
public:
  Init_GNSSMultipathFix_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GNSSMultipathFix_navsatfix header(::gnss_multipath_plugin::msg::GNSSMultipathFix::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_GNSSMultipathFix_navsatfix(msg_);
  }

private:
  ::gnss_multipath_plugin::msg::GNSSMultipathFix msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::gnss_multipath_plugin::msg::GNSSMultipathFix>()
{
  return gnss_multipath_plugin::msg::builder::Init_GNSSMultipathFix_header();
}

}  // namespace gnss_multipath_plugin

#endif  // GNSS_MULTIPATH_PLUGIN__MSG__DETAIL__GNSS_MULTIPATH_FIX__BUILDER_HPP_
