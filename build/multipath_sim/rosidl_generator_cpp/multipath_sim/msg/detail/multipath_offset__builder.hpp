// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from multipath_sim:msg/MultipathOffset.idl
// generated code does not contain a copyright notice

#ifndef MULTIPATH_SIM__MSG__DETAIL__MULTIPATH_OFFSET__BUILDER_HPP_
#define MULTIPATH_SIM__MSG__DETAIL__MULTIPATH_OFFSET__BUILDER_HPP_

#include "multipath_sim/msg/detail/multipath_offset__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace multipath_sim
{

namespace msg
{

namespace builder
{

class Init_MultipathOffset_num_block_sat
{
public:
  explicit Init_MultipathOffset_num_block_sat(::multipath_sim::msg::MultipathOffset & msg)
  : msg_(msg)
  {}
  ::multipath_sim::msg::MultipathOffset num_block_sat(::multipath_sim::msg::MultipathOffset::_num_block_sat_type arg)
  {
    msg_.num_block_sat = std::move(arg);
    return std::move(msg_);
  }

private:
  ::multipath_sim::msg::MultipathOffset msg_;
};

class Init_MultipathOffset_num_vis_sat
{
public:
  explicit Init_MultipathOffset_num_vis_sat(::multipath_sim::msg::MultipathOffset & msg)
  : msg_(msg)
  {}
  Init_MultipathOffset_num_block_sat num_vis_sat(::multipath_sim::msg::MultipathOffset::_num_vis_sat_type arg)
  {
    msg_.num_vis_sat = std::move(arg);
    return Init_MultipathOffset_num_block_sat(msg_);
  }

private:
  ::multipath_sim::msg::MultipathOffset msg_;
};

class Init_MultipathOffset_sats_blocked
{
public:
  explicit Init_MultipathOffset_sats_blocked(::multipath_sim::msg::MultipathOffset & msg)
  : msg_(msg)
  {}
  Init_MultipathOffset_num_vis_sat sats_blocked(::multipath_sim::msg::MultipathOffset::_sats_blocked_type arg)
  {
    msg_.sats_blocked = std::move(arg);
    return Init_MultipathOffset_num_vis_sat(msg_);
  }

private:
  ::multipath_sim::msg::MultipathOffset msg_;
};

class Init_MultipathOffset_range_offset
{
public:
  explicit Init_MultipathOffset_range_offset(::multipath_sim::msg::MultipathOffset & msg)
  : msg_(msg)
  {}
  Init_MultipathOffset_sats_blocked range_offset(::multipath_sim::msg::MultipathOffset::_range_offset_type arg)
  {
    msg_.range_offset = std::move(arg);
    return Init_MultipathOffset_sats_blocked(msg_);
  }

private:
  ::multipath_sim::msg::MultipathOffset msg_;
};

class Init_MultipathOffset_header
{
public:
  Init_MultipathOffset_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MultipathOffset_range_offset header(::multipath_sim::msg::MultipathOffset::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_MultipathOffset_range_offset(msg_);
  }

private:
  ::multipath_sim::msg::MultipathOffset msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::multipath_sim::msg::MultipathOffset>()
{
  return multipath_sim::msg::builder::Init_MultipathOffset_header();
}

}  // namespace multipath_sim

#endif  // MULTIPATH_SIM__MSG__DETAIL__MULTIPATH_OFFSET__BUILDER_HPP_
