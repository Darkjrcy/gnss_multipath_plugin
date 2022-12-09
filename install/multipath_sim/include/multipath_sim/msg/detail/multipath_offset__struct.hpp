// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from multipath_sim:msg/MultipathOffset.idl
// generated code does not contain a copyright notice

#ifndef MULTIPATH_SIM__MSG__DETAIL__MULTIPATH_OFFSET__STRUCT_HPP_
#define MULTIPATH_SIM__MSG__DETAIL__MULTIPATH_OFFSET__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__multipath_sim__msg__MultipathOffset __attribute__((deprecated))
#else
# define DEPRECATED__multipath_sim__msg__MultipathOffset __declspec(deprecated)
#endif

namespace multipath_sim
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MultipathOffset_
{
  using Type = MultipathOffset_<ContainerAllocator>;

  explicit MultipathOffset_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->num_vis_sat = 0l;
      this->num_block_sat = 0l;
    }
  }

  explicit MultipathOffset_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->num_vis_sat = 0l;
      this->num_block_sat = 0l;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _range_offset_type =
    std::vector<float, typename ContainerAllocator::template rebind<float>::other>;
  _range_offset_type range_offset;
  using _sats_blocked_type =
    std::vector<int32_t, typename ContainerAllocator::template rebind<int32_t>::other>;
  _sats_blocked_type sats_blocked;
  using _num_vis_sat_type =
    int32_t;
  _num_vis_sat_type num_vis_sat;
  using _num_block_sat_type =
    int32_t;
  _num_block_sat_type num_block_sat;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__range_offset(
    const std::vector<float, typename ContainerAllocator::template rebind<float>::other> & _arg)
  {
    this->range_offset = _arg;
    return *this;
  }
  Type & set__sats_blocked(
    const std::vector<int32_t, typename ContainerAllocator::template rebind<int32_t>::other> & _arg)
  {
    this->sats_blocked = _arg;
    return *this;
  }
  Type & set__num_vis_sat(
    const int32_t & _arg)
  {
    this->num_vis_sat = _arg;
    return *this;
  }
  Type & set__num_block_sat(
    const int32_t & _arg)
  {
    this->num_block_sat = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    multipath_sim::msg::MultipathOffset_<ContainerAllocator> *;
  using ConstRawPtr =
    const multipath_sim::msg::MultipathOffset_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<multipath_sim::msg::MultipathOffset_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<multipath_sim::msg::MultipathOffset_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      multipath_sim::msg::MultipathOffset_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<multipath_sim::msg::MultipathOffset_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      multipath_sim::msg::MultipathOffset_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<multipath_sim::msg::MultipathOffset_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<multipath_sim::msg::MultipathOffset_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<multipath_sim::msg::MultipathOffset_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__multipath_sim__msg__MultipathOffset
    std::shared_ptr<multipath_sim::msg::MultipathOffset_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__multipath_sim__msg__MultipathOffset
    std::shared_ptr<multipath_sim::msg::MultipathOffset_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MultipathOffset_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->range_offset != other.range_offset) {
      return false;
    }
    if (this->sats_blocked != other.sats_blocked) {
      return false;
    }
    if (this->num_vis_sat != other.num_vis_sat) {
      return false;
    }
    if (this->num_block_sat != other.num_block_sat) {
      return false;
    }
    return true;
  }
  bool operator!=(const MultipathOffset_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MultipathOffset_

// alias to use template instance with default allocator
using MultipathOffset =
  multipath_sim::msg::MultipathOffset_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace multipath_sim

#endif  // MULTIPATH_SIM__MSG__DETAIL__MULTIPATH_OFFSET__STRUCT_HPP_
