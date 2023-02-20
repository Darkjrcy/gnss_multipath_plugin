// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from gnss_multipath_plugin:msg/GNSSMultipathFix.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "gnss_multipath_plugin/msg/detail/gnss_multipath_fix__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace gnss_multipath_plugin
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void GNSSMultipathFix_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) gnss_multipath_plugin::msg::GNSSMultipathFix(_init);
}

void GNSSMultipathFix_fini_function(void * message_memory)
{
  auto typed_message = static_cast<gnss_multipath_plugin::msg::GNSSMultipathFix *>(message_memory);
  typed_message->~GNSSMultipathFix();
}

size_t size_function__GNSSMultipathFix__range_offset(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<float> *>(untyped_member);
  return member->size();
}

const void * get_const_function__GNSSMultipathFix__range_offset(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<float> *>(untyped_member);
  return &member[index];
}

void * get_function__GNSSMultipathFix__range_offset(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<float> *>(untyped_member);
  return &member[index];
}

void resize_function__GNSSMultipathFix__range_offset(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<float> *>(untyped_member);
  member->resize(size);
}

size_t size_function__GNSSMultipathFix__sats_blocked(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<int32_t> *>(untyped_member);
  return member->size();
}

const void * get_const_function__GNSSMultipathFix__sats_blocked(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<int32_t> *>(untyped_member);
  return &member[index];
}

void * get_function__GNSSMultipathFix__sats_blocked(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<int32_t> *>(untyped_member);
  return &member[index];
}

void resize_function__GNSSMultipathFix__sats_blocked(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<int32_t> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember GNSSMultipathFix_message_member_array[6] = {
  {
    "header",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Header>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(gnss_multipath_plugin::msg::GNSSMultipathFix, header),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "navsatfix",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<sensor_msgs::msg::NavSatFix>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(gnss_multipath_plugin::msg::GNSSMultipathFix, navsatfix),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "range_offset",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(gnss_multipath_plugin::msg::GNSSMultipathFix, range_offset),  // bytes offset in struct
    nullptr,  // default value
    size_function__GNSSMultipathFix__range_offset,  // size() function pointer
    get_const_function__GNSSMultipathFix__range_offset,  // get_const(index) function pointer
    get_function__GNSSMultipathFix__range_offset,  // get(index) function pointer
    resize_function__GNSSMultipathFix__range_offset  // resize(index) function pointer
  },
  {
    "sats_blocked",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(gnss_multipath_plugin::msg::GNSSMultipathFix, sats_blocked),  // bytes offset in struct
    nullptr,  // default value
    size_function__GNSSMultipathFix__sats_blocked,  // size() function pointer
    get_const_function__GNSSMultipathFix__sats_blocked,  // get_const(index) function pointer
    get_function__GNSSMultipathFix__sats_blocked,  // get(index) function pointer
    resize_function__GNSSMultipathFix__sats_blocked  // resize(index) function pointer
  },
  {
    "num_vis_sat",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(gnss_multipath_plugin::msg::GNSSMultipathFix, num_vis_sat),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "num_block_sat",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(gnss_multipath_plugin::msg::GNSSMultipathFix, num_block_sat),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers GNSSMultipathFix_message_members = {
  "gnss_multipath_plugin::msg",  // message namespace
  "GNSSMultipathFix",  // message name
  6,  // number of fields
  sizeof(gnss_multipath_plugin::msg::GNSSMultipathFix),
  GNSSMultipathFix_message_member_array,  // message members
  GNSSMultipathFix_init_function,  // function to initialize message memory (memory has to be allocated)
  GNSSMultipathFix_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t GNSSMultipathFix_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &GNSSMultipathFix_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace gnss_multipath_plugin


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<gnss_multipath_plugin::msg::GNSSMultipathFix>()
{
  return &::gnss_multipath_plugin::msg::rosidl_typesupport_introspection_cpp::GNSSMultipathFix_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, gnss_multipath_plugin, msg, GNSSMultipathFix)() {
  return &::gnss_multipath_plugin::msg::rosidl_typesupport_introspection_cpp::GNSSMultipathFix_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
