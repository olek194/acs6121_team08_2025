// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from part1_pubsub:msg/Example.idl
// generated code does not contain a copyright notice

#ifndef PART1_PUBSUB__MSG__DETAIL__EXAMPLE__BUILDER_HPP_
#define PART1_PUBSUB__MSG__DETAIL__EXAMPLE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "part1_pubsub/msg/detail/example__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace part1_pubsub
{

namespace msg
{


}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::part1_pubsub::msg::Example>()
{
  return ::part1_pubsub::msg::Example(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace part1_pubsub

#endif  // PART1_PUBSUB__MSG__DETAIL__EXAMPLE__BUILDER_HPP_
