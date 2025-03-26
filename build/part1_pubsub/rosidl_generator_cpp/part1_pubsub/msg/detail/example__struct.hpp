// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from part1_pubsub:msg/Example.idl
// generated code does not contain a copyright notice

#ifndef PART1_PUBSUB__MSG__DETAIL__EXAMPLE__STRUCT_HPP_
#define PART1_PUBSUB__MSG__DETAIL__EXAMPLE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__part1_pubsub__msg__Example __attribute__((deprecated))
#else
# define DEPRECATED__part1_pubsub__msg__Example __declspec(deprecated)
#endif

namespace part1_pubsub
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Example_
{
  using Type = Example_<ContainerAllocator>;

  explicit Example_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit Example_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  // field types and members
  using _structure_needs_at_least_one_member_type =
    uint8_t;
  _structure_needs_at_least_one_member_type structure_needs_at_least_one_member;


  // constant declarations

  // pointer types
  using RawPtr =
    part1_pubsub::msg::Example_<ContainerAllocator> *;
  using ConstRawPtr =
    const part1_pubsub::msg::Example_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<part1_pubsub::msg::Example_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<part1_pubsub::msg::Example_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      part1_pubsub::msg::Example_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<part1_pubsub::msg::Example_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      part1_pubsub::msg::Example_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<part1_pubsub::msg::Example_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<part1_pubsub::msg::Example_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<part1_pubsub::msg::Example_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__part1_pubsub__msg__Example
    std::shared_ptr<part1_pubsub::msg::Example_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__part1_pubsub__msg__Example
    std::shared_ptr<part1_pubsub::msg::Example_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Example_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const Example_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Example_

// alias to use template instance with default allocator
using Example =
  part1_pubsub::msg::Example_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace part1_pubsub

#endif  // PART1_PUBSUB__MSG__DETAIL__EXAMPLE__STRUCT_HPP_
