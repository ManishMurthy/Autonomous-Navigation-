// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from vox_nav_msgs:msg/SemanticRoadLabel.idl
// generated code does not contain a copyright notice

#ifndef VOX_NAV_MSGS__MSG__DETAIL__SEMANTIC_ROAD_LABEL__STRUCT_HPP_
#define VOX_NAV_MSGS__MSG__DETAIL__SEMANTIC_ROAD_LABEL__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__vox_nav_msgs__msg__SemanticRoadLabel __attribute__((deprecated))
#else
# define DEPRECATED__vox_nav_msgs__msg__SemanticRoadLabel __declspec(deprecated)
#endif

namespace vox_nav_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct SemanticRoadLabel_
{
  using Type = SemanticRoadLabel_<ContainerAllocator>;

  explicit SemanticRoadLabel_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit SemanticRoadLabel_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
  static const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> MAJOR_ROADS_WHITE;
  static const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> MISC_ORANGE;
  static const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> MINOR_ROAD_MAROON;
  static const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> FOOTWAY_OLIVE;
  static const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> TRACK_NAVY;

  // pointer types
  using RawPtr =
    vox_nav_msgs::msg::SemanticRoadLabel_<ContainerAllocator> *;
  using ConstRawPtr =
    const vox_nav_msgs::msg::SemanticRoadLabel_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<vox_nav_msgs::msg::SemanticRoadLabel_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<vox_nav_msgs::msg::SemanticRoadLabel_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      vox_nav_msgs::msg::SemanticRoadLabel_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<vox_nav_msgs::msg::SemanticRoadLabel_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      vox_nav_msgs::msg::SemanticRoadLabel_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<vox_nav_msgs::msg::SemanticRoadLabel_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<vox_nav_msgs::msg::SemanticRoadLabel_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<vox_nav_msgs::msg::SemanticRoadLabel_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__vox_nav_msgs__msg__SemanticRoadLabel
    std::shared_ptr<vox_nav_msgs::msg::SemanticRoadLabel_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__vox_nav_msgs__msg__SemanticRoadLabel
    std::shared_ptr<vox_nav_msgs::msg::SemanticRoadLabel_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SemanticRoadLabel_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const SemanticRoadLabel_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SemanticRoadLabel_

// alias to use template instance with default allocator
using SemanticRoadLabel =
  vox_nav_msgs::msg::SemanticRoadLabel_<std::allocator<void>>;

// constant definitions
template<typename ContainerAllocator>
const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>
SemanticRoadLabel_<ContainerAllocator>::MAJOR_ROADS_WHITE = "255255255";
template<typename ContainerAllocator>
const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>
SemanticRoadLabel_<ContainerAllocator>::MISC_ORANGE = "255127000";
template<typename ContainerAllocator>
const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>
SemanticRoadLabel_<ContainerAllocator>::MINOR_ROAD_MAROON = "127000000";
template<typename ContainerAllocator>
const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>
SemanticRoadLabel_<ContainerAllocator>::FOOTWAY_OLIVE = "127127000";
template<typename ContainerAllocator>
const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>
SemanticRoadLabel_<ContainerAllocator>::TRACK_NAVY = "000000127";

}  // namespace msg

}  // namespace vox_nav_msgs

#endif  // VOX_NAV_MSGS__MSG__DETAIL__SEMANTIC_ROAD_LABEL__STRUCT_HPP_
