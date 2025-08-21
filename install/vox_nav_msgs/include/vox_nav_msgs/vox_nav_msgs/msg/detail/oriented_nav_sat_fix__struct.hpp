// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from vox_nav_msgs:msg/OrientedNavSatFix.idl
// generated code does not contain a copyright notice

#ifndef VOX_NAV_MSGS__MSG__DETAIL__ORIENTED_NAV_SAT_FIX__STRUCT_HPP_
#define VOX_NAV_MSGS__MSG__DETAIL__ORIENTED_NAV_SAT_FIX__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'position'
#include "sensor_msgs/msg/detail/nav_sat_fix__struct.hpp"
// Member 'orientation'
#include "geometry_msgs/msg/detail/quaternion__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__vox_nav_msgs__msg__OrientedNavSatFix __attribute__((deprecated))
#else
# define DEPRECATED__vox_nav_msgs__msg__OrientedNavSatFix __declspec(deprecated)
#endif

namespace vox_nav_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct OrientedNavSatFix_
{
  using Type = OrientedNavSatFix_<ContainerAllocator>;

  explicit OrientedNavSatFix_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : position(_init),
    orientation(_init)
  {
    (void)_init;
  }

  explicit OrientedNavSatFix_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : position(_alloc, _init),
    orientation(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _position_type =
    sensor_msgs::msg::NavSatFix_<ContainerAllocator>;
  _position_type position;
  using _orientation_type =
    geometry_msgs::msg::Quaternion_<ContainerAllocator>;
  _orientation_type orientation;

  // setters for named parameter idiom
  Type & set__position(
    const sensor_msgs::msg::NavSatFix_<ContainerAllocator> & _arg)
  {
    this->position = _arg;
    return *this;
  }
  Type & set__orientation(
    const geometry_msgs::msg::Quaternion_<ContainerAllocator> & _arg)
  {
    this->orientation = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    vox_nav_msgs::msg::OrientedNavSatFix_<ContainerAllocator> *;
  using ConstRawPtr =
    const vox_nav_msgs::msg::OrientedNavSatFix_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<vox_nav_msgs::msg::OrientedNavSatFix_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<vox_nav_msgs::msg::OrientedNavSatFix_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      vox_nav_msgs::msg::OrientedNavSatFix_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<vox_nav_msgs::msg::OrientedNavSatFix_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      vox_nav_msgs::msg::OrientedNavSatFix_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<vox_nav_msgs::msg::OrientedNavSatFix_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<vox_nav_msgs::msg::OrientedNavSatFix_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<vox_nav_msgs::msg::OrientedNavSatFix_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__vox_nav_msgs__msg__OrientedNavSatFix
    std::shared_ptr<vox_nav_msgs::msg::OrientedNavSatFix_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__vox_nav_msgs__msg__OrientedNavSatFix
    std::shared_ptr<vox_nav_msgs::msg::OrientedNavSatFix_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const OrientedNavSatFix_ & other) const
  {
    if (this->position != other.position) {
      return false;
    }
    if (this->orientation != other.orientation) {
      return false;
    }
    return true;
  }
  bool operator!=(const OrientedNavSatFix_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct OrientedNavSatFix_

// alias to use template instance with default allocator
using OrientedNavSatFix =
  vox_nav_msgs::msg::OrientedNavSatFix_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace vox_nav_msgs

#endif  // VOX_NAV_MSGS__MSG__DETAIL__ORIENTED_NAV_SAT_FIX__STRUCT_HPP_
