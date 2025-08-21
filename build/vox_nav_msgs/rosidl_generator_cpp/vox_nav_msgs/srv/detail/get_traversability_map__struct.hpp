// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from vox_nav_msgs:srv/GetTraversabilityMap.idl
// generated code does not contain a copyright notice

#ifndef VOX_NAV_MSGS__SRV__DETAIL__GET_TRAVERSABILITY_MAP__STRUCT_HPP_
#define VOX_NAV_MSGS__SRV__DETAIL__GET_TRAVERSABILITY_MAP__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__vox_nav_msgs__srv__GetTraversabilityMap_Request __attribute__((deprecated))
#else
# define DEPRECATED__vox_nav_msgs__srv__GetTraversabilityMap_Request __declspec(deprecated)
#endif

namespace vox_nav_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetTraversabilityMap_Request_
{
  using Type = GetTraversabilityMap_Request_<ContainerAllocator>;

  explicit GetTraversabilityMap_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit GetTraversabilityMap_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    vox_nav_msgs::srv::GetTraversabilityMap_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const vox_nav_msgs::srv::GetTraversabilityMap_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<vox_nav_msgs::srv::GetTraversabilityMap_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<vox_nav_msgs::srv::GetTraversabilityMap_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      vox_nav_msgs::srv::GetTraversabilityMap_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<vox_nav_msgs::srv::GetTraversabilityMap_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      vox_nav_msgs::srv::GetTraversabilityMap_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<vox_nav_msgs::srv::GetTraversabilityMap_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<vox_nav_msgs::srv::GetTraversabilityMap_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<vox_nav_msgs::srv::GetTraversabilityMap_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__vox_nav_msgs__srv__GetTraversabilityMap_Request
    std::shared_ptr<vox_nav_msgs::srv::GetTraversabilityMap_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__vox_nav_msgs__srv__GetTraversabilityMap_Request
    std::shared_ptr<vox_nav_msgs::srv::GetTraversabilityMap_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetTraversabilityMap_Request_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetTraversabilityMap_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetTraversabilityMap_Request_

// alias to use template instance with default allocator
using GetTraversabilityMap_Request =
  vox_nav_msgs::srv::GetTraversabilityMap_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace vox_nav_msgs


// Include directives for member types
// Member 'original_octomap'
// Member 'collision_octomap'
// Member 'elevated_surfel_octomap'
#include "octomap_msgs/msg/detail/octomap__struct.hpp"
// Member 'elevated_surfel_poses'
#include "geometry_msgs/msg/detail/pose_array__struct.hpp"
// Member 'traversable_elevated_cloud'
// Member 'traversable_cloud'
#include "sensor_msgs/msg/detail/point_cloud2__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__vox_nav_msgs__srv__GetTraversabilityMap_Response __attribute__((deprecated))
#else
# define DEPRECATED__vox_nav_msgs__srv__GetTraversabilityMap_Response __declspec(deprecated)
#endif

namespace vox_nav_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetTraversabilityMap_Response_
{
  using Type = GetTraversabilityMap_Response_<ContainerAllocator>;

  explicit GetTraversabilityMap_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : original_octomap(_init),
    collision_octomap(_init),
    elevated_surfel_octomap(_init),
    elevated_surfel_poses(_init),
    traversable_elevated_cloud(_init),
    traversable_cloud(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->is_valid = false;
    }
  }

  explicit GetTraversabilityMap_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : original_octomap(_alloc, _init),
    collision_octomap(_alloc, _init),
    elevated_surfel_octomap(_alloc, _init),
    elevated_surfel_poses(_alloc, _init),
    traversable_elevated_cloud(_alloc, _init),
    traversable_cloud(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->is_valid = false;
    }
  }

  // field types and members
  using _original_octomap_type =
    octomap_msgs::msg::Octomap_<ContainerAllocator>;
  _original_octomap_type original_octomap;
  using _collision_octomap_type =
    octomap_msgs::msg::Octomap_<ContainerAllocator>;
  _collision_octomap_type collision_octomap;
  using _elevated_surfel_octomap_type =
    octomap_msgs::msg::Octomap_<ContainerAllocator>;
  _elevated_surfel_octomap_type elevated_surfel_octomap;
  using _elevated_surfel_poses_type =
    geometry_msgs::msg::PoseArray_<ContainerAllocator>;
  _elevated_surfel_poses_type elevated_surfel_poses;
  using _traversable_elevated_cloud_type =
    sensor_msgs::msg::PointCloud2_<ContainerAllocator>;
  _traversable_elevated_cloud_type traversable_elevated_cloud;
  using _traversable_cloud_type =
    sensor_msgs::msg::PointCloud2_<ContainerAllocator>;
  _traversable_cloud_type traversable_cloud;
  using _is_valid_type =
    bool;
  _is_valid_type is_valid;

  // setters for named parameter idiom
  Type & set__original_octomap(
    const octomap_msgs::msg::Octomap_<ContainerAllocator> & _arg)
  {
    this->original_octomap = _arg;
    return *this;
  }
  Type & set__collision_octomap(
    const octomap_msgs::msg::Octomap_<ContainerAllocator> & _arg)
  {
    this->collision_octomap = _arg;
    return *this;
  }
  Type & set__elevated_surfel_octomap(
    const octomap_msgs::msg::Octomap_<ContainerAllocator> & _arg)
  {
    this->elevated_surfel_octomap = _arg;
    return *this;
  }
  Type & set__elevated_surfel_poses(
    const geometry_msgs::msg::PoseArray_<ContainerAllocator> & _arg)
  {
    this->elevated_surfel_poses = _arg;
    return *this;
  }
  Type & set__traversable_elevated_cloud(
    const sensor_msgs::msg::PointCloud2_<ContainerAllocator> & _arg)
  {
    this->traversable_elevated_cloud = _arg;
    return *this;
  }
  Type & set__traversable_cloud(
    const sensor_msgs::msg::PointCloud2_<ContainerAllocator> & _arg)
  {
    this->traversable_cloud = _arg;
    return *this;
  }
  Type & set__is_valid(
    const bool & _arg)
  {
    this->is_valid = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    vox_nav_msgs::srv::GetTraversabilityMap_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const vox_nav_msgs::srv::GetTraversabilityMap_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<vox_nav_msgs::srv::GetTraversabilityMap_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<vox_nav_msgs::srv::GetTraversabilityMap_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      vox_nav_msgs::srv::GetTraversabilityMap_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<vox_nav_msgs::srv::GetTraversabilityMap_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      vox_nav_msgs::srv::GetTraversabilityMap_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<vox_nav_msgs::srv::GetTraversabilityMap_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<vox_nav_msgs::srv::GetTraversabilityMap_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<vox_nav_msgs::srv::GetTraversabilityMap_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__vox_nav_msgs__srv__GetTraversabilityMap_Response
    std::shared_ptr<vox_nav_msgs::srv::GetTraversabilityMap_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__vox_nav_msgs__srv__GetTraversabilityMap_Response
    std::shared_ptr<vox_nav_msgs::srv::GetTraversabilityMap_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetTraversabilityMap_Response_ & other) const
  {
    if (this->original_octomap != other.original_octomap) {
      return false;
    }
    if (this->collision_octomap != other.collision_octomap) {
      return false;
    }
    if (this->elevated_surfel_octomap != other.elevated_surfel_octomap) {
      return false;
    }
    if (this->elevated_surfel_poses != other.elevated_surfel_poses) {
      return false;
    }
    if (this->traversable_elevated_cloud != other.traversable_elevated_cloud) {
      return false;
    }
    if (this->traversable_cloud != other.traversable_cloud) {
      return false;
    }
    if (this->is_valid != other.is_valid) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetTraversabilityMap_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetTraversabilityMap_Response_

// alias to use template instance with default allocator
using GetTraversabilityMap_Response =
  vox_nav_msgs::srv::GetTraversabilityMap_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace vox_nav_msgs

namespace vox_nav_msgs
{

namespace srv
{

struct GetTraversabilityMap
{
  using Request = vox_nav_msgs::srv::GetTraversabilityMap_Request;
  using Response = vox_nav_msgs::srv::GetTraversabilityMap_Response;
};

}  // namespace srv

}  // namespace vox_nav_msgs

#endif  // VOX_NAV_MSGS__SRV__DETAIL__GET_TRAVERSABILITY_MAP__STRUCT_HPP_
