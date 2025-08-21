// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from vox_nav_msgs:srv/GetOctomap.idl
// generated code does not contain a copyright notice

#ifndef VOX_NAV_MSGS__SRV__DETAIL__GET_OCTOMAP__STRUCT_HPP_
#define VOX_NAV_MSGS__SRV__DETAIL__GET_OCTOMAP__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'bounding_box_origin'
// Member 'bounding_box_lengths'
#include "geometry_msgs/msg/detail/point__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__vox_nav_msgs__srv__GetOctomap_Request __attribute__((deprecated))
#else
# define DEPRECATED__vox_nav_msgs__srv__GetOctomap_Request __declspec(deprecated)
#endif

namespace vox_nav_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetOctomap_Request_
{
  using Type = GetOctomap_Request_<ContainerAllocator>;

  explicit GetOctomap_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : bounding_box_origin(_init),
    bounding_box_lengths(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->leaf_size = 0.0;
      this->publish_octomap = false;
      this->filename = "";
    }
  }

  explicit GetOctomap_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : bounding_box_origin(_alloc, _init),
    bounding_box_lengths(_alloc, _init),
    filename(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->leaf_size = 0.0;
      this->publish_octomap = false;
      this->filename = "";
    }
  }

  // field types and members
  using _bounding_box_origin_type =
    geometry_msgs::msg::Point_<ContainerAllocator>;
  _bounding_box_origin_type bounding_box_origin;
  using _bounding_box_lengths_type =
    geometry_msgs::msg::Point_<ContainerAllocator>;
  _bounding_box_lengths_type bounding_box_lengths;
  using _leaf_size_type =
    double;
  _leaf_size_type leaf_size;
  using _publish_octomap_type =
    bool;
  _publish_octomap_type publish_octomap;
  using _filename_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _filename_type filename;

  // setters for named parameter idiom
  Type & set__bounding_box_origin(
    const geometry_msgs::msg::Point_<ContainerAllocator> & _arg)
  {
    this->bounding_box_origin = _arg;
    return *this;
  }
  Type & set__bounding_box_lengths(
    const geometry_msgs::msg::Point_<ContainerAllocator> & _arg)
  {
    this->bounding_box_lengths = _arg;
    return *this;
  }
  Type & set__leaf_size(
    const double & _arg)
  {
    this->leaf_size = _arg;
    return *this;
  }
  Type & set__publish_octomap(
    const bool & _arg)
  {
    this->publish_octomap = _arg;
    return *this;
  }
  Type & set__filename(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->filename = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    vox_nav_msgs::srv::GetOctomap_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const vox_nav_msgs::srv::GetOctomap_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<vox_nav_msgs::srv::GetOctomap_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<vox_nav_msgs::srv::GetOctomap_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      vox_nav_msgs::srv::GetOctomap_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<vox_nav_msgs::srv::GetOctomap_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      vox_nav_msgs::srv::GetOctomap_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<vox_nav_msgs::srv::GetOctomap_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<vox_nav_msgs::srv::GetOctomap_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<vox_nav_msgs::srv::GetOctomap_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__vox_nav_msgs__srv__GetOctomap_Request
    std::shared_ptr<vox_nav_msgs::srv::GetOctomap_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__vox_nav_msgs__srv__GetOctomap_Request
    std::shared_ptr<vox_nav_msgs::srv::GetOctomap_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetOctomap_Request_ & other) const
  {
    if (this->bounding_box_origin != other.bounding_box_origin) {
      return false;
    }
    if (this->bounding_box_lengths != other.bounding_box_lengths) {
      return false;
    }
    if (this->leaf_size != other.leaf_size) {
      return false;
    }
    if (this->publish_octomap != other.publish_octomap) {
      return false;
    }
    if (this->filename != other.filename) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetOctomap_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetOctomap_Request_

// alias to use template instance with default allocator
using GetOctomap_Request =
  vox_nav_msgs::srv::GetOctomap_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace vox_nav_msgs


// Include directives for member types
// Member 'map'
#include "octomap_msgs/msg/detail/octomap__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__vox_nav_msgs__srv__GetOctomap_Response __attribute__((deprecated))
#else
# define DEPRECATED__vox_nav_msgs__srv__GetOctomap_Response __declspec(deprecated)
#endif

namespace vox_nav_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetOctomap_Response_
{
  using Type = GetOctomap_Response_<ContainerAllocator>;

  explicit GetOctomap_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : map(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->origin_latitude = 0.0;
      this->origin_longitude = 0.0;
      this->origin_altitude = 0.0;
    }
  }

  explicit GetOctomap_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : map(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->origin_latitude = 0.0;
      this->origin_longitude = 0.0;
      this->origin_altitude = 0.0;
    }
  }

  // field types and members
  using _map_type =
    octomap_msgs::msg::Octomap_<ContainerAllocator>;
  _map_type map;
  using _origin_latitude_type =
    double;
  _origin_latitude_type origin_latitude;
  using _origin_longitude_type =
    double;
  _origin_longitude_type origin_longitude;
  using _origin_altitude_type =
    double;
  _origin_altitude_type origin_altitude;

  // setters for named parameter idiom
  Type & set__map(
    const octomap_msgs::msg::Octomap_<ContainerAllocator> & _arg)
  {
    this->map = _arg;
    return *this;
  }
  Type & set__origin_latitude(
    const double & _arg)
  {
    this->origin_latitude = _arg;
    return *this;
  }
  Type & set__origin_longitude(
    const double & _arg)
  {
    this->origin_longitude = _arg;
    return *this;
  }
  Type & set__origin_altitude(
    const double & _arg)
  {
    this->origin_altitude = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    vox_nav_msgs::srv::GetOctomap_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const vox_nav_msgs::srv::GetOctomap_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<vox_nav_msgs::srv::GetOctomap_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<vox_nav_msgs::srv::GetOctomap_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      vox_nav_msgs::srv::GetOctomap_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<vox_nav_msgs::srv::GetOctomap_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      vox_nav_msgs::srv::GetOctomap_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<vox_nav_msgs::srv::GetOctomap_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<vox_nav_msgs::srv::GetOctomap_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<vox_nav_msgs::srv::GetOctomap_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__vox_nav_msgs__srv__GetOctomap_Response
    std::shared_ptr<vox_nav_msgs::srv::GetOctomap_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__vox_nav_msgs__srv__GetOctomap_Response
    std::shared_ptr<vox_nav_msgs::srv::GetOctomap_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetOctomap_Response_ & other) const
  {
    if (this->map != other.map) {
      return false;
    }
    if (this->origin_latitude != other.origin_latitude) {
      return false;
    }
    if (this->origin_longitude != other.origin_longitude) {
      return false;
    }
    if (this->origin_altitude != other.origin_altitude) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetOctomap_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetOctomap_Response_

// alias to use template instance with default allocator
using GetOctomap_Response =
  vox_nav_msgs::srv::GetOctomap_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace vox_nav_msgs

namespace vox_nav_msgs
{

namespace srv
{

struct GetOctomap
{
  using Request = vox_nav_msgs::srv::GetOctomap_Request;
  using Response = vox_nav_msgs::srv::GetOctomap_Response;
};

}  // namespace srv

}  // namespace vox_nav_msgs

#endif  // VOX_NAV_MSGS__SRV__DETAIL__GET_OCTOMAP__STRUCT_HPP_
