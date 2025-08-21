// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from vox_nav_msgs:srv/GetOSMRoadTopologyMap.idl
// generated code does not contain a copyright notice

#ifndef VOX_NAV_MSGS__SRV__DETAIL__GET_OSM_ROAD_TOPOLOGY_MAP__STRUCT_HPP_
#define VOX_NAV_MSGS__SRV__DETAIL__GET_OSM_ROAD_TOPOLOGY_MAP__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__vox_nav_msgs__srv__GetOSMRoadTopologyMap_Request __attribute__((deprecated))
#else
# define DEPRECATED__vox_nav_msgs__srv__GetOSMRoadTopologyMap_Request __declspec(deprecated)
#endif

namespace vox_nav_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetOSMRoadTopologyMap_Request_
{
  using Type = GetOSMRoadTopologyMap_Request_<ContainerAllocator>;

  explicit GetOSMRoadTopologyMap_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit GetOSMRoadTopologyMap_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    vox_nav_msgs::srv::GetOSMRoadTopologyMap_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const vox_nav_msgs::srv::GetOSMRoadTopologyMap_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<vox_nav_msgs::srv::GetOSMRoadTopologyMap_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<vox_nav_msgs::srv::GetOSMRoadTopologyMap_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      vox_nav_msgs::srv::GetOSMRoadTopologyMap_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<vox_nav_msgs::srv::GetOSMRoadTopologyMap_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      vox_nav_msgs::srv::GetOSMRoadTopologyMap_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<vox_nav_msgs::srv::GetOSMRoadTopologyMap_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<vox_nav_msgs::srv::GetOSMRoadTopologyMap_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<vox_nav_msgs::srv::GetOSMRoadTopologyMap_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__vox_nav_msgs__srv__GetOSMRoadTopologyMap_Request
    std::shared_ptr<vox_nav_msgs::srv::GetOSMRoadTopologyMap_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__vox_nav_msgs__srv__GetOSMRoadTopologyMap_Request
    std::shared_ptr<vox_nav_msgs::srv::GetOSMRoadTopologyMap_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetOSMRoadTopologyMap_Request_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetOSMRoadTopologyMap_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetOSMRoadTopologyMap_Request_

// alias to use template instance with default allocator
using GetOSMRoadTopologyMap_Request =
  vox_nav_msgs::srv::GetOSMRoadTopologyMap_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace vox_nav_msgs


// Include directives for member types
// Member 'osm_road_topology'
#include "sensor_msgs/msg/detail/point_cloud2__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__vox_nav_msgs__srv__GetOSMRoadTopologyMap_Response __attribute__((deprecated))
#else
# define DEPRECATED__vox_nav_msgs__srv__GetOSMRoadTopologyMap_Response __declspec(deprecated)
#endif

namespace vox_nav_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetOSMRoadTopologyMap_Response_
{
  using Type = GetOSMRoadTopologyMap_Response_<ContainerAllocator>;

  explicit GetOSMRoadTopologyMap_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : osm_road_topology(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->is_valid = false;
    }
  }

  explicit GetOSMRoadTopologyMap_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : osm_road_topology(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->is_valid = false;
    }
  }

  // field types and members
  using _osm_road_topology_type =
    sensor_msgs::msg::PointCloud2_<ContainerAllocator>;
  _osm_road_topology_type osm_road_topology;
  using _is_valid_type =
    bool;
  _is_valid_type is_valid;

  // setters for named parameter idiom
  Type & set__osm_road_topology(
    const sensor_msgs::msg::PointCloud2_<ContainerAllocator> & _arg)
  {
    this->osm_road_topology = _arg;
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
    vox_nav_msgs::srv::GetOSMRoadTopologyMap_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const vox_nav_msgs::srv::GetOSMRoadTopologyMap_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<vox_nav_msgs::srv::GetOSMRoadTopologyMap_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<vox_nav_msgs::srv::GetOSMRoadTopologyMap_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      vox_nav_msgs::srv::GetOSMRoadTopologyMap_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<vox_nav_msgs::srv::GetOSMRoadTopologyMap_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      vox_nav_msgs::srv::GetOSMRoadTopologyMap_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<vox_nav_msgs::srv::GetOSMRoadTopologyMap_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<vox_nav_msgs::srv::GetOSMRoadTopologyMap_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<vox_nav_msgs::srv::GetOSMRoadTopologyMap_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__vox_nav_msgs__srv__GetOSMRoadTopologyMap_Response
    std::shared_ptr<vox_nav_msgs::srv::GetOSMRoadTopologyMap_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__vox_nav_msgs__srv__GetOSMRoadTopologyMap_Response
    std::shared_ptr<vox_nav_msgs::srv::GetOSMRoadTopologyMap_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetOSMRoadTopologyMap_Response_ & other) const
  {
    if (this->osm_road_topology != other.osm_road_topology) {
      return false;
    }
    if (this->is_valid != other.is_valid) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetOSMRoadTopologyMap_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetOSMRoadTopologyMap_Response_

// alias to use template instance with default allocator
using GetOSMRoadTopologyMap_Response =
  vox_nav_msgs::srv::GetOSMRoadTopologyMap_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace vox_nav_msgs

namespace vox_nav_msgs
{

namespace srv
{

struct GetOSMRoadTopologyMap
{
  using Request = vox_nav_msgs::srv::GetOSMRoadTopologyMap_Request;
  using Response = vox_nav_msgs::srv::GetOSMRoadTopologyMap_Response;
};

}  // namespace srv

}  // namespace vox_nav_msgs

#endif  // VOX_NAV_MSGS__SRV__DETAIL__GET_OSM_ROAD_TOPOLOGY_MAP__STRUCT_HPP_
