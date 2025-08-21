// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from vox_nav_msgs:msg/Object.idl
// generated code does not contain a copyright notice

#ifndef VOX_NAV_MSGS__MSG__DETAIL__OBJECT__STRUCT_HPP_
#define VOX_NAV_MSGS__MSG__DETAIL__OBJECT__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__struct.hpp"
// Member 'twist'
#include "geometry_msgs/msg/detail/twist__struct.hpp"
// Member 'accel'
#include "geometry_msgs/msg/detail/accel__struct.hpp"
// Member 'polygon'
#include "geometry_msgs/msg/detail/polygon__struct.hpp"
// Member 'shape'
#include "shape_msgs/msg/detail/solid_primitive__struct.hpp"
// Member 'cluster'
#include "sensor_msgs/msg/detail/point_cloud2__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__vox_nav_msgs__msg__Object __attribute__((deprecated))
#else
# define DEPRECATED__vox_nav_msgs__msg__Object __declspec(deprecated)
#endif

namespace vox_nav_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Object_
{
  using Type = Object_<ContainerAllocator>;

  explicit Object_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    pose(_init),
    twist(_init),
    accel(_init),
    polygon(_init),
    shape(_init),
    cluster(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0ul;
      this->detection_level = 0;
      this->object_classified = false;
      this->velocity = 0.0f;
      this->heading = 0.0f;
      this->classification_label = 0;
      this->classification_probability = 0.0f;
      this->classification_age = 0ul;
      this->is_dynamic = false;
    }
  }

  explicit Object_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    pose(_alloc, _init),
    twist(_alloc, _init),
    accel(_alloc, _init),
    polygon(_alloc, _init),
    shape(_alloc, _init),
    cluster(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0ul;
      this->detection_level = 0;
      this->object_classified = false;
      this->velocity = 0.0f;
      this->heading = 0.0f;
      this->classification_label = 0;
      this->classification_probability = 0.0f;
      this->classification_age = 0ul;
      this->is_dynamic = false;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _id_type =
    uint32_t;
  _id_type id;
  using _detection_level_type =
    uint8_t;
  _detection_level_type detection_level;
  using _object_classified_type =
    bool;
  _object_classified_type object_classified;
  using _pose_type =
    geometry_msgs::msg::Pose_<ContainerAllocator>;
  _pose_type pose;
  using _twist_type =
    geometry_msgs::msg::Twist_<ContainerAllocator>;
  _twist_type twist;
  using _accel_type =
    geometry_msgs::msg::Accel_<ContainerAllocator>;
  _accel_type accel;
  using _velocity_type =
    float;
  _velocity_type velocity;
  using _heading_type =
    float;
  _heading_type heading;
  using _polygon_type =
    geometry_msgs::msg::Polygon_<ContainerAllocator>;
  _polygon_type polygon;
  using _shape_type =
    shape_msgs::msg::SolidPrimitive_<ContainerAllocator>;
  _shape_type shape;
  using _classification_label_type =
    uint8_t;
  _classification_label_type classification_label;
  using _classification_probability_type =
    float;
  _classification_probability_type classification_probability;
  using _classification_age_type =
    uint32_t;
  _classification_age_type classification_age;
  using _is_dynamic_type =
    bool;
  _is_dynamic_type is_dynamic;
  using _cluster_type =
    sensor_msgs::msg::PointCloud2_<ContainerAllocator>;
  _cluster_type cluster;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__id(
    const uint32_t & _arg)
  {
    this->id = _arg;
    return *this;
  }
  Type & set__detection_level(
    const uint8_t & _arg)
  {
    this->detection_level = _arg;
    return *this;
  }
  Type & set__object_classified(
    const bool & _arg)
  {
    this->object_classified = _arg;
    return *this;
  }
  Type & set__pose(
    const geometry_msgs::msg::Pose_<ContainerAllocator> & _arg)
  {
    this->pose = _arg;
    return *this;
  }
  Type & set__twist(
    const geometry_msgs::msg::Twist_<ContainerAllocator> & _arg)
  {
    this->twist = _arg;
    return *this;
  }
  Type & set__accel(
    const geometry_msgs::msg::Accel_<ContainerAllocator> & _arg)
  {
    this->accel = _arg;
    return *this;
  }
  Type & set__velocity(
    const float & _arg)
  {
    this->velocity = _arg;
    return *this;
  }
  Type & set__heading(
    const float & _arg)
  {
    this->heading = _arg;
    return *this;
  }
  Type & set__polygon(
    const geometry_msgs::msg::Polygon_<ContainerAllocator> & _arg)
  {
    this->polygon = _arg;
    return *this;
  }
  Type & set__shape(
    const shape_msgs::msg::SolidPrimitive_<ContainerAllocator> & _arg)
  {
    this->shape = _arg;
    return *this;
  }
  Type & set__classification_label(
    const uint8_t & _arg)
  {
    this->classification_label = _arg;
    return *this;
  }
  Type & set__classification_probability(
    const float & _arg)
  {
    this->classification_probability = _arg;
    return *this;
  }
  Type & set__classification_age(
    const uint32_t & _arg)
  {
    this->classification_age = _arg;
    return *this;
  }
  Type & set__is_dynamic(
    const bool & _arg)
  {
    this->is_dynamic = _arg;
    return *this;
  }
  Type & set__cluster(
    const sensor_msgs::msg::PointCloud2_<ContainerAllocator> & _arg)
  {
    this->cluster = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t OBJECT_DETECTED =
    0u;
  static constexpr uint8_t OBJECT_TRACKED =
    1u;
  static constexpr uint8_t CLASSIFICATION_UNKNOWN =
    0u;
  static constexpr uint8_t CLASSIFICATION_UNKNOWN_SMALL =
    1u;
  static constexpr uint8_t CLASSIFICATION_UNKNOWN_MEDIUM =
    2u;
  static constexpr uint8_t CLASSIFICATION_UNKNOWN_BIG =
    3u;
  static constexpr uint8_t CLASSIFICATION_PEDESTRIAN =
    4u;
  static constexpr uint8_t CLASSIFICATION_BIKE =
    5u;
  static constexpr uint8_t CLASSIFICATION_CAR =
    6u;
  static constexpr uint8_t CLASSIFICATION_TRUCK =
    7u;
  static constexpr uint8_t CLASSIFICATION_MOTORCYCLE =
    8u;
  static constexpr uint8_t CLASSIFICATION_OTHER_VEHICLE =
    9u;
  static constexpr uint8_t CLASSIFICATION_BARRIER =
    10u;
  static constexpr uint8_t CLASSIFICATION_SIGN =
    11u;
  static constexpr uint8_t CLASSIFICATION_BUS =
    12u;

  // pointer types
  using RawPtr =
    vox_nav_msgs::msg::Object_<ContainerAllocator> *;
  using ConstRawPtr =
    const vox_nav_msgs::msg::Object_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<vox_nav_msgs::msg::Object_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<vox_nav_msgs::msg::Object_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      vox_nav_msgs::msg::Object_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<vox_nav_msgs::msg::Object_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      vox_nav_msgs::msg::Object_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<vox_nav_msgs::msg::Object_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<vox_nav_msgs::msg::Object_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<vox_nav_msgs::msg::Object_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__vox_nav_msgs__msg__Object
    std::shared_ptr<vox_nav_msgs::msg::Object_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__vox_nav_msgs__msg__Object
    std::shared_ptr<vox_nav_msgs::msg::Object_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Object_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->id != other.id) {
      return false;
    }
    if (this->detection_level != other.detection_level) {
      return false;
    }
    if (this->object_classified != other.object_classified) {
      return false;
    }
    if (this->pose != other.pose) {
      return false;
    }
    if (this->twist != other.twist) {
      return false;
    }
    if (this->accel != other.accel) {
      return false;
    }
    if (this->velocity != other.velocity) {
      return false;
    }
    if (this->heading != other.heading) {
      return false;
    }
    if (this->polygon != other.polygon) {
      return false;
    }
    if (this->shape != other.shape) {
      return false;
    }
    if (this->classification_label != other.classification_label) {
      return false;
    }
    if (this->classification_probability != other.classification_probability) {
      return false;
    }
    if (this->classification_age != other.classification_age) {
      return false;
    }
    if (this->is_dynamic != other.is_dynamic) {
      return false;
    }
    if (this->cluster != other.cluster) {
      return false;
    }
    return true;
  }
  bool operator!=(const Object_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Object_

// alias to use template instance with default allocator
using Object =
  vox_nav_msgs::msg::Object_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Object_<ContainerAllocator>::OBJECT_DETECTED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Object_<ContainerAllocator>::OBJECT_TRACKED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Object_<ContainerAllocator>::CLASSIFICATION_UNKNOWN;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Object_<ContainerAllocator>::CLASSIFICATION_UNKNOWN_SMALL;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Object_<ContainerAllocator>::CLASSIFICATION_UNKNOWN_MEDIUM;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Object_<ContainerAllocator>::CLASSIFICATION_UNKNOWN_BIG;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Object_<ContainerAllocator>::CLASSIFICATION_PEDESTRIAN;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Object_<ContainerAllocator>::CLASSIFICATION_BIKE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Object_<ContainerAllocator>::CLASSIFICATION_CAR;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Object_<ContainerAllocator>::CLASSIFICATION_TRUCK;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Object_<ContainerAllocator>::CLASSIFICATION_MOTORCYCLE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Object_<ContainerAllocator>::CLASSIFICATION_OTHER_VEHICLE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Object_<ContainerAllocator>::CLASSIFICATION_BARRIER;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Object_<ContainerAllocator>::CLASSIFICATION_SIGN;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Object_<ContainerAllocator>::CLASSIFICATION_BUS;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace vox_nav_msgs

#endif  // VOX_NAV_MSGS__MSG__DETAIL__OBJECT__STRUCT_HPP_
