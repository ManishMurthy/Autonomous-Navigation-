// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from vox_nav_msgs:msg/Object.idl
// generated code does not contain a copyright notice

#ifndef VOX_NAV_MSGS__MSG__DETAIL__OBJECT__BUILDER_HPP_
#define VOX_NAV_MSGS__MSG__DETAIL__OBJECT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "vox_nav_msgs/msg/detail/object__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace vox_nav_msgs
{

namespace msg
{

namespace builder
{

class Init_Object_cluster
{
public:
  explicit Init_Object_cluster(::vox_nav_msgs::msg::Object & msg)
  : msg_(msg)
  {}
  ::vox_nav_msgs::msg::Object cluster(::vox_nav_msgs::msg::Object::_cluster_type arg)
  {
    msg_.cluster = std::move(arg);
    return std::move(msg_);
  }

private:
  ::vox_nav_msgs::msg::Object msg_;
};

class Init_Object_is_dynamic
{
public:
  explicit Init_Object_is_dynamic(::vox_nav_msgs::msg::Object & msg)
  : msg_(msg)
  {}
  Init_Object_cluster is_dynamic(::vox_nav_msgs::msg::Object::_is_dynamic_type arg)
  {
    msg_.is_dynamic = std::move(arg);
    return Init_Object_cluster(msg_);
  }

private:
  ::vox_nav_msgs::msg::Object msg_;
};

class Init_Object_classification_age
{
public:
  explicit Init_Object_classification_age(::vox_nav_msgs::msg::Object & msg)
  : msg_(msg)
  {}
  Init_Object_is_dynamic classification_age(::vox_nav_msgs::msg::Object::_classification_age_type arg)
  {
    msg_.classification_age = std::move(arg);
    return Init_Object_is_dynamic(msg_);
  }

private:
  ::vox_nav_msgs::msg::Object msg_;
};

class Init_Object_classification_probability
{
public:
  explicit Init_Object_classification_probability(::vox_nav_msgs::msg::Object & msg)
  : msg_(msg)
  {}
  Init_Object_classification_age classification_probability(::vox_nav_msgs::msg::Object::_classification_probability_type arg)
  {
    msg_.classification_probability = std::move(arg);
    return Init_Object_classification_age(msg_);
  }

private:
  ::vox_nav_msgs::msg::Object msg_;
};

class Init_Object_classification_label
{
public:
  explicit Init_Object_classification_label(::vox_nav_msgs::msg::Object & msg)
  : msg_(msg)
  {}
  Init_Object_classification_probability classification_label(::vox_nav_msgs::msg::Object::_classification_label_type arg)
  {
    msg_.classification_label = std::move(arg);
    return Init_Object_classification_probability(msg_);
  }

private:
  ::vox_nav_msgs::msg::Object msg_;
};

class Init_Object_shape
{
public:
  explicit Init_Object_shape(::vox_nav_msgs::msg::Object & msg)
  : msg_(msg)
  {}
  Init_Object_classification_label shape(::vox_nav_msgs::msg::Object::_shape_type arg)
  {
    msg_.shape = std::move(arg);
    return Init_Object_classification_label(msg_);
  }

private:
  ::vox_nav_msgs::msg::Object msg_;
};

class Init_Object_polygon
{
public:
  explicit Init_Object_polygon(::vox_nav_msgs::msg::Object & msg)
  : msg_(msg)
  {}
  Init_Object_shape polygon(::vox_nav_msgs::msg::Object::_polygon_type arg)
  {
    msg_.polygon = std::move(arg);
    return Init_Object_shape(msg_);
  }

private:
  ::vox_nav_msgs::msg::Object msg_;
};

class Init_Object_heading
{
public:
  explicit Init_Object_heading(::vox_nav_msgs::msg::Object & msg)
  : msg_(msg)
  {}
  Init_Object_polygon heading(::vox_nav_msgs::msg::Object::_heading_type arg)
  {
    msg_.heading = std::move(arg);
    return Init_Object_polygon(msg_);
  }

private:
  ::vox_nav_msgs::msg::Object msg_;
};

class Init_Object_velocity
{
public:
  explicit Init_Object_velocity(::vox_nav_msgs::msg::Object & msg)
  : msg_(msg)
  {}
  Init_Object_heading velocity(::vox_nav_msgs::msg::Object::_velocity_type arg)
  {
    msg_.velocity = std::move(arg);
    return Init_Object_heading(msg_);
  }

private:
  ::vox_nav_msgs::msg::Object msg_;
};

class Init_Object_accel
{
public:
  explicit Init_Object_accel(::vox_nav_msgs::msg::Object & msg)
  : msg_(msg)
  {}
  Init_Object_velocity accel(::vox_nav_msgs::msg::Object::_accel_type arg)
  {
    msg_.accel = std::move(arg);
    return Init_Object_velocity(msg_);
  }

private:
  ::vox_nav_msgs::msg::Object msg_;
};

class Init_Object_twist
{
public:
  explicit Init_Object_twist(::vox_nav_msgs::msg::Object & msg)
  : msg_(msg)
  {}
  Init_Object_accel twist(::vox_nav_msgs::msg::Object::_twist_type arg)
  {
    msg_.twist = std::move(arg);
    return Init_Object_accel(msg_);
  }

private:
  ::vox_nav_msgs::msg::Object msg_;
};

class Init_Object_pose
{
public:
  explicit Init_Object_pose(::vox_nav_msgs::msg::Object & msg)
  : msg_(msg)
  {}
  Init_Object_twist pose(::vox_nav_msgs::msg::Object::_pose_type arg)
  {
    msg_.pose = std::move(arg);
    return Init_Object_twist(msg_);
  }

private:
  ::vox_nav_msgs::msg::Object msg_;
};

class Init_Object_object_classified
{
public:
  explicit Init_Object_object_classified(::vox_nav_msgs::msg::Object & msg)
  : msg_(msg)
  {}
  Init_Object_pose object_classified(::vox_nav_msgs::msg::Object::_object_classified_type arg)
  {
    msg_.object_classified = std::move(arg);
    return Init_Object_pose(msg_);
  }

private:
  ::vox_nav_msgs::msg::Object msg_;
};

class Init_Object_detection_level
{
public:
  explicit Init_Object_detection_level(::vox_nav_msgs::msg::Object & msg)
  : msg_(msg)
  {}
  Init_Object_object_classified detection_level(::vox_nav_msgs::msg::Object::_detection_level_type arg)
  {
    msg_.detection_level = std::move(arg);
    return Init_Object_object_classified(msg_);
  }

private:
  ::vox_nav_msgs::msg::Object msg_;
};

class Init_Object_id
{
public:
  explicit Init_Object_id(::vox_nav_msgs::msg::Object & msg)
  : msg_(msg)
  {}
  Init_Object_detection_level id(::vox_nav_msgs::msg::Object::_id_type arg)
  {
    msg_.id = std::move(arg);
    return Init_Object_detection_level(msg_);
  }

private:
  ::vox_nav_msgs::msg::Object msg_;
};

class Init_Object_header
{
public:
  Init_Object_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Object_id header(::vox_nav_msgs::msg::Object::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Object_id(msg_);
  }

private:
  ::vox_nav_msgs::msg::Object msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::vox_nav_msgs::msg::Object>()
{
  return vox_nav_msgs::msg::builder::Init_Object_header();
}

}  // namespace vox_nav_msgs

#endif  // VOX_NAV_MSGS__MSG__DETAIL__OBJECT__BUILDER_HPP_
