// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from vox_nav_msgs:action/NavigateThroughGPSPoses.idl
// generated code does not contain a copyright notice

#ifndef VOX_NAV_MSGS__ACTION__DETAIL__NAVIGATE_THROUGH_GPS_POSES__TRAITS_HPP_
#define VOX_NAV_MSGS__ACTION__DETAIL__NAVIGATE_THROUGH_GPS_POSES__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "vox_nav_msgs/action/detail/navigate_through_gps_poses__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'gps_poses'
#include "sensor_msgs/msg/detail/nav_sat_fix__traits.hpp"

namespace vox_nav_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const NavigateThroughGPSPoses_Goal & msg,
  std::ostream & out)
{
  out << "{";
  // member: gps_poses
  {
    if (msg.gps_poses.size() == 0) {
      out << "gps_poses: []";
    } else {
      out << "gps_poses: [";
      size_t pending_items = msg.gps_poses.size();
      for (auto item : msg.gps_poses) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const NavigateThroughGPSPoses_Goal & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: gps_poses
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.gps_poses.size() == 0) {
      out << "gps_poses: []\n";
    } else {
      out << "gps_poses:\n";
      for (auto item : msg.gps_poses) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const NavigateThroughGPSPoses_Goal & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace vox_nav_msgs

namespace rosidl_generator_traits
{

[[deprecated("use vox_nav_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const vox_nav_msgs::action::NavigateThroughGPSPoses_Goal & msg,
  std::ostream & out, size_t indentation = 0)
{
  vox_nav_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use vox_nav_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const vox_nav_msgs::action::NavigateThroughGPSPoses_Goal & msg)
{
  return vox_nav_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<vox_nav_msgs::action::NavigateThroughGPSPoses_Goal>()
{
  return "vox_nav_msgs::action::NavigateThroughGPSPoses_Goal";
}

template<>
inline const char * name<vox_nav_msgs::action::NavigateThroughGPSPoses_Goal>()
{
  return "vox_nav_msgs/action/NavigateThroughGPSPoses_Goal";
}

template<>
struct has_fixed_size<vox_nav_msgs::action::NavigateThroughGPSPoses_Goal>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<vox_nav_msgs::action::NavigateThroughGPSPoses_Goal>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<vox_nav_msgs::action::NavigateThroughGPSPoses_Goal>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'result'
#include "std_msgs/msg/detail/empty__traits.hpp"

namespace vox_nav_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const NavigateThroughGPSPoses_Result & msg,
  std::ostream & out)
{
  out << "{";
  // member: result
  {
    out << "result: ";
    to_flow_style_yaml(msg.result, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const NavigateThroughGPSPoses_Result & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: result
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "result:\n";
    to_block_style_yaml(msg.result, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const NavigateThroughGPSPoses_Result & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace vox_nav_msgs

namespace rosidl_generator_traits
{

[[deprecated("use vox_nav_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const vox_nav_msgs::action::NavigateThroughGPSPoses_Result & msg,
  std::ostream & out, size_t indentation = 0)
{
  vox_nav_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use vox_nav_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const vox_nav_msgs::action::NavigateThroughGPSPoses_Result & msg)
{
  return vox_nav_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<vox_nav_msgs::action::NavigateThroughGPSPoses_Result>()
{
  return "vox_nav_msgs::action::NavigateThroughGPSPoses_Result";
}

template<>
inline const char * name<vox_nav_msgs::action::NavigateThroughGPSPoses_Result>()
{
  return "vox_nav_msgs/action/NavigateThroughGPSPoses_Result";
}

template<>
struct has_fixed_size<vox_nav_msgs::action::NavigateThroughGPSPoses_Result>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Empty>::value> {};

template<>
struct has_bounded_size<vox_nav_msgs::action::NavigateThroughGPSPoses_Result>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Empty>::value> {};

template<>
struct is_message<vox_nav_msgs::action::NavigateThroughGPSPoses_Result>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'current_pose'
#include "geometry_msgs/msg/detail/pose_stamped__traits.hpp"
// Member 'navigation_time'
#include "builtin_interfaces/msg/detail/duration__traits.hpp"

namespace vox_nav_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const NavigateThroughGPSPoses_Feedback & msg,
  std::ostream & out)
{
  out << "{";
  // member: current_pose
  {
    out << "current_pose: ";
    to_flow_style_yaml(msg.current_pose, out);
    out << ", ";
  }

  // member: navigation_time
  {
    out << "navigation_time: ";
    to_flow_style_yaml(msg.navigation_time, out);
    out << ", ";
  }

  // member: distance_remaining
  {
    out << "distance_remaining: ";
    rosidl_generator_traits::value_to_yaml(msg.distance_remaining, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const NavigateThroughGPSPoses_Feedback & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: current_pose
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current_pose:\n";
    to_block_style_yaml(msg.current_pose, out, indentation + 2);
  }

  // member: navigation_time
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "navigation_time:\n";
    to_block_style_yaml(msg.navigation_time, out, indentation + 2);
  }

  // member: distance_remaining
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "distance_remaining: ";
    rosidl_generator_traits::value_to_yaml(msg.distance_remaining, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const NavigateThroughGPSPoses_Feedback & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace vox_nav_msgs

namespace rosidl_generator_traits
{

[[deprecated("use vox_nav_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const vox_nav_msgs::action::NavigateThroughGPSPoses_Feedback & msg,
  std::ostream & out, size_t indentation = 0)
{
  vox_nav_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use vox_nav_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const vox_nav_msgs::action::NavigateThroughGPSPoses_Feedback & msg)
{
  return vox_nav_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<vox_nav_msgs::action::NavigateThroughGPSPoses_Feedback>()
{
  return "vox_nav_msgs::action::NavigateThroughGPSPoses_Feedback";
}

template<>
inline const char * name<vox_nav_msgs::action::NavigateThroughGPSPoses_Feedback>()
{
  return "vox_nav_msgs/action/NavigateThroughGPSPoses_Feedback";
}

template<>
struct has_fixed_size<vox_nav_msgs::action::NavigateThroughGPSPoses_Feedback>
  : std::integral_constant<bool, has_fixed_size<builtin_interfaces::msg::Duration>::value && has_fixed_size<geometry_msgs::msg::PoseStamped>::value> {};

template<>
struct has_bounded_size<vox_nav_msgs::action::NavigateThroughGPSPoses_Feedback>
  : std::integral_constant<bool, has_bounded_size<builtin_interfaces::msg::Duration>::value && has_bounded_size<geometry_msgs::msg::PoseStamped>::value> {};

template<>
struct is_message<vox_nav_msgs::action::NavigateThroughGPSPoses_Feedback>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"
// Member 'goal'
#include "vox_nav_msgs/action/detail/navigate_through_gps_poses__traits.hpp"

namespace vox_nav_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const NavigateThroughGPSPoses_SendGoal_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
    out << ", ";
  }

  // member: goal
  {
    out << "goal: ";
    to_flow_style_yaml(msg.goal, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const NavigateThroughGPSPoses_SendGoal_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }

  // member: goal
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal:\n";
    to_block_style_yaml(msg.goal, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const NavigateThroughGPSPoses_SendGoal_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace vox_nav_msgs

namespace rosidl_generator_traits
{

[[deprecated("use vox_nav_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const vox_nav_msgs::action::NavigateThroughGPSPoses_SendGoal_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  vox_nav_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use vox_nav_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const vox_nav_msgs::action::NavigateThroughGPSPoses_SendGoal_Request & msg)
{
  return vox_nav_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<vox_nav_msgs::action::NavigateThroughGPSPoses_SendGoal_Request>()
{
  return "vox_nav_msgs::action::NavigateThroughGPSPoses_SendGoal_Request";
}

template<>
inline const char * name<vox_nav_msgs::action::NavigateThroughGPSPoses_SendGoal_Request>()
{
  return "vox_nav_msgs/action/NavigateThroughGPSPoses_SendGoal_Request";
}

template<>
struct has_fixed_size<vox_nav_msgs::action::NavigateThroughGPSPoses_SendGoal_Request>
  : std::integral_constant<bool, has_fixed_size<unique_identifier_msgs::msg::UUID>::value && has_fixed_size<vox_nav_msgs::action::NavigateThroughGPSPoses_Goal>::value> {};

template<>
struct has_bounded_size<vox_nav_msgs::action::NavigateThroughGPSPoses_SendGoal_Request>
  : std::integral_constant<bool, has_bounded_size<unique_identifier_msgs::msg::UUID>::value && has_bounded_size<vox_nav_msgs::action::NavigateThroughGPSPoses_Goal>::value> {};

template<>
struct is_message<vox_nav_msgs::action::NavigateThroughGPSPoses_SendGoal_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace vox_nav_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const NavigateThroughGPSPoses_SendGoal_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: accepted
  {
    out << "accepted: ";
    rosidl_generator_traits::value_to_yaml(msg.accepted, out);
    out << ", ";
  }

  // member: stamp
  {
    out << "stamp: ";
    to_flow_style_yaml(msg.stamp, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const NavigateThroughGPSPoses_SendGoal_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: accepted
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "accepted: ";
    rosidl_generator_traits::value_to_yaml(msg.accepted, out);
    out << "\n";
  }

  // member: stamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "stamp:\n";
    to_block_style_yaml(msg.stamp, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const NavigateThroughGPSPoses_SendGoal_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace vox_nav_msgs

namespace rosidl_generator_traits
{

[[deprecated("use vox_nav_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const vox_nav_msgs::action::NavigateThroughGPSPoses_SendGoal_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  vox_nav_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use vox_nav_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const vox_nav_msgs::action::NavigateThroughGPSPoses_SendGoal_Response & msg)
{
  return vox_nav_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<vox_nav_msgs::action::NavigateThroughGPSPoses_SendGoal_Response>()
{
  return "vox_nav_msgs::action::NavigateThroughGPSPoses_SendGoal_Response";
}

template<>
inline const char * name<vox_nav_msgs::action::NavigateThroughGPSPoses_SendGoal_Response>()
{
  return "vox_nav_msgs/action/NavigateThroughGPSPoses_SendGoal_Response";
}

template<>
struct has_fixed_size<vox_nav_msgs::action::NavigateThroughGPSPoses_SendGoal_Response>
  : std::integral_constant<bool, has_fixed_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct has_bounded_size<vox_nav_msgs::action::NavigateThroughGPSPoses_SendGoal_Response>
  : std::integral_constant<bool, has_bounded_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct is_message<vox_nav_msgs::action::NavigateThroughGPSPoses_SendGoal_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<vox_nav_msgs::action::NavigateThroughGPSPoses_SendGoal>()
{
  return "vox_nav_msgs::action::NavigateThroughGPSPoses_SendGoal";
}

template<>
inline const char * name<vox_nav_msgs::action::NavigateThroughGPSPoses_SendGoal>()
{
  return "vox_nav_msgs/action/NavigateThroughGPSPoses_SendGoal";
}

template<>
struct has_fixed_size<vox_nav_msgs::action::NavigateThroughGPSPoses_SendGoal>
  : std::integral_constant<
    bool,
    has_fixed_size<vox_nav_msgs::action::NavigateThroughGPSPoses_SendGoal_Request>::value &&
    has_fixed_size<vox_nav_msgs::action::NavigateThroughGPSPoses_SendGoal_Response>::value
  >
{
};

template<>
struct has_bounded_size<vox_nav_msgs::action::NavigateThroughGPSPoses_SendGoal>
  : std::integral_constant<
    bool,
    has_bounded_size<vox_nav_msgs::action::NavigateThroughGPSPoses_SendGoal_Request>::value &&
    has_bounded_size<vox_nav_msgs::action::NavigateThroughGPSPoses_SendGoal_Response>::value
  >
{
};

template<>
struct is_service<vox_nav_msgs::action::NavigateThroughGPSPoses_SendGoal>
  : std::true_type
{
};

template<>
struct is_service_request<vox_nav_msgs::action::NavigateThroughGPSPoses_SendGoal_Request>
  : std::true_type
{
};

template<>
struct is_service_response<vox_nav_msgs::action::NavigateThroughGPSPoses_SendGoal_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"

namespace vox_nav_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const NavigateThroughGPSPoses_GetResult_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const NavigateThroughGPSPoses_GetResult_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const NavigateThroughGPSPoses_GetResult_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace vox_nav_msgs

namespace rosidl_generator_traits
{

[[deprecated("use vox_nav_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const vox_nav_msgs::action::NavigateThroughGPSPoses_GetResult_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  vox_nav_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use vox_nav_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const vox_nav_msgs::action::NavigateThroughGPSPoses_GetResult_Request & msg)
{
  return vox_nav_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<vox_nav_msgs::action::NavigateThroughGPSPoses_GetResult_Request>()
{
  return "vox_nav_msgs::action::NavigateThroughGPSPoses_GetResult_Request";
}

template<>
inline const char * name<vox_nav_msgs::action::NavigateThroughGPSPoses_GetResult_Request>()
{
  return "vox_nav_msgs/action/NavigateThroughGPSPoses_GetResult_Request";
}

template<>
struct has_fixed_size<vox_nav_msgs::action::NavigateThroughGPSPoses_GetResult_Request>
  : std::integral_constant<bool, has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<vox_nav_msgs::action::NavigateThroughGPSPoses_GetResult_Request>
  : std::integral_constant<bool, has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<vox_nav_msgs::action::NavigateThroughGPSPoses_GetResult_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'result'
// already included above
// #include "vox_nav_msgs/action/detail/navigate_through_gps_poses__traits.hpp"

namespace vox_nav_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const NavigateThroughGPSPoses_GetResult_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: status
  {
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << ", ";
  }

  // member: result
  {
    out << "result: ";
    to_flow_style_yaml(msg.result, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const NavigateThroughGPSPoses_GetResult_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << "\n";
  }

  // member: result
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "result:\n";
    to_block_style_yaml(msg.result, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const NavigateThroughGPSPoses_GetResult_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace vox_nav_msgs

namespace rosidl_generator_traits
{

[[deprecated("use vox_nav_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const vox_nav_msgs::action::NavigateThroughGPSPoses_GetResult_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  vox_nav_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use vox_nav_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const vox_nav_msgs::action::NavigateThroughGPSPoses_GetResult_Response & msg)
{
  return vox_nav_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<vox_nav_msgs::action::NavigateThroughGPSPoses_GetResult_Response>()
{
  return "vox_nav_msgs::action::NavigateThroughGPSPoses_GetResult_Response";
}

template<>
inline const char * name<vox_nav_msgs::action::NavigateThroughGPSPoses_GetResult_Response>()
{
  return "vox_nav_msgs/action/NavigateThroughGPSPoses_GetResult_Response";
}

template<>
struct has_fixed_size<vox_nav_msgs::action::NavigateThroughGPSPoses_GetResult_Response>
  : std::integral_constant<bool, has_fixed_size<vox_nav_msgs::action::NavigateThroughGPSPoses_Result>::value> {};

template<>
struct has_bounded_size<vox_nav_msgs::action::NavigateThroughGPSPoses_GetResult_Response>
  : std::integral_constant<bool, has_bounded_size<vox_nav_msgs::action::NavigateThroughGPSPoses_Result>::value> {};

template<>
struct is_message<vox_nav_msgs::action::NavigateThroughGPSPoses_GetResult_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<vox_nav_msgs::action::NavigateThroughGPSPoses_GetResult>()
{
  return "vox_nav_msgs::action::NavigateThroughGPSPoses_GetResult";
}

template<>
inline const char * name<vox_nav_msgs::action::NavigateThroughGPSPoses_GetResult>()
{
  return "vox_nav_msgs/action/NavigateThroughGPSPoses_GetResult";
}

template<>
struct has_fixed_size<vox_nav_msgs::action::NavigateThroughGPSPoses_GetResult>
  : std::integral_constant<
    bool,
    has_fixed_size<vox_nav_msgs::action::NavigateThroughGPSPoses_GetResult_Request>::value &&
    has_fixed_size<vox_nav_msgs::action::NavigateThroughGPSPoses_GetResult_Response>::value
  >
{
};

template<>
struct has_bounded_size<vox_nav_msgs::action::NavigateThroughGPSPoses_GetResult>
  : std::integral_constant<
    bool,
    has_bounded_size<vox_nav_msgs::action::NavigateThroughGPSPoses_GetResult_Request>::value &&
    has_bounded_size<vox_nav_msgs::action::NavigateThroughGPSPoses_GetResult_Response>::value
  >
{
};

template<>
struct is_service<vox_nav_msgs::action::NavigateThroughGPSPoses_GetResult>
  : std::true_type
{
};

template<>
struct is_service_request<vox_nav_msgs::action::NavigateThroughGPSPoses_GetResult_Request>
  : std::true_type
{
};

template<>
struct is_service_response<vox_nav_msgs::action::NavigateThroughGPSPoses_GetResult_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"
// Member 'feedback'
// already included above
// #include "vox_nav_msgs/action/detail/navigate_through_gps_poses__traits.hpp"

namespace vox_nav_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const NavigateThroughGPSPoses_FeedbackMessage & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
    out << ", ";
  }

  // member: feedback
  {
    out << "feedback: ";
    to_flow_style_yaml(msg.feedback, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const NavigateThroughGPSPoses_FeedbackMessage & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }

  // member: feedback
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "feedback:\n";
    to_block_style_yaml(msg.feedback, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const NavigateThroughGPSPoses_FeedbackMessage & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace vox_nav_msgs

namespace rosidl_generator_traits
{

[[deprecated("use vox_nav_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const vox_nav_msgs::action::NavigateThroughGPSPoses_FeedbackMessage & msg,
  std::ostream & out, size_t indentation = 0)
{
  vox_nav_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use vox_nav_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const vox_nav_msgs::action::NavigateThroughGPSPoses_FeedbackMessage & msg)
{
  return vox_nav_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<vox_nav_msgs::action::NavigateThroughGPSPoses_FeedbackMessage>()
{
  return "vox_nav_msgs::action::NavigateThroughGPSPoses_FeedbackMessage";
}

template<>
inline const char * name<vox_nav_msgs::action::NavigateThroughGPSPoses_FeedbackMessage>()
{
  return "vox_nav_msgs/action/NavigateThroughGPSPoses_FeedbackMessage";
}

template<>
struct has_fixed_size<vox_nav_msgs::action::NavigateThroughGPSPoses_FeedbackMessage>
  : std::integral_constant<bool, has_fixed_size<unique_identifier_msgs::msg::UUID>::value && has_fixed_size<vox_nav_msgs::action::NavigateThroughGPSPoses_Feedback>::value> {};

template<>
struct has_bounded_size<vox_nav_msgs::action::NavigateThroughGPSPoses_FeedbackMessage>
  : std::integral_constant<bool, has_bounded_size<unique_identifier_msgs::msg::UUID>::value && has_bounded_size<vox_nav_msgs::action::NavigateThroughGPSPoses_Feedback>::value> {};

template<>
struct is_message<vox_nav_msgs::action::NavigateThroughGPSPoses_FeedbackMessage>
  : std::true_type {};

}  // namespace rosidl_generator_traits


namespace rosidl_generator_traits
{

template<>
struct is_action<vox_nav_msgs::action::NavigateThroughGPSPoses>
  : std::true_type
{
};

template<>
struct is_action_goal<vox_nav_msgs::action::NavigateThroughGPSPoses_Goal>
  : std::true_type
{
};

template<>
struct is_action_result<vox_nav_msgs::action::NavigateThroughGPSPoses_Result>
  : std::true_type
{
};

template<>
struct is_action_feedback<vox_nav_msgs::action::NavigateThroughGPSPoses_Feedback>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits


#endif  // VOX_NAV_MSGS__ACTION__DETAIL__NAVIGATE_THROUGH_GPS_POSES__TRAITS_HPP_
