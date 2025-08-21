// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from vox_nav_msgs:action/NavigateThroughPoses.idl
// generated code does not contain a copyright notice

#ifndef VOX_NAV_MSGS__ACTION__DETAIL__NAVIGATE_THROUGH_POSES__BUILDER_HPP_
#define VOX_NAV_MSGS__ACTION__DETAIL__NAVIGATE_THROUGH_POSES__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "vox_nav_msgs/action/detail/navigate_through_poses__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace vox_nav_msgs
{

namespace action
{

namespace builder
{

class Init_NavigateThroughPoses_Goal_behavior_tree
{
public:
  explicit Init_NavigateThroughPoses_Goal_behavior_tree(::vox_nav_msgs::action::NavigateThroughPoses_Goal & msg)
  : msg_(msg)
  {}
  ::vox_nav_msgs::action::NavigateThroughPoses_Goal behavior_tree(::vox_nav_msgs::action::NavigateThroughPoses_Goal::_behavior_tree_type arg)
  {
    msg_.behavior_tree = std::move(arg);
    return std::move(msg_);
  }

private:
  ::vox_nav_msgs::action::NavigateThroughPoses_Goal msg_;
};

class Init_NavigateThroughPoses_Goal_poses
{
public:
  Init_NavigateThroughPoses_Goal_poses()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_NavigateThroughPoses_Goal_behavior_tree poses(::vox_nav_msgs::action::NavigateThroughPoses_Goal::_poses_type arg)
  {
    msg_.poses = std::move(arg);
    return Init_NavigateThroughPoses_Goal_behavior_tree(msg_);
  }

private:
  ::vox_nav_msgs::action::NavigateThroughPoses_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::vox_nav_msgs::action::NavigateThroughPoses_Goal>()
{
  return vox_nav_msgs::action::builder::Init_NavigateThroughPoses_Goal_poses();
}

}  // namespace vox_nav_msgs


namespace vox_nav_msgs
{

namespace action
{

namespace builder
{

class Init_NavigateThroughPoses_Result_result
{
public:
  Init_NavigateThroughPoses_Result_result()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::vox_nav_msgs::action::NavigateThroughPoses_Result result(::vox_nav_msgs::action::NavigateThroughPoses_Result::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::vox_nav_msgs::action::NavigateThroughPoses_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::vox_nav_msgs::action::NavigateThroughPoses_Result>()
{
  return vox_nav_msgs::action::builder::Init_NavigateThroughPoses_Result_result();
}

}  // namespace vox_nav_msgs


namespace vox_nav_msgs
{

namespace action
{

namespace builder
{

class Init_NavigateThroughPoses_Feedback_distance_remaining
{
public:
  explicit Init_NavigateThroughPoses_Feedback_distance_remaining(::vox_nav_msgs::action::NavigateThroughPoses_Feedback & msg)
  : msg_(msg)
  {}
  ::vox_nav_msgs::action::NavigateThroughPoses_Feedback distance_remaining(::vox_nav_msgs::action::NavigateThroughPoses_Feedback::_distance_remaining_type arg)
  {
    msg_.distance_remaining = std::move(arg);
    return std::move(msg_);
  }

private:
  ::vox_nav_msgs::action::NavigateThroughPoses_Feedback msg_;
};

class Init_NavigateThroughPoses_Feedback_number_of_recoveries
{
public:
  explicit Init_NavigateThroughPoses_Feedback_number_of_recoveries(::vox_nav_msgs::action::NavigateThroughPoses_Feedback & msg)
  : msg_(msg)
  {}
  Init_NavigateThroughPoses_Feedback_distance_remaining number_of_recoveries(::vox_nav_msgs::action::NavigateThroughPoses_Feedback::_number_of_recoveries_type arg)
  {
    msg_.number_of_recoveries = std::move(arg);
    return Init_NavigateThroughPoses_Feedback_distance_remaining(msg_);
  }

private:
  ::vox_nav_msgs::action::NavigateThroughPoses_Feedback msg_;
};

class Init_NavigateThroughPoses_Feedback_navigation_time
{
public:
  explicit Init_NavigateThroughPoses_Feedback_navigation_time(::vox_nav_msgs::action::NavigateThroughPoses_Feedback & msg)
  : msg_(msg)
  {}
  Init_NavigateThroughPoses_Feedback_number_of_recoveries navigation_time(::vox_nav_msgs::action::NavigateThroughPoses_Feedback::_navigation_time_type arg)
  {
    msg_.navigation_time = std::move(arg);
    return Init_NavigateThroughPoses_Feedback_number_of_recoveries(msg_);
  }

private:
  ::vox_nav_msgs::action::NavigateThroughPoses_Feedback msg_;
};

class Init_NavigateThroughPoses_Feedback_current_pose
{
public:
  Init_NavigateThroughPoses_Feedback_current_pose()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_NavigateThroughPoses_Feedback_navigation_time current_pose(::vox_nav_msgs::action::NavigateThroughPoses_Feedback::_current_pose_type arg)
  {
    msg_.current_pose = std::move(arg);
    return Init_NavigateThroughPoses_Feedback_navigation_time(msg_);
  }

private:
  ::vox_nav_msgs::action::NavigateThroughPoses_Feedback msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::vox_nav_msgs::action::NavigateThroughPoses_Feedback>()
{
  return vox_nav_msgs::action::builder::Init_NavigateThroughPoses_Feedback_current_pose();
}

}  // namespace vox_nav_msgs


namespace vox_nav_msgs
{

namespace action
{

namespace builder
{

class Init_NavigateThroughPoses_SendGoal_Request_goal
{
public:
  explicit Init_NavigateThroughPoses_SendGoal_Request_goal(::vox_nav_msgs::action::NavigateThroughPoses_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::vox_nav_msgs::action::NavigateThroughPoses_SendGoal_Request goal(::vox_nav_msgs::action::NavigateThroughPoses_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::vox_nav_msgs::action::NavigateThroughPoses_SendGoal_Request msg_;
};

class Init_NavigateThroughPoses_SendGoal_Request_goal_id
{
public:
  Init_NavigateThroughPoses_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_NavigateThroughPoses_SendGoal_Request_goal goal_id(::vox_nav_msgs::action::NavigateThroughPoses_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_NavigateThroughPoses_SendGoal_Request_goal(msg_);
  }

private:
  ::vox_nav_msgs::action::NavigateThroughPoses_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::vox_nav_msgs::action::NavigateThroughPoses_SendGoal_Request>()
{
  return vox_nav_msgs::action::builder::Init_NavigateThroughPoses_SendGoal_Request_goal_id();
}

}  // namespace vox_nav_msgs


namespace vox_nav_msgs
{

namespace action
{

namespace builder
{

class Init_NavigateThroughPoses_SendGoal_Response_stamp
{
public:
  explicit Init_NavigateThroughPoses_SendGoal_Response_stamp(::vox_nav_msgs::action::NavigateThroughPoses_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::vox_nav_msgs::action::NavigateThroughPoses_SendGoal_Response stamp(::vox_nav_msgs::action::NavigateThroughPoses_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::vox_nav_msgs::action::NavigateThroughPoses_SendGoal_Response msg_;
};

class Init_NavigateThroughPoses_SendGoal_Response_accepted
{
public:
  Init_NavigateThroughPoses_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_NavigateThroughPoses_SendGoal_Response_stamp accepted(::vox_nav_msgs::action::NavigateThroughPoses_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_NavigateThroughPoses_SendGoal_Response_stamp(msg_);
  }

private:
  ::vox_nav_msgs::action::NavigateThroughPoses_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::vox_nav_msgs::action::NavigateThroughPoses_SendGoal_Response>()
{
  return vox_nav_msgs::action::builder::Init_NavigateThroughPoses_SendGoal_Response_accepted();
}

}  // namespace vox_nav_msgs


namespace vox_nav_msgs
{

namespace action
{

namespace builder
{

class Init_NavigateThroughPoses_GetResult_Request_goal_id
{
public:
  Init_NavigateThroughPoses_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::vox_nav_msgs::action::NavigateThroughPoses_GetResult_Request goal_id(::vox_nav_msgs::action::NavigateThroughPoses_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::vox_nav_msgs::action::NavigateThroughPoses_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::vox_nav_msgs::action::NavigateThroughPoses_GetResult_Request>()
{
  return vox_nav_msgs::action::builder::Init_NavigateThroughPoses_GetResult_Request_goal_id();
}

}  // namespace vox_nav_msgs


namespace vox_nav_msgs
{

namespace action
{

namespace builder
{

class Init_NavigateThroughPoses_GetResult_Response_result
{
public:
  explicit Init_NavigateThroughPoses_GetResult_Response_result(::vox_nav_msgs::action::NavigateThroughPoses_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::vox_nav_msgs::action::NavigateThroughPoses_GetResult_Response result(::vox_nav_msgs::action::NavigateThroughPoses_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::vox_nav_msgs::action::NavigateThroughPoses_GetResult_Response msg_;
};

class Init_NavigateThroughPoses_GetResult_Response_status
{
public:
  Init_NavigateThroughPoses_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_NavigateThroughPoses_GetResult_Response_result status(::vox_nav_msgs::action::NavigateThroughPoses_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_NavigateThroughPoses_GetResult_Response_result(msg_);
  }

private:
  ::vox_nav_msgs::action::NavigateThroughPoses_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::vox_nav_msgs::action::NavigateThroughPoses_GetResult_Response>()
{
  return vox_nav_msgs::action::builder::Init_NavigateThroughPoses_GetResult_Response_status();
}

}  // namespace vox_nav_msgs


namespace vox_nav_msgs
{

namespace action
{

namespace builder
{

class Init_NavigateThroughPoses_FeedbackMessage_feedback
{
public:
  explicit Init_NavigateThroughPoses_FeedbackMessage_feedback(::vox_nav_msgs::action::NavigateThroughPoses_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::vox_nav_msgs::action::NavigateThroughPoses_FeedbackMessage feedback(::vox_nav_msgs::action::NavigateThroughPoses_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::vox_nav_msgs::action::NavigateThroughPoses_FeedbackMessage msg_;
};

class Init_NavigateThroughPoses_FeedbackMessage_goal_id
{
public:
  Init_NavigateThroughPoses_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_NavigateThroughPoses_FeedbackMessage_feedback goal_id(::vox_nav_msgs::action::NavigateThroughPoses_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_NavigateThroughPoses_FeedbackMessage_feedback(msg_);
  }

private:
  ::vox_nav_msgs::action::NavigateThroughPoses_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::vox_nav_msgs::action::NavigateThroughPoses_FeedbackMessage>()
{
  return vox_nav_msgs::action::builder::Init_NavigateThroughPoses_FeedbackMessage_goal_id();
}

}  // namespace vox_nav_msgs

#endif  // VOX_NAV_MSGS__ACTION__DETAIL__NAVIGATE_THROUGH_POSES__BUILDER_HPP_
