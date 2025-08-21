// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from vox_nav_msgs:action/FollowPath.idl
// generated code does not contain a copyright notice

#ifndef VOX_NAV_MSGS__ACTION__DETAIL__FOLLOW_PATH__BUILDER_HPP_
#define VOX_NAV_MSGS__ACTION__DETAIL__FOLLOW_PATH__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "vox_nav_msgs/action/detail/follow_path__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace vox_nav_msgs
{

namespace action
{

namespace builder
{

class Init_FollowPath_Goal_controller_id
{
public:
  explicit Init_FollowPath_Goal_controller_id(::vox_nav_msgs::action::FollowPath_Goal & msg)
  : msg_(msg)
  {}
  ::vox_nav_msgs::action::FollowPath_Goal controller_id(::vox_nav_msgs::action::FollowPath_Goal::_controller_id_type arg)
  {
    msg_.controller_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::vox_nav_msgs::action::FollowPath_Goal msg_;
};

class Init_FollowPath_Goal_path
{
public:
  Init_FollowPath_Goal_path()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FollowPath_Goal_controller_id path(::vox_nav_msgs::action::FollowPath_Goal::_path_type arg)
  {
    msg_.path = std::move(arg);
    return Init_FollowPath_Goal_controller_id(msg_);
  }

private:
  ::vox_nav_msgs::action::FollowPath_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::vox_nav_msgs::action::FollowPath_Goal>()
{
  return vox_nav_msgs::action::builder::Init_FollowPath_Goal_path();
}

}  // namespace vox_nav_msgs


namespace vox_nav_msgs
{

namespace action
{

namespace builder
{

class Init_FollowPath_Result_total_time
{
public:
  Init_FollowPath_Result_total_time()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::vox_nav_msgs::action::FollowPath_Result total_time(::vox_nav_msgs::action::FollowPath_Result::_total_time_type arg)
  {
    msg_.total_time = std::move(arg);
    return std::move(msg_);
  }

private:
  ::vox_nav_msgs::action::FollowPath_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::vox_nav_msgs::action::FollowPath_Result>()
{
  return vox_nav_msgs::action::builder::Init_FollowPath_Result_total_time();
}

}  // namespace vox_nav_msgs


namespace vox_nav_msgs
{

namespace action
{

namespace builder
{

class Init_FollowPath_Feedback_elapsed_time
{
public:
  explicit Init_FollowPath_Feedback_elapsed_time(::vox_nav_msgs::action::FollowPath_Feedback & msg)
  : msg_(msg)
  {}
  ::vox_nav_msgs::action::FollowPath_Feedback elapsed_time(::vox_nav_msgs::action::FollowPath_Feedback::_elapsed_time_type arg)
  {
    msg_.elapsed_time = std::move(arg);
    return std::move(msg_);
  }

private:
  ::vox_nav_msgs::action::FollowPath_Feedback msg_;
};

class Init_FollowPath_Feedback_speed
{
public:
  explicit Init_FollowPath_Feedback_speed(::vox_nav_msgs::action::FollowPath_Feedback & msg)
  : msg_(msg)
  {}
  Init_FollowPath_Feedback_elapsed_time speed(::vox_nav_msgs::action::FollowPath_Feedback::_speed_type arg)
  {
    msg_.speed = std::move(arg);
    return Init_FollowPath_Feedback_elapsed_time(msg_);
  }

private:
  ::vox_nav_msgs::action::FollowPath_Feedback msg_;
};

class Init_FollowPath_Feedback_distance_to_goal
{
public:
  Init_FollowPath_Feedback_distance_to_goal()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FollowPath_Feedback_speed distance_to_goal(::vox_nav_msgs::action::FollowPath_Feedback::_distance_to_goal_type arg)
  {
    msg_.distance_to_goal = std::move(arg);
    return Init_FollowPath_Feedback_speed(msg_);
  }

private:
  ::vox_nav_msgs::action::FollowPath_Feedback msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::vox_nav_msgs::action::FollowPath_Feedback>()
{
  return vox_nav_msgs::action::builder::Init_FollowPath_Feedback_distance_to_goal();
}

}  // namespace vox_nav_msgs


namespace vox_nav_msgs
{

namespace action
{

namespace builder
{

class Init_FollowPath_SendGoal_Request_goal
{
public:
  explicit Init_FollowPath_SendGoal_Request_goal(::vox_nav_msgs::action::FollowPath_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::vox_nav_msgs::action::FollowPath_SendGoal_Request goal(::vox_nav_msgs::action::FollowPath_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::vox_nav_msgs::action::FollowPath_SendGoal_Request msg_;
};

class Init_FollowPath_SendGoal_Request_goal_id
{
public:
  Init_FollowPath_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FollowPath_SendGoal_Request_goal goal_id(::vox_nav_msgs::action::FollowPath_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_FollowPath_SendGoal_Request_goal(msg_);
  }

private:
  ::vox_nav_msgs::action::FollowPath_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::vox_nav_msgs::action::FollowPath_SendGoal_Request>()
{
  return vox_nav_msgs::action::builder::Init_FollowPath_SendGoal_Request_goal_id();
}

}  // namespace vox_nav_msgs


namespace vox_nav_msgs
{

namespace action
{

namespace builder
{

class Init_FollowPath_SendGoal_Response_stamp
{
public:
  explicit Init_FollowPath_SendGoal_Response_stamp(::vox_nav_msgs::action::FollowPath_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::vox_nav_msgs::action::FollowPath_SendGoal_Response stamp(::vox_nav_msgs::action::FollowPath_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::vox_nav_msgs::action::FollowPath_SendGoal_Response msg_;
};

class Init_FollowPath_SendGoal_Response_accepted
{
public:
  Init_FollowPath_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FollowPath_SendGoal_Response_stamp accepted(::vox_nav_msgs::action::FollowPath_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_FollowPath_SendGoal_Response_stamp(msg_);
  }

private:
  ::vox_nav_msgs::action::FollowPath_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::vox_nav_msgs::action::FollowPath_SendGoal_Response>()
{
  return vox_nav_msgs::action::builder::Init_FollowPath_SendGoal_Response_accepted();
}

}  // namespace vox_nav_msgs


namespace vox_nav_msgs
{

namespace action
{

namespace builder
{

class Init_FollowPath_GetResult_Request_goal_id
{
public:
  Init_FollowPath_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::vox_nav_msgs::action::FollowPath_GetResult_Request goal_id(::vox_nav_msgs::action::FollowPath_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::vox_nav_msgs::action::FollowPath_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::vox_nav_msgs::action::FollowPath_GetResult_Request>()
{
  return vox_nav_msgs::action::builder::Init_FollowPath_GetResult_Request_goal_id();
}

}  // namespace vox_nav_msgs


namespace vox_nav_msgs
{

namespace action
{

namespace builder
{

class Init_FollowPath_GetResult_Response_result
{
public:
  explicit Init_FollowPath_GetResult_Response_result(::vox_nav_msgs::action::FollowPath_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::vox_nav_msgs::action::FollowPath_GetResult_Response result(::vox_nav_msgs::action::FollowPath_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::vox_nav_msgs::action::FollowPath_GetResult_Response msg_;
};

class Init_FollowPath_GetResult_Response_status
{
public:
  Init_FollowPath_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FollowPath_GetResult_Response_result status(::vox_nav_msgs::action::FollowPath_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_FollowPath_GetResult_Response_result(msg_);
  }

private:
  ::vox_nav_msgs::action::FollowPath_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::vox_nav_msgs::action::FollowPath_GetResult_Response>()
{
  return vox_nav_msgs::action::builder::Init_FollowPath_GetResult_Response_status();
}

}  // namespace vox_nav_msgs


namespace vox_nav_msgs
{

namespace action
{

namespace builder
{

class Init_FollowPath_FeedbackMessage_feedback
{
public:
  explicit Init_FollowPath_FeedbackMessage_feedback(::vox_nav_msgs::action::FollowPath_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::vox_nav_msgs::action::FollowPath_FeedbackMessage feedback(::vox_nav_msgs::action::FollowPath_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::vox_nav_msgs::action::FollowPath_FeedbackMessage msg_;
};

class Init_FollowPath_FeedbackMessage_goal_id
{
public:
  Init_FollowPath_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FollowPath_FeedbackMessage_feedback goal_id(::vox_nav_msgs::action::FollowPath_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_FollowPath_FeedbackMessage_feedback(msg_);
  }

private:
  ::vox_nav_msgs::action::FollowPath_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::vox_nav_msgs::action::FollowPath_FeedbackMessage>()
{
  return vox_nav_msgs::action::builder::Init_FollowPath_FeedbackMessage_goal_id();
}

}  // namespace vox_nav_msgs

#endif  // VOX_NAV_MSGS__ACTION__DETAIL__FOLLOW_PATH__BUILDER_HPP_
