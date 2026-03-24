#include "robot_ctrl/bt_plugins/condition/is_time_for_action_condition.hpp"

#include <chrono>
#include <ctime>
#include <limits>

#include "robot_ctrl/bt_plugins/error_log_queue.hpp"

namespace robot_ctrl
{

IsTimeForAction::IsTimeForAction(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config),
  system_clock_(RCL_SYSTEM_TIME),
  last_trigger_time_(0, 0, RCL_SYSTEM_TIME)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
}

BT::PortsList IsTimeForAction::providedPorts()
{
  return {
    BT::InputPort<int>("trigger_hour", -1, "触发小时[0-23]，-1 表示忽略小时"),
    BT::InputPort<int>("trigger_minute", 0, "在第几分钟触发[0-59]"),
    BT::InputPort<int>("window_seconds", 5, "触发秒宽度，避免重复触发"),
    BT::InputPort<int>("min_interval_sec", 3600, "两次触发的最小间隔")
  };
}

BT::NodeStatus IsTimeForAction::tick()
{
  int trigger_hour = -1;
  (void)getInput("trigger_hour", trigger_hour);
  if (trigger_hour < -1 || trigger_hour > 23)
  {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
      "[IsTimeForAction] trigger_hour 超出范围: %d", trigger_hour);
    if (!invalid_trigger_hour_reported_)
    {
      ErrorLogQueue::instance().pushError(
        error_code::kInvalidParam,
        "IsTimeForAction: trigger_hour out of range (" + std::to_string(trigger_hour) + ")");
      invalid_trigger_hour_reported_ = true;
    }
    return BT::NodeStatus::FAILURE;
  }
  invalid_trigger_hour_reported_ = false;

  int trigger_minute = 0;
  (void)getInput("trigger_minute", trigger_minute);

  if (trigger_minute < 0 || trigger_minute > 59)
  {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
      "[IsTimeForAction] trigger_minute 超出范围: %d", trigger_minute);
    if (!invalid_trigger_reported_)
    {
      ErrorLogQueue::instance().pushError(
        error_code::kInvalidParam,
        "IsTimeForAction: trigger_minute out of range (" + std::to_string(trigger_minute) + ")");
      invalid_trigger_reported_ = true;
    }
    return BT::NodeStatus::FAILURE;
  }
  invalid_trigger_reported_ = false;

  int window_seconds = 5;
  (void)getInput("window_seconds", window_seconds);
  if (window_seconds <= 0)
  {
    window_seconds = 1;
  }

  int min_interval_sec = 3600;
  (void)getInput("min_interval_sec", min_interval_sec);
  if (min_interval_sec == 0)
  {
    return BT::NodeStatus::FAILURE;
  }
  if (min_interval_sec < 0)
  {
    min_interval_sec = 1;
  }

  const auto now = system_clock_.now();
  const auto now_time_point = std::chrono::system_clock::time_point(
    std::chrono::duration_cast<std::chrono::system_clock::duration>(
      std::chrono::nanoseconds(now.nanoseconds())));
  const std::time_t now_c = std::chrono::system_clock::to_time_t(now_time_point);

  std::tm now_tm;
  if (!localtime_r(&now_c, &now_tm))
  {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
      "[IsTimeForPatrol] 无法获取本地时间");
    if (!time_error_reported_)
    {
      ErrorLogQueue::instance().pushError(
        error_code::kUnknown, "IsTimeForAction: Failed to get localtime");
      time_error_reported_ = true;
    }
    return BT::NodeStatus::FAILURE;
  }
  time_error_reported_ = false;
  ErrorLogQueue::instance().clearLastErrorIfPrefix("IsTimeForAction:");

  const bool hour_match = (trigger_hour < 0) ? true : (now_tm.tm_hour == trigger_hour);
  const bool in_trigger_window =
    hour_match && now_tm.tm_min == trigger_minute && now_tm.tm_sec < window_seconds;

  double seconds_since_last = std::numeric_limits<double>::infinity();
  if (last_trigger_time_.nanoseconds() > 0)
  {
    seconds_since_last = (now - last_trigger_time_).seconds();
  }

  if (in_trigger_window && seconds_since_last >= static_cast<double>(min_interval_sec))
  {
    last_trigger_time_ = now;
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE;
}

}  // namespace robot_ctrl
