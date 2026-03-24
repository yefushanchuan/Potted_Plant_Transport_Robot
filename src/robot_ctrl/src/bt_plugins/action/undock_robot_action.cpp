#include "robot_ctrl/bt_plugins/action/undock_robot_action.hpp"
#include "rclcpp/rclcpp.hpp"

namespace robot_ctrl
{

UndockRobotActionBT::UndockRobotActionBT(const std::string & name,
                                         const BT::NodeConfiguration & config)
: BT::StatefulActionNode(name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
  action_client_ = rclcpp_action::create_client<UndockRobot>(node_, "/robot_dock");
  node_->get_parameter_or("send_goal_timeout_ms", send_goal_timeout_, 1000);
}

BT::NodeStatus UndockRobotActionBT::onStart()
{
  auto update_feedback = [this](const std::string & feedback, double progress)
  {
    if (config().blackboard)
    {
      config().blackboard->set("action_feedback", feedback);
      config().blackboard->set("progress", progress);
    }
    setOutput("action_feedback", feedback);
    setOutput("progress", progress);
  };

  {
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    last_feedback_.clear();
    last_progress_ = 0.0;
  }

  // 从黑板获取参数
  if (!getInput("charge_stop", goal_.charge_stop)) {
    RCLCPP_ERROR(node_->get_logger(), "Missing 'charge_stop'");
    update_feedback("UndockRobot: 缺少 charge_stop 输入", 0.0);
    return BT::NodeStatus::FAILURE;
  }
  if (!getInput("charge_enable", goal_.charge_enable)) {
    RCLCPP_ERROR(node_->get_logger(), "Missing 'charge_enable'");
    update_feedback("UndockRobot: 缺少 charge_enable 输入", 0.0);
    return BT::NodeStatus::FAILURE;
  }

  // 等待 action server
  if (!action_client_->wait_for_action_server(std::chrono::seconds(2))) {
    RCLCPP_ERROR(node_->get_logger(), "UndockRobot action server not available");
    ErrorLogQueue::instance().pushError(
      error_code::kServiceUnavailable,
      "UndockRobotActionBT: UndockRobot action server not available");
    update_feedback("UndockRobot: action 服务不可用", 0.0);
    return BT::NodeStatus::FAILURE;
  }

  auto send_goal_options = rclcpp_action::Client<UndockRobot>::SendGoalOptions();
  send_goal_options.feedback_callback =
    [this](GoalHandleUndockRobot::SharedPtr,
           const std::shared_ptr<const UndockRobot::Feedback> feedback)
    {
      if (!feedback)
      {
        return;
      }
      std::lock_guard<std::mutex> lock(feedback_mutex_);
      last_feedback_ = "UndockRobot: " + feedback->state;
    };

  auto future_goal_handle = action_client_->async_send_goal(goal_, send_goal_options);

  if (rclcpp::spin_until_future_complete(node_, future_goal_handle, std::chrono::milliseconds(send_goal_timeout_)) !=
      rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(), "Send goal failed.");
    ErrorLogQueue::instance().pushError(
      error_code::kActionFailure,
      "UndockRobotActionBT: Send goal failed");
    update_feedback("UndockRobot: 发送目标失败", 0.0);
    return BT::NodeStatus::FAILURE;
  }

  goal_handle_ = future_goal_handle.get();
  if (!goal_handle_) {
    RCLCPP_ERROR(node_->get_logger(), "Goal handle is null.");
    ErrorLogQueue::instance().pushError(
      error_code::kActionFailure,
      "UndockRobotActionBT: Goal handle is null");
    update_feedback("UndockRobot: 目标句柄为空", 0.0);
    return BT::NodeStatus::FAILURE;
  }

  result_future_ = action_client_->async_get_result(goal_handle_);
  RCLCPP_INFO(node_->get_logger(), "Undock goal sent");
  update_feedback("UndockRobot: 目标已发送", 0.0);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus UndockRobotActionBT::onRunning()
{
  auto update_feedback = [this](const std::string & feedback, double progress)
  {
    if (config().blackboard)
    {
      config().blackboard->set("action_feedback", feedback);
      config().blackboard->set("progress", progress);
    }
    setOutput("action_feedback", feedback);
    setOutput("progress", progress);
  };

  if (!goal_handle_) {
    RCLCPP_ERROR(node_->get_logger(), "Goal handle not initialized.");
    ErrorLogQueue::instance().pushError(
      error_code::kActionFailure,
      "UndockRobotActionBT: Goal handle not initialized");
    update_feedback("UndockRobot: 目标句柄未初始化", 0.0);
    return BT::NodeStatus::FAILURE;
  }

  auto status = result_future_.wait_for(std::chrono::milliseconds(2));

  if (status == std::future_status::ready)
  {
    auto wrapped_result = result_future_.get();

    switch (wrapped_result.code)
    {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(node_->get_logger(), "UNDock succeeded.");
        ErrorLogQueue::instance().clearLastErrorIfPrefix("UndockRobotActionBT:");
        update_feedback("UndockRobot: 执行成功", 1.0);
        return BT::NodeStatus::SUCCESS;

      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(node_->get_logger(), "UNDock aborted.");
        ErrorLogQueue::instance().pushError(
          error_code::kActionFailure,
          "UndockRobotActionBT: UNDock aborted");
        update_feedback("UndockRobot: 执行中止", 0.0);
        return BT::NodeStatus::FAILURE;

      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_WARN(node_->get_logger(), "UNDock canceled.");
        ErrorLogQueue::instance().pushError(
          error_code::kActionFailure,
          "UndockRobotActionBT: UNDock canceled");
        update_feedback("UndockRobot: 执行取消", 0.0);
        return BT::NodeStatus::FAILURE;

      default:
        RCLCPP_ERROR(node_->get_logger(), "Unknown result code.");
        ErrorLogQueue::instance().pushError(
          error_code::kActionFailure,
          "UndockRobotActionBT: Unknown result code");
        update_feedback("UndockRobot: 未知结果码", 0.0);
        return BT::NodeStatus::FAILURE;
    }
  }

  std::string feedback = "UndockRobot: 执行中";
  {
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    if (!last_feedback_.empty())
    {
      feedback = last_feedback_;
    }
  }
  update_feedback(feedback, 0.5);
  return BT::NodeStatus::RUNNING;
}

// ---------------- onHalted ----------------
void UndockRobotActionBT::onHalted()
{
  RCLCPP_INFO(node_->get_logger(), "Undock goal halted.");
  if (goal_handle_) {
    auto cancel_future = action_client_->async_cancel_goal(goal_handle_);
    if (rclcpp::spin_until_future_complete(node_, cancel_future) != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(node_->get_logger(), "Cancel undock goal failed.");
    }
    RCLCPP_INFO(node_->get_logger(), "Undock goal canceled.");
  }

  if (config().blackboard)
  {
    config().blackboard->set("action_feedback", "UndockRobot: 已终止");
    config().blackboard->set("progress", 0.0);
  }
  setOutput("action_feedback", "UndockRobot: 已终止");
  setOutput("progress", 0.0);
}

} 
