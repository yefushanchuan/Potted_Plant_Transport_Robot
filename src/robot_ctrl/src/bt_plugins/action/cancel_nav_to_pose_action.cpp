#include "robot_ctrl/bt_plugins/action/cancel_nav_to_pose_action.hpp"

#include <chrono>
#include <thread>

#include "action_msgs/srv/cancel_goal.hpp"
#include "robot_ctrl/bt_plugins/error_log_queue.hpp"

namespace robot_ctrl
{

CancelNavToPose::CancelNavToPose(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
  action_client_ = rclcpp_action::create_client<NavigateToPose>(node_, "navigate_to_pose");
  cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", rclcpp::QoS(10));
}

BT::PortsList CancelNavToPose::providedPorts()
{
  return {
    BT::InputPort<int>("timeout_ms", 1000, "等待取消指令完成的超时时间，单位毫秒"),
    BT::InputPort<int>("brake_duration_ms", 300, "取消导航后发布零速的持续时间，单位毫秒"),
    BT::OutputPort<std::string>("action_feedback"),
    BT::OutputPort<double>("progress")
  };
}

BT::NodeStatus CancelNavToPose::tick()
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

  update_feedback("CancelNavToPose: 开始取消", 0.0);

  int timeout_ms = 1000;
  if (auto timeout_port = getInput<int>("timeout_ms"))
  {
    timeout_ms = timeout_port.value();
  }

  int brake_duration_ms = 300;
  if (auto brake_port = getInput<int>("brake_duration_ms"))
  {
    brake_duration_ms = std::max(0, brake_port.value());
  }

  // 先发一次零速，尽量降低取消链路带来的继续前冲。
  if (cmd_vel_pub_)
  {
    cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
  }

  const auto timeout = std::chrono::milliseconds(timeout_ms);

  if (!action_client_->wait_for_action_server(timeout))
  {
    RCLCPP_ERROR(node_->get_logger(), "CancelNavToPose: 等待 navigate_to_pose 服务器超时。");
    ErrorLogQueue::instance().pushError(
      error_code::kTimeout,
      "CancelNavToPose: waiting for navigate_to_pose server timed out");
    update_feedback("CancelNavToPose: 服务器超时", 0.0);
    return BT::NodeStatus::FAILURE;
  }

  auto cancel_future = action_client_->async_cancel_all_goals();
  auto future_state = rclcpp::spin_until_future_complete(node_, cancel_future, timeout);

  if (future_state != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(), "CancelNavToPose: 取消请求超时或失败。");
    ErrorLogQueue::instance().pushError(
      error_code::kTimeout,
      "CancelNavToPose: cancel request timed out or failed");
    update_feedback("CancelNavToPose: 取消请求失败", 0.0);
    return BT::NodeStatus::FAILURE;
  }

  auto response = cancel_future.get();
  if (!response)
  {
    RCLCPP_ERROR(node_->get_logger(), "CancelNavToPose: 取消响应为空。");
    ErrorLogQueue::instance().pushError(
      error_code::kActionFailure,
      "CancelNavToPose: cancel response is null");
    update_feedback("CancelNavToPose: 取消响应为空", 0.0);
    return BT::NodeStatus::FAILURE;
  }

  if (response->return_code != action_msgs::srv::CancelGoal::Response::ERROR_NONE)
  {
    RCLCPP_WARN(node_->get_logger(),
      "CancelNavToPose: 取消请求被拒绝，返回码=%d。", response->return_code);
    ErrorLogQueue::instance().pushError(
      error_code::kActionFailure,
      "CancelNavToPose: cancel request rejected");
    update_feedback("CancelNavToPose: 取消请求被拒绝", 0.0);
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(node_->get_logger(), "CancelNavToPose: 已发出取消导航指令。");
  update_feedback("CancelNavToPose: 已发出取消指令", 1.0);

  // 取消后短时间持续发布零速，确保底盘侧收到“停车”命令（避免仅靠超时/平滑器导致停车滞后）。
  if (cmd_vel_pub_ && brake_duration_ms > 0)
  {
    const auto start = std::chrono::steady_clock::now();
    const auto duration = std::chrono::milliseconds(brake_duration_ms);
    while (rclcpp::ok() && (std::chrono::steady_clock::now() - start) < duration)
    {
      cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  }

  update_feedback("CancelNavToPose: 取消完成", 1.0);
  ErrorLogQueue::instance().clearLastErrorIfPrefix("CancelNavToPose:");
  return BT::NodeStatus::SUCCESS;
}

}  // namespace robot_ctrl
