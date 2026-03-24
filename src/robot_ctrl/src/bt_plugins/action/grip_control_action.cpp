#include "robot_ctrl/bt_plugins/action/grip_control_action.hpp"
#include <behaviortree_cpp_v3/bt_factory.h>
#include "robot_ctrl/bt_plugins/error_log_queue.hpp"

using namespace std::chrono_literals;

namespace robot_ctrl
{
GripControlAction::GripControlAction(const std::string& name, const BT::NodeConfiguration& config)
: BT::StatefulActionNode(name, config)
{
  // 从黑板里取 Node
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");

  // 创建 service client
  client_ = node_->create_client<robotcar_base::srv::SetControlMode>("/set_control_mode");

  // 订阅 carinfo
  sub_ = node_->create_subscription<robotcar_base::msg::CarInfo>(
    "/car_info", 10,
    [this](const robotcar_base::msg::CarInfo::SharedPtr msg)
    {
      std::lock_guard<std::mutex> lock(mutex_);
      current_hand_cap_ = msg->hand_capture;
    });
}

BT::PortsList GripControlAction::providedPorts()
{
  return { 
    BT::InputPort<int>("mode2", "Gripper control mode: 1=open, 2=close")
  };
}

BT::NodeStatus GripControlAction::onStart()
{
  // 读取输入参数
  if (!getInput<int>("mode2", target_mode2_)) {
    RCLCPP_ERROR(node_->get_logger(), "GripControlAction: missing required input [mode2]");
    ErrorLogQueue::instance().pushError(
      error_code::kInputMissing,
      "GripControlAction: missing required input [mode2]");
    return BT::NodeStatus::FAILURE;
  }

  // 建立服务请求
  auto request = std::make_shared<robotcar_base::srv::SetControlMode::Request>();
  request->mode2 = target_mode2_;

  if (!client_->wait_for_service(1s)) {
    RCLCPP_ERROR(node_->get_logger(), "Service /set_control_mode not available");
    ErrorLogQueue::instance().pushError(
      error_code::kServiceUnavailable,
      "GripControlAction: Service /set_control_mode not available");
    return BT::NodeStatus::FAILURE;
  }

  auto future = client_->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node_, future, 2s) !=
      rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(), "Failed to call /set_control_mode");
    ErrorLogQueue::instance().pushError(
      error_code::kActionFailure,
      "GripControlAction: Failed to call /set_control_mode");
    return BT::NodeStatus::FAILURE;
  }

  start_time_ = node_->now();
  RCLCPP_INFO(node_->get_logger(), "Sent gripper command: mode2=%d", target_mode2_);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GripControlAction::onRunning()
{
  std::lock_guard<std::mutex> lock(mutex_);
  int expected_hand_cap = (target_mode2_ == 1) ? 1 : 2;

  if (current_hand_cap_ == expected_hand_cap) {
    RCLCPP_INFO(node_->get_logger(), "Gripper reached target state: hand_cap=%d", current_hand_cap_);
    ErrorLogQueue::instance().clearLastErrorIfPrefix("GripControlAction:");
    return BT::NodeStatus::SUCCESS;
  }

  if ((node_->now() - start_time_).seconds() > 30.0) {
    RCLCPP_ERROR(node_->get_logger(), "Gripper did not reach target state within 30s (expected=%d, current=%d)",
                 expected_hand_cap, current_hand_cap_);
    ErrorLogQueue::instance().pushError(
      error_code::kTimeout,
      "GripControlAction: Gripper did not reach target state within 30s");
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::RUNNING;
}

void GripControlAction::onHalted()
{
  RCLCPP_WARN(node_->get_logger(), "GripControlAction halted");
}

}
