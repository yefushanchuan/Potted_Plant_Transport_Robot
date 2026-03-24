#pragma once

#include <chrono>
#include <future>
#include <mutex>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_ctrl/action/bt_task_file_upload.hpp"

namespace robot_ctrl
{

class TaskfileUploadAction : public BT::StatefulActionNode
{
public:
  TaskfileUploadAction(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  using BtTaskFileUpload = robot_ctrl::action::BtTaskFileUpload;
  using BtTaskFileUploadGoalHandle = rclcpp_action::ClientGoalHandle<BtTaskFileUpload>;

  void resetState();
  void setResultMessage(const std::string & message);

  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<BtTaskFileUpload>::SharedPtr action_client_;
  BtTaskFileUploadGoalHandle::SharedPtr goal_handle_;
  std::shared_future<BtTaskFileUploadGoalHandle::WrappedResult> result_future_;
  std::mutex feedback_mutex_;
  float latest_progress_{0.0F};
  int send_goal_timeout_ms_{1000};
  std::chrono::steady_clock::time_point deadline_;
  std::chrono::milliseconds timeout_{120000};
};

}  // namespace robot_ctrl
