#pragma once

#include <chrono>
#include <future>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "pheno_module_controller/action/camera_capture.hpp"

namespace robot_ctrl
{

class CameraCaptureAction : public BT::StatefulActionNode
{
public:
  CameraCaptureAction(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  using CameraCaptureBtAction = pheno_module_controller::action::CameraCapture;
  using CameraCaptureGoalHandle = rclcpp_action::ClientGoalHandle<CameraCaptureBtAction>;

  void resetState();

  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<CameraCaptureBtAction>::SharedPtr action_client_;
  CameraCaptureGoalHandle::SharedPtr goal_handle_;
  std::shared_future<CameraCaptureGoalHandle::WrappedResult> result_future_;
  int send_goal_timeout_ms_{1000};
  std::chrono::steady_clock::time_point deadline_;
  std::chrono::milliseconds timeout_{8000};
};

}  // namespace robot_ctrl
