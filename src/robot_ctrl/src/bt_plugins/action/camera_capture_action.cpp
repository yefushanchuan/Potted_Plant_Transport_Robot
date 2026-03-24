#include "robot_ctrl/bt_plugins/action/camera_capture_action.hpp"

#include <chrono>

#include "rclcpp_action/exceptions.hpp"
#include "robot_ctrl/bt_plugins/error_log_queue.hpp"

using namespace std::chrono_literals;

namespace robot_ctrl
{

CameraCaptureAction::CameraCaptureAction(const std::string & name, const BT::NodeConfiguration & config)
: BT::StatefulActionNode(name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
  action_client_ = rclcpp_action::create_client<CameraCaptureBtAction>(
    node_, "/pheno_module_controller/camera_capture");
  node_->get_parameter_or("send_goal_timeout_ms", send_goal_timeout_ms_, 1000);
}

BT::PortsList CameraCaptureAction::providedPorts()
{
  return {
    BT::InputPort<std::string>("task_id", "任务ID"),
    BT::InputPort<std::string>("pot_id", "", "盆栽ID（字符串）"),
    BT::InputPort<int>("timeout_ms", 8000, "等待服务响应超时时间(毫秒)"),
  };
}

void CameraCaptureAction::resetState()
{
  goal_handle_.reset();
  result_future_ = std::shared_future<CameraCaptureGoalHandle::WrappedResult>();
  timeout_ = std::chrono::milliseconds(8000);
}

BT::NodeStatus CameraCaptureAction::onStart()
{
  resetState();

  if (!action_client_) {
    ErrorLogQueue::instance().pushError(
      error_code::kServiceUnavailable,
      "CameraCaptureAction: camera_capture action client not initialized");
    return BT::NodeStatus::FAILURE;
  }

  auto task_id_input = getInput<std::string>("task_id");
  if (!task_id_input || task_id_input->empty()) {
    ErrorLogQueue::instance().pushError(
      error_code::kInputMissing,
      "CameraCaptureAction: Missing input [task_id]");
    return BT::NodeStatus::FAILURE;
  }

  const auto pot_id_input = getInput<std::string>("pot_id");
  const std::string pot_id = pot_id_input ? pot_id_input.value() : "";

  auto timeout_ms = 8000;
  getInput("timeout_ms", timeout_ms);

  if (timeout_ms <= 0) {
    timeout_ms = 8000;
  }
  timeout_ = std::chrono::milliseconds(timeout_ms);
  deadline_ = std::chrono::steady_clock::now() + timeout_;

  if (!action_client_->wait_for_action_server(1s)) {
    ErrorLogQueue::instance().pushError(
      error_code::kServiceUnavailable,
      "CameraCaptureAction: Action server [/pheno_module_controller/camera_capture] unavailable");
    return BT::NodeStatus::FAILURE;
  }

  CameraCaptureBtAction::Goal goal;
  goal.task_id = task_id_input.value();
  goal.pot_id = pot_id;

  auto send_goal_options = rclcpp_action::Client<CameraCaptureBtAction>::SendGoalOptions();
  auto future_goal_handle = action_client_->async_send_goal(goal, send_goal_options);
  const auto send_goal_rc = rclcpp::spin_until_future_complete(
    node_, future_goal_handle, std::chrono::milliseconds(send_goal_timeout_ms_));
  if (send_goal_rc != rclcpp::FutureReturnCode::SUCCESS) {
    ErrorLogQueue::instance().pushError(
      error_code::kActionFailure,
      "CameraCaptureAction: send goal timed out");
    resetState();
    return BT::NodeStatus::FAILURE;
  }

  goal_handle_ = future_goal_handle.get();
  if (!goal_handle_) {
    ErrorLogQueue::instance().pushError(
      error_code::kActionFailure,
      "CameraCaptureAction: goal rejected");
    resetState();
    return BT::NodeStatus::FAILURE;
  }

  result_future_ = action_client_->async_get_result(goal_handle_);
  if (!result_future_.valid()) {
    ErrorLogQueue::instance().pushError(
      error_code::kActionFailure,
      "CameraCaptureAction: async_get_result failed");
    resetState();
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus CameraCaptureAction::onRunning()
{
  if (!result_future_.valid()) {
    ErrorLogQueue::instance().pushError(
      error_code::kActionFailure,
      "CameraCaptureAction: pending result invalid");
    return BT::NodeStatus::FAILURE;
  }

  if (std::chrono::steady_clock::now() > deadline_) {
    if (goal_handle_) {
      try {
        action_client_->async_cancel_goal(goal_handle_);
      } catch (const rclcpp_action::exceptions::UnknownGoalHandleError &) {
      }
    }
    ErrorLogQueue::instance().pushError(
      error_code::kTimeout,
      "CameraCaptureAction: camera_capture action timed out");
    resetState();
    return BT::NodeStatus::FAILURE;
  }

  if (result_future_.wait_for(0ms) != std::future_status::ready) {
    return BT::NodeStatus::RUNNING;
  }

  const auto wrapped_result = result_future_.get();
  if (wrapped_result.code != rclcpp_action::ResultCode::SUCCEEDED || !wrapped_result.result) {
    std::string message = "CameraCaptureAction: camera_capture action failed";
    if (wrapped_result.code == rclcpp_action::ResultCode::ABORTED) {
      message = "CameraCaptureAction: camera_capture action aborted";
    } else if (wrapped_result.code == rclcpp_action::ResultCode::CANCELED) {
      message = "CameraCaptureAction: camera_capture action canceled";
    }
    ErrorLogQueue::instance().pushError(
      error_code::kActionFailure,
      message);
    resetState();
    return BT::NodeStatus::FAILURE;
  }

  if (!wrapped_result.result->success) {
    const std::string err_msg = wrapped_result.result->error_message.empty() ?
      "CameraCaptureAction: camera_capture action failed" :
      "CameraCaptureAction: " + wrapped_result.result->error_message;
    ErrorLogQueue::instance().pushError(error_code::kActionFailure, err_msg);
    resetState();
    return BT::NodeStatus::FAILURE;
  }

  ErrorLogQueue::instance().clearLastErrorIfPrefix("CameraCaptureAction:");
  resetState();
  return BT::NodeStatus::SUCCESS;
}

void CameraCaptureAction::onHalted()
{
  if (goal_handle_) {
    try {
      action_client_->async_cancel_goal(goal_handle_);
    } catch (const rclcpp_action::exceptions::UnknownGoalHandleError &) {
    }
  }
  resetState();
}

}  // namespace robot_ctrl
