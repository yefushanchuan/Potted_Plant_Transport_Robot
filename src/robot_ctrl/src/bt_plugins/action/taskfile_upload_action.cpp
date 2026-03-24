#include "robot_ctrl/bt_plugins/action/taskfile_upload_action.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>

#include "rclcpp_action/exceptions.hpp"
#include "robot_ctrl/bt_plugins/error_log_queue.hpp"

namespace robot_ctrl
{
namespace
{

bool isSafeTaskId(const std::string & task_id)
{
  if (task_id.empty()) {
    return false;
  }
  return std::all_of(task_id.begin(), task_id.end(), [](unsigned char c) {
    return std::isalnum(c) != 0 || c == '_' || c == '-';
  });
}

}  // namespace

TaskfileUploadAction::TaskfileUploadAction(const std::string & name, const BT::NodeConfiguration & config)
: BT::StatefulActionNode(name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
  action_client_ = rclcpp_action::create_client<BtTaskFileUpload>(node_, "/robot_mqtt_bridge/taskfile_upload");
  node_->get_parameter_or("send_goal_timeout_ms", send_goal_timeout_ms_, 1000);
}

BT::PortsList TaskfileUploadAction::providedPorts()
{
  return {
    BT::InputPort<std::string>("task_id", "任务ID"),
    BT::InputPort<int>("timeout_ms", 1200, "调用 taskfile_upload action 超时时间(毫秒)"),
    BT::OutputPort<std::string>("result_message", "执行结果文本")
  };
}

void TaskfileUploadAction::setResultMessage(const std::string & message)
{
  setOutput("result_message", message);
  if (config().blackboard) {
    config().blackboard->set("taskfile_upload_result_message", message);
  }
}

void TaskfileUploadAction::resetState()
{
  goal_handle_.reset();
  result_future_ = std::shared_future<BtTaskFileUploadGoalHandle::WrappedResult>();
  latest_progress_ = 0.0F;
  timeout_ = std::chrono::milliseconds(120000);
}

BT::NodeStatus TaskfileUploadAction::onStart()
{
  resetState();

  if (!action_client_) {
    ErrorLogQueue::instance().pushError(
      error_code::kServiceUnavailable,
      "TaskfileUploadAction: bt taskfile_upload client not initialized");
    setResultMessage("taskfile_upload action client not initialized");
    return BT::NodeStatus::FAILURE;
  }

  auto task_id_input = getInput<std::string>("task_id");
  if (!task_id_input || task_id_input->empty()) {
    ErrorLogQueue::instance().pushError(
      error_code::kInputMissing,
      "TaskfileUploadAction: Missing input [task_id]");
    setResultMessage("missing task_id");
    return BT::NodeStatus::FAILURE;
  }

  const std::string task_id = task_id_input.value();
  if (!isSafeTaskId(task_id)) {
    ErrorLogQueue::instance().pushError(
      error_code::kInvalidParam,
      "TaskfileUploadAction: task_id contains unsupported characters");
    setResultMessage("task_id contains unsupported characters");
    return BT::NodeStatus::FAILURE;
  }

  int timeout_ms = 120000;
  getInput("timeout_ms", timeout_ms);
  if (timeout_ms <= 0) {
    timeout_ms = 120000;
  }
  timeout_ = std::chrono::milliseconds(timeout_ms);
  deadline_ = std::chrono::steady_clock::now() + timeout_;

  if (!action_client_->wait_for_action_server(std::chrono::seconds(1))) {
    ErrorLogQueue::instance().pushError(
      error_code::kServiceUnavailable,
      "TaskfileUploadAction: action server [/robot_mqtt_bridge/taskfile_upload] unavailable");
    setResultMessage("action server unavailable");
    return BT::NodeStatus::FAILURE;
  }

  BtTaskFileUpload::Goal goal;
  goal.task_id = task_id;

  auto send_goal_options = rclcpp_action::Client<BtTaskFileUpload>::SendGoalOptions();
  send_goal_options.feedback_callback =
    [this](
    BtTaskFileUploadGoalHandle::SharedPtr,
    const std::shared_ptr<const BtTaskFileUpload::Feedback> feedback) {
      if (!feedback) {
        return;
      }
      std::lock_guard<std::mutex> lock(feedback_mutex_);
      latest_progress_ = feedback->progress_percent;
    };

  auto future_goal_handle = action_client_->async_send_goal(goal, send_goal_options);
  const auto wait_result = rclcpp::spin_until_future_complete(
    node_,
    future_goal_handle,
    std::chrono::milliseconds(send_goal_timeout_ms_));
  if (wait_result != rclcpp::FutureReturnCode::SUCCESS) {
    ErrorLogQueue::instance().pushError(
      error_code::kActionFailure,
      "TaskfileUploadAction: send goal timed out");
    setResultMessage("send goal timed out");
    return BT::NodeStatus::FAILURE;
  }

  goal_handle_ = future_goal_handle.get();
  if (!goal_handle_) {
    ErrorLogQueue::instance().pushError(
      error_code::kActionFailure,
      "TaskfileUploadAction: goal rejected");
    setResultMessage("goal rejected");
    return BT::NodeStatus::FAILURE;
  }

  result_future_ = action_client_->async_get_result(goal_handle_);
  setResultMessage("taskfile_upload accepted");
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus TaskfileUploadAction::onRunning()
{
  if (!result_future_.valid()) {
    ErrorLogQueue::instance().pushError(
      error_code::kActionFailure,
      "TaskfileUploadAction: pending result invalid");
    setResultMessage("pending result invalid");
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
      "TaskfileUploadAction: action timeout");
    setResultMessage("taskfile_upload timeout");
    resetState();
    return BT::NodeStatus::FAILURE;
  }

  if (result_future_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) {
    float progress = 0.0F;
    {
      std::lock_guard<std::mutex> lock(feedback_mutex_);
      progress = latest_progress_;
    }
    setResultMessage("taskfile_upload running, progress=" + std::to_string(progress));
    return BT::NodeStatus::RUNNING;
  }

  const auto wrapped_result = result_future_.get();

  if (
    wrapped_result.code != rclcpp_action::ResultCode::SUCCEEDED ||
    !wrapped_result.result ||
    !wrapped_result.result->success)
  {
    std::string message = "taskfile_upload failed";
    if (wrapped_result.code == rclcpp_action::ResultCode::ABORTED) {
      message = "taskfile_upload aborted";
    } else if (wrapped_result.code == rclcpp_action::ResultCode::CANCELED) {
      message = "taskfile_upload canceled";
    }
    ErrorLogQueue::instance().pushError(
      error_code::kActionFailure,
      "TaskfileUploadAction: " + message);
    setResultMessage(message);
    resetState();
    return BT::NodeStatus::FAILURE;
  }

  ErrorLogQueue::instance().clearLastErrorIfPrefix("TaskfileUploadAction:");
  setResultMessage("taskfile_upload success");
  resetState();
  return BT::NodeStatus::SUCCESS;
}

void TaskfileUploadAction::onHalted()
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
