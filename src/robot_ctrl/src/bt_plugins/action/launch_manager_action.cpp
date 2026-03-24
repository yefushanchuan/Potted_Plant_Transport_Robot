#include "robot_ctrl/bt_plugins/action/launch_manager_action.hpp"

#include <algorithm>
#include <cctype>

#include "robot_ctrl/bt_plugins/error_log_queue.hpp"

namespace robot_ctrl
{
namespace
{

const std::map<std::string, std::string> kCommandToService {
  {"start", "/launch_manager/start"},
  {"stop", "/launch_manager/stop"},
  {"restart", "/launch_manager/restart"},
  {"start_mapping", "/launch_manager/start_mapping"},
  {"start_navigation", "/launch_manager/start_navigation"},
  {"mapping", "/launch_manager/start_mapping"},
  {"navigation", "/launch_manager/start_navigation"}
};

std::string normalizeCommand(std::string command)
{
  std::transform(command.begin(), command.end(), command.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return command;
}

bool isIdempotentSuccess(const std::string & command, const std::string & message)
{
  if (command == "start" && message.find("already running") != std::string::npos) {
    return true;
  }
  if (command == "stop" && message.find("not running") != std::string::npos) {
    return true;
  }
  if ((command == "start_mapping" || command == "mapping") &&
    message.find("Already running in mapping") != std::string::npos) {
    return true;
  }
  if ((command == "start_navigation" || command == "navigation") &&
    message.find("Already running in navigation") != std::string::npos) {
    return true;
  }
  return false;
}

}  // namespace

LaunchManagerAction::LaunchManagerAction(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::StatefulActionNode(name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
}

BT::PortsList LaunchManagerAction::providedPorts()
{
  return {
    BT::InputPort<std::string>(
      "command",
      "start",
      "要发送给 launch_manager 的指令（start/stop/restart/start_mapping/start_navigation）"),
    BT::InputPort<int>(
      "timeout_ms",
      5000,
      "等待服务响应的超时时间，单位毫秒"),
    BT::OutputPort<std::string>(
      "result_message",
      "launch_manager 返回的信息")
  };
}

rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr LaunchManagerAction::getClient(
  const std::string & service_name)
{
  auto it = clients_.find(service_name);
  if (it != clients_.end()) {
    return it->second;
  }

  auto client = node_->create_client<std_srvs::srv::Trigger>(service_name);
  clients_.emplace(service_name, client);
  return client;
}

void LaunchManagerAction::resetPendingRequest()
{
  pending_future_ = rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture();
  pending_service_.clear();
  pending_command_.clear();
  timeout_ = std::chrono::milliseconds(5000);
}

BT::NodeStatus LaunchManagerAction::onStart()
{
  resetPendingRequest();

  auto command_input = getInput<std::string>("command");
  if (!command_input) {
    RCLCPP_ERROR(node_->get_logger(), "[LaunchManagerAction] Missing input port command");
    ErrorLogQueue::instance().pushError(
      error_code::kInputMissing,
      "LaunchManagerAction: Missing input [command]");
    return BT::NodeStatus::FAILURE;
  }
  pending_command_ = normalizeCommand(command_input.value());

  auto service_it = kCommandToService.find(pending_command_);
  if (service_it == kCommandToService.end()) {
    RCLCPP_ERROR(node_->get_logger(), "[LaunchManagerAction] Unknown command %s", pending_command_.c_str());
    ErrorLogQueue::instance().pushError(
      error_code::kInvalidParam,
      "LaunchManagerAction: Unknown command " + pending_command_);
    return BT::NodeStatus::FAILURE;
  }
  pending_service_ = service_it->second;

  int timeout_ms = 5000;
  getInput("timeout_ms", timeout_ms);
  if (timeout_ms <= 0) {
    timeout_ms = 5000;
  }
  timeout_ = std::chrono::milliseconds(timeout_ms);
  deadline_ = std::chrono::steady_clock::now() + timeout_;

  auto client = getClient(pending_service_);
  if (!client) {
    RCLCPP_ERROR(node_->get_logger(), "[LaunchManagerAction] Failed to create client %s",
      pending_service_.c_str());
    ErrorLogQueue::instance().pushError(
      error_code::kActionFailure,
      "LaunchManagerAction: Failed to create client " + pending_service_);
    return BT::NodeStatus::FAILURE;
  }

  if (!client->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_WARN(node_->get_logger(), "[LaunchManagerAction] Service %s unavailable",
      pending_service_.c_str());
    ErrorLogQueue::instance().pushError(
      error_code::kServiceUnavailable,
      "LaunchManagerAction: Service unavailable " + pending_service_);
    return BT::NodeStatus::FAILURE;
  }

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto future_and_request = client->async_send_request(request);
  pending_future_ = future_and_request.share();
  if (!pending_future_.valid()) {
    RCLCPP_ERROR(node_->get_logger(), "[LaunchManagerAction] Failed to send request to %s",
      pending_service_.c_str());
    ErrorLogQueue::instance().pushError(
      error_code::kActionFailure,
      "LaunchManagerAction: Failed to send request to " + pending_service_);
    resetPendingRequest();
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(node_->get_logger(), "[LaunchManagerAction] Send %s to %s",
    pending_command_.c_str(), pending_service_.c_str());
  return BT::NodeStatus::RUNNING;
}

rclcpp::FutureReturnCode LaunchManagerAction::pollFuture()
{
  if (!pending_future_.valid()) {
    return rclcpp::FutureReturnCode::INTERRUPTED;
  }
  return rclcpp::spin_until_future_complete(
    node_, pending_future_, std::chrono::milliseconds(1));
}

BT::NodeStatus LaunchManagerAction::onRunning()
{
  if (!pending_future_.valid()) {
    ErrorLogQueue::instance().pushError(
      error_code::kActionFailure,
      "LaunchManagerAction: Pending future invalid");
    return BT::NodeStatus::FAILURE;
  }

  if (std::chrono::steady_clock::now() > deadline_) {
    RCLCPP_ERROR(node_->get_logger(),
      "[LaunchManagerAction] Command %s timed out after %ld ms",
      pending_command_.c_str(), timeout_.count());
    ErrorLogQueue::instance().pushError(
      error_code::kTimeout,
      "LaunchManagerAction: Command " + pending_command_ + " timed out");
    resetPendingRequest();
    return BT::NodeStatus::FAILURE;
  }

  auto status = pollFuture();
  if (status == rclcpp::FutureReturnCode::SUCCESS) {
    auto response = pending_future_.get();
    setOutput("result_message", response->message);
    if (!response->success && !isIdempotentSuccess(pending_command_, response->message)) {
      RCLCPP_WARN(node_->get_logger(),
        "[LaunchManagerAction] %s returned failure: %s",
        pending_service_.c_str(), response->message.c_str());
      ErrorLogQueue::instance().pushError(
        error_code::kActionFailure,
        "LaunchManagerAction: " + pending_service_ + " failed - " + response->message);
      resetPendingRequest();
      return BT::NodeStatus::FAILURE;
    }

    RCLCPP_INFO(node_->get_logger(),
      "[LaunchManagerAction] Command %s succeeded: %s",
      pending_command_.c_str(), response->message.c_str());
    ErrorLogQueue::instance().clearLastErrorIfPrefix("LaunchManagerAction:");
    resetPendingRequest();
    return BT::NodeStatus::SUCCESS;
  }

  if (status == rclcpp::FutureReturnCode::TIMEOUT) {
    return BT::NodeStatus::RUNNING;
  }

  RCLCPP_ERROR(node_->get_logger(),
    "[LaunchManagerAction] Command %s interrupted", pending_command_.c_str());
  ErrorLogQueue::instance().pushError(
    error_code::kActionFailure,
    "LaunchManagerAction: Command " + pending_command_ + " interrupted");
  resetPendingRequest();
  return BT::NodeStatus::FAILURE;
}

void LaunchManagerAction::onHalted()
{
  if (!pending_command_.empty()) {
    RCLCPP_INFO(node_->get_logger(), "[LaunchManagerAction] Halted command %s",
      pending_command_.c_str());
  }
  resetPendingRequest();
}

}  // namespace robot_ctrl
