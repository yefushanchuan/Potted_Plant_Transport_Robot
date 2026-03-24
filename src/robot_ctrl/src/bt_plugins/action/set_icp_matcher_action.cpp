#include "robot_ctrl/bt_plugins/action/set_icp_matcher_action.hpp"
#include "robot_ctrl/bt_plugins/error_log_queue.hpp"

using namespace std::chrono_literals;

namespace robot_ctrl
{
SetIcpActive::SetIcpActive(const std::string & name, const BT::NodeConfiguration & config)
: BT::StatefulActionNode(name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
  client_ = node_->create_client<std_srvs::srv::SetBool>("/set_icp_active");
}

BT::PortsList SetIcpActive::providedPorts()
{
  return { BT::InputPort<bool>("icp_active") };
}

BT::NodeStatus SetIcpActive::onStart()
{
  // 每次进入节点都重置 future
  future_ = {};

  auto res = getInput<bool>("icp_active");
  if (!res)
  {
    RCLCPP_ERROR(node_->get_logger(), "Missing blackboard input [icp_active]");
    ErrorLogQueue::instance().pushError(
      error_code::kInputMissing,
      "SetIcpActive: Missing input [icp_active]");
    return BT::NodeStatus::FAILURE;
  }

  if (!client_->wait_for_service(0s))   // 非阻塞检测
  {
    RCLCPP_ERROR(node_->get_logger(), "Service [/icp_active] not available");
    ErrorLogQueue::instance().pushError(
      error_code::kServiceUnavailable,
      "SetIcpActive: Service [/icp_active] not available");
    return BT::NodeStatus::FAILURE;
  }

  auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
  req->data = res.value();

  future_ = client_->async_send_request(req).future.share();
  return BT::NodeStatus::RUNNING;   // 转到 onRunning
}

BT::NodeStatus SetIcpActive::onRunning()
{
  if (!future_.valid()){
    ErrorLogQueue::instance().pushError(
      error_code::kActionFailure,
      "SetIcpActive: service not valid");
    return BT::NodeStatus::FAILURE;
  }

  // 非阻塞检查
  if (future_.wait_for(0s) != std::future_status::ready)
    return BT::NodeStatus::RUNNING;

  auto resp = future_.get();
  if (!resp){
    ErrorLogQueue::instance().pushError(
      error_code::kActionFailure,
      "SetIcpActive: service no respence");
    return BT::NodeStatus::FAILURE;
  }

  ErrorLogQueue::instance().clearLastErrorIfPrefix("SetIcpActive:");
  return BT::NodeStatus::SUCCESS;
}

void SetIcpActive::onHalted()
{
  future_ = {};
}
}
