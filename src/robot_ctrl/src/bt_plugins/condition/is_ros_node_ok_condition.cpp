#include "robot_ctrl/bt_plugins/condition/is_ros_node_ok_condition.hpp"
#include "robot_ctrl/bt_plugins/error_log_queue.hpp"

namespace robot_ctrl
{

IsRosNodeOk::IsRosNodeOk(const std::string & name, const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
}

BT::NodeStatus IsRosNodeOk::tick()
{
  std::string node_name, topic_name;
  getInput("node_name", node_name);
  getInput("topic_name", topic_name);

  // === 1. 生命周期节点检查 ===
  if (!node_name.empty()) {
    std::string service_name = node_name + "/get_state";
    auto client = node_->create_client<lifecycle_msgs::srv::GetState>(service_name);

    if (client->wait_for_service(std::chrono::milliseconds(100))) {
      auto req = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
      auto future = client->async_send_request(req);

      if (rclcpp::spin_until_future_complete(node_, future, std::chrono::milliseconds(300))
          == rclcpp::FutureReturnCode::SUCCESS)
      {
        auto resp = future.get();
        if (resp->current_state.id == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
          ErrorLogQueue::instance().clearLastErrorIfPrefix("IsRosNodeOk:");
          return BT::NodeStatus::SUCCESS;
        }
      }
    }
  }

  // === 2. 普通参数服务检查 ===
  if (!node_name.empty()) {
    std::string param_service = node_name + "/describe_parameters";
    auto param_client = node_->create_client<rcl_interfaces::srv::DescribeParameters>(param_service);

    if (param_client->wait_for_service(std::chrono::milliseconds(100))) {
      ErrorLogQueue::instance().clearLastErrorIfPrefix("IsRosNodeOk:");
      return BT::NodeStatus::SUCCESS;
    }
    else
    {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
        "Parameter service '%s' is not available.", param_service.c_str());
      ErrorLogQueue::instance().pushError(
        error_code::kServiceUnavailable,
        "IsRosNodeOk: Parameter service '" + param_service + "' is not available.");
    }
  }

  // === 3. Topic 存活检查 ===
  if (!topic_name.empty()) {
    if(node_->count_publishers(topic_name) > 0)
    {
      ErrorLogQueue::instance().clearLastErrorIfPrefix("IsRosNodeOk:");
      return BT::NodeStatus::SUCCESS;
    }
    else
    {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
        "Topic '%s' has no active publishers.", topic_name.c_str());
      ErrorLogQueue::instance().pushError(
        error_code::kDataMissing,
        "IsRosNodeOk: Topic '" + topic_name + "' has no active publishers.");
    }
  }

  return BT::NodeStatus::FAILURE;
}

}  // namespace robot_ctrl
