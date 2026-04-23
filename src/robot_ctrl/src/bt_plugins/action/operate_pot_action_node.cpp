#include "robot_ctrl/bt_plugins/action/operate_pot_action_node.hpp"
#include "rclcpp/rclcpp.hpp"

namespace robot_ctrl
{

OperatePotAction::OperatePotAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<ActionT>(xml_tag_name, action_name, conf)
{}

BT::PortsList OperatePotAction::providedPorts()
{
  return providedBasicPorts({
    BT::InputPort<int>("action_type", "动作类型: 1(搬运), 2(卸载), 3(slave)"),
    BT::InputPort<int>("rack_index", "架子索引"),
    BT::OutputPort<std::string>("message", "底层返回的信息")
  });
}

void OperatePotAction::on_tick()
{
  int action_type = 0;
  int rack_index = 0;

  if (!getInput("action_type", action_type)) {
    RCLCPP_WARN(node_->get_logger(), "Missing input [action_type], using default 0");
  }

  if (!getInput("rack_index", rack_index)) {
    RCLCPP_WARN(node_->get_logger(), "Missing input [rack_index], using default 0");
  }

  goal_.action_type = action_type;
  goal_.rack_index = rack_index;

  RCLCPP_INFO(node_->get_logger(),
              "OperatePot Goal: type=%d, rack=%d",
              action_type, rack_index);
}

BT::NodeStatus OperatePotAction::on_success()
{
  RCLCPP_INFO(node_->get_logger(), "OperatePot Action Success!");

  if (result_.result) {
    setOutput("message", result_.result->message);
  } else {
    RCLCPP_WARN(node_->get_logger(), "OperatePot result is null");
  }

  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus OperatePotAction::on_aborted()
{
  RCLCPP_ERROR(node_->get_logger(), "OperatePot Action Aborted!");
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus OperatePotAction::on_cancelled()
{
  RCLCPP_WARN(node_->get_logger(), "OperatePot Action Cancelled!");
  return BT::NodeStatus::SUCCESS;  // 可按需求改成 FAILURE
}

}  // namespace robot_ctrl
