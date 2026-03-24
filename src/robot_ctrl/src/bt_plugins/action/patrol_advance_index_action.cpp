#include "robot_ctrl/bt_plugins/action/patrol_advance_index_action.hpp"

#include "robot_ctrl/bt_plugins/error_log_queue.hpp"

namespace robot_ctrl
{

PatrolAdvanceIndexAction::PatrolAdvanceIndexAction(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
}

BT::PortsList PatrolAdvanceIndexAction::providedPorts()
{
  return {
    BT::BidirectionalPort<int>("patrol_index", "当前巡检索引（读写）")
  };
}

BT::NodeStatus PatrolAdvanceIndexAction::tick()
{
  int patrol_index = 0;
  if (!getInput("patrol_index", patrol_index)) {
    ErrorLogQueue::instance().pushError(
      error_code::kInputMissing,
      "PatrolAdvanceIndex: missing input [patrol_index]");
    return BT::NodeStatus::FAILURE;
  }

  int next_index = patrol_index + 1;
  if (auto output_result = setOutput("patrol_index", next_index); !output_result) {
    ErrorLogQueue::instance().pushError(
      error_code::kStateConflict,
      std::string("PatrolAdvanceIndex: failed to set output [patrol_index], ") + output_result.error());
    return BT::NodeStatus::FAILURE;
  }
  RCLCPP_INFO(node_->get_logger(), "PatrolAdvanceIndex: advanced patrol_index from %d to %d", patrol_index, next_index);
  ErrorLogQueue::instance().clearLastErrorIfPrefix("PatrolAdvanceIndex:");
  return BT::NodeStatus::SUCCESS;
}

}  // namespace robot_ctrl
