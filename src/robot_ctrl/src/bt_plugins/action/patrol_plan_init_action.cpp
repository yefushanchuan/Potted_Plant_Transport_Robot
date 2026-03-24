#include "robot_ctrl/bt_plugins/action/patrol_plan_init_action.hpp"

#include "robot_ctrl/bt_plugins/error_log_queue.hpp"

namespace robot_ctrl
{

PatrolPlanInitAction::PatrolPlanInitAction(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
}

BT::PortsList PatrolPlanInitAction::providedPorts()
{
  return {
    BT::InputPort<std::vector<std::string>>("waypoint_rooms", "巡检途径点列表（按顺序）"),
    BT::OutputPort<std::vector<std::string>>("patrol_points", "展开后的完整巡检点列表"),
    BT::OutputPort<int>("patrol_count", "巡检总点数"),
    BT::OutputPort<int>("patrol_index", "当前巡检索引（初始化为0）")
  };
}

BT::NodeStatus PatrolPlanInitAction::tick()
{
  std::vector<std::string> waypoint_rooms;
  if (!getInput("waypoint_rooms", waypoint_rooms)) {
    ErrorLogQueue::instance().pushError(
      error_code::kInputMissing,
      "PatrolPlanInit: missing input [waypoint_rooms]");
    return BT::NodeStatus::FAILURE;
  }

  std::vector<std::string> patrol_points;
  patrol_points.reserve(waypoint_rooms.size());
  for (const auto & room : waypoint_rooms) {
    if (!room.empty()) {
      patrol_points.push_back(room);
    }
  }

  if (patrol_points.empty()) {
    ErrorLogQueue::instance().pushError(
      error_code::kInvalidParam,
      "PatrolPlanInit: waypoint_rooms is empty");
    return BT::NodeStatus::FAILURE;
  }

  setOutput("patrol_points", patrol_points);
  setOutput("patrol_count", static_cast<int>(patrol_points.size()));
  setOutput("patrol_index", 0);
  RCLCPP_INFO(node_->get_logger(), "PatrolPlanInit: initialized patrol plan with %zu points", patrol_points.size());
  ErrorLogQueue::instance().clearLastErrorIfPrefix("PatrolPlanInit:");
  return BT::NodeStatus::SUCCESS;
}

}  // namespace robot_ctrl
