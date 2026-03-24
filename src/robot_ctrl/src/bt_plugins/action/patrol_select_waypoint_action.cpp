#include "robot_ctrl/bt_plugins/action/patrol_select_waypoint_action.hpp"

#include "robot_ctrl/bt_plugins/error_log_queue.hpp"

namespace robot_ctrl
{

PatrolSelectWaypointAction::PatrolSelectWaypointAction(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
}

BT::PortsList PatrolSelectWaypointAction::providedPorts()
{
  return {
    BT::InputPort<std::vector<std::string>>("patrol_points", "完整巡检点列表"),
    BT::InputPort<int>("patrol_index", "当前巡检索引"),
    BT::OutputPort<std::string>("nav_target_room", "当前索引对应的目标房间名")
  };
}

BT::NodeStatus PatrolSelectWaypointAction::tick()
{
  std::vector<std::string> patrol_points;
  if (!getInput("patrol_points", patrol_points)) {
    ErrorLogQueue::instance().pushError(
      error_code::kInputMissing,
      "PatrolSelectWaypoint: missing input [patrol_points]");
    return BT::NodeStatus::FAILURE;
  }

  int patrol_index = 0;
  if (!getInput("patrol_index", patrol_index)) {
    ErrorLogQueue::instance().pushError(
      error_code::kInputMissing,
      "PatrolSelectWaypoint: missing input [patrol_index]");
    return BT::NodeStatus::FAILURE;
  }

  if (patrol_index < 0 || patrol_index >= static_cast<int>(patrol_points.size())) {
    ErrorLogQueue::instance().pushError(
      error_code::kInvalidParam,
      "PatrolSelectWaypoint: patrol_index out of range");
    return BT::NodeStatus::FAILURE;
  }

  const auto & room = patrol_points.at(static_cast<std::size_t>(patrol_index));
  if (room.empty()) {
    ErrorLogQueue::instance().pushError(
      error_code::kInvalidParam,
      "PatrolSelectWaypoint: selected room is empty");
    return BT::NodeStatus::FAILURE;
  }

  setOutput("nav_target_room", room);
  ErrorLogQueue::instance().clearLastErrorIfPrefix("PatrolSelectWaypoint:");
  return BT::NodeStatus::SUCCESS;
}

}  // namespace robot_ctrl
