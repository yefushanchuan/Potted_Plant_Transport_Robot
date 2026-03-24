#include "robot_ctrl/bt_plugins/action/get_pending_taskfile_action.hpp"

#include "robot_ctrl/bt_plugins/action/taskfile_retry_queue_storage.hpp"

namespace robot_ctrl
{

GetPendingTaskfileAction::GetPendingTaskfileAction(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
}

BT::PortsList GetPendingTaskfileAction::providedPorts()
{
  return {
    BT::OutputPort<std::string>("task_id", "待重传任务ID")
  };
}

BT::NodeStatus GetPendingTaskfileAction::tick()
{
  const auto failed_task_ids = taskfile_retry_queue::getFailedTaskIds(config().blackboard);
  if (failed_task_ids.empty()) {
    return BT::NodeStatus::FAILURE;
  }

  const auto & task_id = failed_task_ids.front();
  setOutput("task_id", task_id);
  RCLCPP_INFO(
    node_->get_logger(),
    "Retrying pending taskfile upload for task_id: %s (pending=%zu)",
    task_id.c_str(),
    failed_task_ids.size());
  return BT::NodeStatus::SUCCESS;
}

}  // namespace robot_ctrl
