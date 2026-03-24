#include "robot_ctrl/bt_plugins/action/ack_pending_taskfile_action.hpp"

#include <algorithm>

#include "robot_ctrl/bt_plugins/action/taskfile_retry_queue_storage.hpp"

namespace robot_ctrl
{

AckPendingTaskfileAction::AckPendingTaskfileAction(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
}

BT::PortsList AckPendingTaskfileAction::providedPorts()
{
  return {
    BT::InputPort<std::string>("task_id", "已重传成功任务ID")
  };
}

BT::NodeStatus AckPendingTaskfileAction::tick()
{
  auto task_id_input = getInput<std::string>("task_id");
  if (!task_id_input || task_id_input->empty()) {
    RCLCPP_WARN(node_->get_logger(), "AckPendingTaskfileAction: missing task_id, skip ack");
    return BT::NodeStatus::SUCCESS;
  }

  auto failed_task_ids = taskfile_retry_queue::getFailedTaskIds(config().blackboard);
  if (failed_task_ids.empty()) {
    return BT::NodeStatus::SUCCESS;
  }

  const std::string & task_id = task_id_input.value();
  const auto old_size = failed_task_ids.size();
  failed_task_ids.erase(
    std::remove(failed_task_ids.begin(), failed_task_ids.end(), task_id),
    failed_task_ids.end());

  if (failed_task_ids.size() != old_size) {
    taskfile_retry_queue::setFailedTaskIds(config().blackboard, failed_task_ids);

    const auto queue_file_path = taskfile_retry_queue::getRetryQueueFilePath(config().blackboard);
    if (!queue_file_path.empty()) {
      std::string error_message;
      if (!taskfile_retry_queue::saveToFile(queue_file_path, failed_task_ids, error_message)) {
        RCLCPP_WARN(
          node_->get_logger(),
          "AckPendingTaskfileAction: failed to persist retry queue [%s]: %s",
          queue_file_path.c_str(),
          error_message.c_str());
      }
    }

    RCLCPP_INFO(
      node_->get_logger(),
      "Taskfile retry upload succeeded, removed task_id: %s (pending=%zu)",
      task_id.c_str(),
      failed_task_ids.size());
  }

  return BT::NodeStatus::SUCCESS;
}

}  // namespace robot_ctrl
