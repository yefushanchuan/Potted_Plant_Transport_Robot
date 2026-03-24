#include "robot_ctrl/bt_plugins/action/record_failed_taskfile_action.hpp"

#include <algorithm>
#include <vector>

#include "robot_ctrl/bt_plugins/action/taskfile_retry_queue_storage.hpp"

namespace robot_ctrl
{

RecordFailedTaskfileAction::RecordFailedTaskfileAction(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
}

BT::PortsList RecordFailedTaskfileAction::providedPorts()
{
  return {
    BT::InputPort<std::string>("task_id", "失败待重传任务ID")
  };
}

BT::NodeStatus RecordFailedTaskfileAction::tick()
{
  auto task_id_input = getInput<std::string>("task_id");
  if (!task_id_input || task_id_input->empty()) {
    RCLCPP_WARN(node_->get_logger(), "RecordFailedTaskfileAction: missing task_id, skip record");
    return BT::NodeStatus::SUCCESS;
  }

  const std::string & task_id = task_id_input.value();
  auto failed_task_ids = taskfile_retry_queue::getFailedTaskIds(config().blackboard);
  if (std::find(failed_task_ids.begin(), failed_task_ids.end(), task_id) == failed_task_ids.end()) {
    failed_task_ids.push_back(task_id);
    taskfile_retry_queue::setFailedTaskIds(config().blackboard, failed_task_ids);

    const auto queue_file_path = taskfile_retry_queue::getRetryQueueFilePath(config().blackboard);
    if (!queue_file_path.empty()) {
      std::string error_message;
      if (!taskfile_retry_queue::saveToFile(queue_file_path, failed_task_ids, error_message)) {
        RCLCPP_WARN(
          node_->get_logger(),
          "RecordFailedTaskfileAction: failed to persist retry queue [%s]: %s",
          queue_file_path.c_str(),
          error_message.c_str());
      }
    }

    RCLCPP_WARN(
      node_->get_logger(),
      "TaskfileUpload failed, queued task_id for retry: %s (pending=%zu)",
      task_id.c_str(),
      failed_task_ids.size());
  }

  return BT::NodeStatus::SUCCESS;
}

}  // namespace robot_ctrl
