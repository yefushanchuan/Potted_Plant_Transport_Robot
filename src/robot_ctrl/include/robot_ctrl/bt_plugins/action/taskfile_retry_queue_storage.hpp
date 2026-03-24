#pragma once

#include <string>
#include <vector>

#include "behaviortree_cpp_v3/blackboard.h"

namespace robot_ctrl::taskfile_retry_queue
{

extern const char * const kFailedTaskIdsKey;
extern const char * const kRetryQueueFilePathKey;

std::vector<std::string> getFailedTaskIds(const BT::Blackboard::Ptr & blackboard);
void setFailedTaskIds(const BT::Blackboard::Ptr & blackboard, const std::vector<std::string> & task_ids);

std::string getRetryQueueFilePath(const BT::Blackboard::Ptr & blackboard);

std::vector<std::string> deduplicateTaskIds(const std::vector<std::string> & task_ids);

bool loadFromFile(
  const std::string & file_path,
  std::vector<std::string> & task_ids,
  std::string & error_message);

bool saveToFile(
  const std::string & file_path,
  const std::vector<std::string> & task_ids,
  std::string & error_message);

}  // namespace robot_ctrl::taskfile_retry_queue
