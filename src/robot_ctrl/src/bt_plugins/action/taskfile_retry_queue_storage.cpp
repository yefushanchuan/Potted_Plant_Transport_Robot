#include "robot_ctrl/bt_plugins/action/taskfile_retry_queue_storage.hpp"

#include <filesystem>
#include <fstream>
#include <unordered_set>

namespace robot_ctrl::taskfile_retry_queue
{

const char * const kFailedTaskIdsKey = "failed_taskfile_task_ids";
const char * const kRetryQueueFilePathKey = "taskfile_retry_queue_file";

std::vector<std::string> getFailedTaskIds(const BT::Blackboard::Ptr & blackboard)
{
  std::vector<std::string> task_ids;
  if (blackboard) {
    blackboard->get(kFailedTaskIdsKey, task_ids);
  }
  return task_ids;
}

void setFailedTaskIds(const BT::Blackboard::Ptr & blackboard, const std::vector<std::string> & task_ids)
{
  if (blackboard) {
    blackboard->set(kFailedTaskIdsKey, task_ids);
  }
}

std::string getRetryQueueFilePath(const BT::Blackboard::Ptr & blackboard)
{
  std::string file_path;
  if (blackboard) {
    blackboard->get(kRetryQueueFilePathKey, file_path);
  }
  return file_path;
}

std::vector<std::string> deduplicateTaskIds(const std::vector<std::string> & task_ids)
{
  std::vector<std::string> deduped;
  deduped.reserve(task_ids.size());
  std::unordered_set<std::string> seen;
  for (const auto & task_id : task_ids) {
    if (task_id.empty()) {
      continue;
    }
    if (seen.insert(task_id).second) {
      deduped.push_back(task_id);
    }
  }
  return deduped;
}

bool loadFromFile(
  const std::string & file_path,
  std::vector<std::string> & task_ids,
  std::string & error_message)
{
  task_ids.clear();
  error_message.clear();

  if (file_path.empty()) {
    error_message = "retry queue file path is empty";
    return false;
  }

  const std::filesystem::path path(file_path);
  std::error_code ec;
  if (!std::filesystem::exists(path, ec)) {
    if (ec) {
      error_message = "failed to check retry queue file: " + ec.message();
      return false;
    }
    return true;
  }

  std::ifstream ifs(path);
  if (!ifs.is_open()) {
    error_message = "failed to open retry queue file for reading";
    return false;
  }

  std::string line;
  while (std::getline(ifs, line)) {
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }
    if (!line.empty()) {
      task_ids.push_back(line);
    }
  }

  if (!ifs.good() && !ifs.eof()) {
    error_message = "failed while reading retry queue file";
    return false;
  }

  task_ids = deduplicateTaskIds(task_ids);
  return true;
}

bool saveToFile(
  const std::string & file_path,
  const std::vector<std::string> & task_ids,
  std::string & error_message)
{
  error_message.clear();
  if (file_path.empty()) {
    error_message = "retry queue file path is empty";
    return false;
  }

  const std::filesystem::path path(file_path);
  const auto deduped_task_ids = deduplicateTaskIds(task_ids);

  if (path.has_parent_path()) {
    std::error_code ec;
    std::filesystem::create_directories(path.parent_path(), ec);
    if (ec) {
      error_message = "failed to create retry queue directory: " + ec.message();
      return false;
    }
  }

  std::ofstream ofs(path, std::ios::trunc);
  if (!ofs.is_open()) {
    error_message = "failed to open retry queue file for writing";
    return false;
  }

  for (const auto & task_id : deduped_task_ids) {
    ofs << task_id << '\n';
  }
  ofs.flush();

  if (!ofs.good()) {
    error_message = "failed to write retry queue file";
    return false;
  }

  return true;
}

}  // namespace robot_ctrl::taskfile_retry_queue
