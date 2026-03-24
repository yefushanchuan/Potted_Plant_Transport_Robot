#ifndef LOG_ERROR_CONDITION_HPP
#define LOG_ERROR_CONDITION_HPP

#include <behaviortree_cpp_v3/behavior_tree.h>
#include <fstream>
#include <iomanip>
#include <chrono>
#include <ctime>
#include "robot_ctrl/bt_plugins/error_log_queue.hpp"  // 引入错误队列头文件

namespace robot_ctrl
{
class LogErrorCondition : public BT::ConditionNode
{
public:
    LogErrorCondition(const std::string& name, const BT::NodeConfiguration& config);
    ~LogErrorCondition();

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts()
    {
        return {};
    }

private:
    std::ofstream log_file_;
    std::string log_file_path_;
    bool openLogFile(const std::string& log_file_path);
    void logError(const std::string& error_message);

    std::unordered_map<std::string, std::chrono::steady_clock::time_point> last_print_time_;
    std::chrono::seconds min_interval_ = std::chrono::seconds(120); // 同一错误最小间隔
};

} // namespace robot_ctrl

#endif // LOG_ERROR_CONDITION_HPP
