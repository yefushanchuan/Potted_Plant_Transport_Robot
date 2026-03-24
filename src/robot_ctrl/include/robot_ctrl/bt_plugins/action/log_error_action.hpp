#ifndef LOG_ERROR_ACTION_HPP
#define LOG_ERROR_ACTION_HPP

#include <behaviortree_cpp_v3/action_node.h>
#include <fstream>
#include <sstream>
#include <chrono>
#include <iomanip>
#include <unordered_map>
#include "robot_ctrl/bt_plugins/error_log_queue.hpp"

namespace robot_ctrl
{

class LogErrorAction : public BT::ActionNodeBase
{
public:
    LogErrorAction(const std::string& name, const BT::NodeConfiguration& config);
    ~LogErrorAction();

    BT::NodeStatus tick() override;
    void halt() override;

    static BT::PortsList providedPorts()
    {
        return {};
    }

private:
    std::ofstream log_file_;
    void logError(const std::string& error_message);

    // 去重机制：记录每条错误上次打印时间
    std::unordered_map<std::string, std::chrono::steady_clock::time_point> last_print_time_;
    std::chrono::seconds min_interval_ = std::chrono::seconds(120); // 同一错误最小间隔
};

} // namespace robot_ctrl

#endif // LOG_ERROR_ACTION_HPP
