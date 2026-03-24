#include "robot_ctrl/bt_plugins/action/log_error_action.hpp"
#include "rclcpp/rclcpp.hpp"

namespace robot_ctrl
{

LogErrorAction::LogErrorAction(const std::string& name,
                               const BT::NodeConfiguration& config)
    : BT::ActionNodeBase(name, config)
{
    // 打开日志文件（追加模式）
    log_file_.open("src/robot_ctrl/logs/error_log.txt", std::ios::app);
}

LogErrorAction::~LogErrorAction()
{
    if (log_file_.is_open())
        log_file_.close();
}

BT::NodeStatus LogErrorAction::tick()
{
    robot_ctrl::ErrorRecord record;
    while (ErrorLogQueue::instance().popError(record))
    {
        auto now = std::chrono::steady_clock::now();
        const std::string error_message =
            "[" + std::to_string(record.code) + "] " + record.msg;
        auto it = last_print_time_.find(error_message);

        // 如果没有记录，或者距离上次打印超过最小间隔
        if (it == last_print_time_.end() || now - it->second > min_interval_)
        {
            logError(error_message);
            last_print_time_[error_message] = now;
        }
    }

    // 永久 RUNNING
    return BT::NodeStatus::RUNNING;
}

void LogErrorAction::halt()
{
    // 不做任何操作，保持 RUNNING
}

void LogErrorAction::logError(const std::string& error_message)
{
    auto now = std::chrono::system_clock::now();
    auto now_c = std::chrono::system_clock::to_time_t(now);
    std::ostringstream timestamp_stream;
    timestamp_stream << std::put_time(std::localtime(&now_c), "%Y-%m-%d %H:%M:%S");
    log_file_ << timestamp_stream.str() << " - Error: " << error_message << std::endl;
    log_file_.flush();
    RCLCPP_INFO(rclcpp::get_logger("LogErrorAction"), "%s - Error: %s",
                timestamp_stream.str().c_str(), error_message.c_str());
}

} // namespace robot_ctrl
