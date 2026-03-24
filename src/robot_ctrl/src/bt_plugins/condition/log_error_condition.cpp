#include "robot_ctrl/bt_plugins/condition/log_error_condition.hpp"
#include "robot_ctrl/bt_plugins/error_log_queue.hpp"
#include "rclcpp/rclcpp.hpp"
#include <sstream>
#include <iomanip>
#include <chrono>
#include <filesystem>

namespace robot_ctrl
{
LogErrorCondition::LogErrorCondition(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config)
{
    if (config.blackboard && config.blackboard->get("error_log_file", log_file_path_) &&
        !log_file_path_.empty())
    {
        // 使用 bt_main 注入黑板的日志路径参数
    }
    else
    {
        log_file_path_ = "src/robot_ctrl/logs/error_log.txt";
    }

    if (!openLogFile(log_file_path_))
    {
        const std::string fallback = "src/robot_ctrl/logs/error_log.txt";
        RCLCPP_WARN(
            rclcpp::get_logger("LogErrorCondition"),
            "Open log file failed: %s, fallback to %s",
            log_file_path_.c_str(), fallback.c_str());
        log_file_path_ = fallback;
        (void)openLogFile(log_file_path_);
    }
}

LogErrorCondition::~LogErrorCondition()
{
    if (log_file_.is_open())
        log_file_.close();
}

BT::NodeStatus LogErrorCondition::tick()
{
    robot_ctrl::ErrorRecord record;

    // 遍历队列，把所有错误写入日志
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

    // 始终返回 SUCCESS
    return BT::NodeStatus::SUCCESS;
}

void LogErrorCondition::logError(const std::string& error_message)
{
    if (!log_file_.is_open())
    {
        return;
    }

    auto now = std::chrono::system_clock::now();
    auto now_c = std::chrono::system_clock::to_time_t(now);
    std::ostringstream timestamp_stream;
    timestamp_stream << std::put_time(std::localtime(&now_c), "%Y-%m-%d %H:%M:%S");
    log_file_ << timestamp_stream.str() << " - Error: " << error_message << std::endl;
}

bool LogErrorCondition::openLogFile(const std::string& log_file_path)
{
    if (log_file_.is_open())
    {
        log_file_.close();
    }
    log_file_.clear();

    std::error_code ec;
    const std::filesystem::path path(log_file_path);
    if (path.has_parent_path())
    {
        std::filesystem::create_directories(path.parent_path(), ec);
    }

    if (ec)
    {
        RCLCPP_WARN(
            rclcpp::get_logger("LogErrorCondition"),
            "Create log directory failed: %s",
            path.parent_path().string().c_str());
    }

    log_file_.open(log_file_path, std::ios::app);
    return log_file_.is_open();
}
}
