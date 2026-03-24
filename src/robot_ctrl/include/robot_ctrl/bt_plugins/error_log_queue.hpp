#pragma once
#include <queue>
#include <mutex>
#include <string>
#include <cstdint>

namespace robot_ctrl
{
namespace error_code
{
constexpr uint16_t kInputMissing = 1000;
constexpr uint16_t kServiceUnavailable = 1100;
constexpr uint16_t kActionFailure = 1200;
constexpr uint16_t kTimeout = 1300;
constexpr uint16_t kDataMissing = 1400;
constexpr uint16_t kInvalidParam = 1500;
constexpr uint16_t kStateConflict = 1600;
constexpr uint16_t kRecoveryFailed = 1700;
constexpr uint16_t kUnknown = 1999;
}  // namespace error_code

struct ErrorRecord
{
    uint16_t code = error_code::kUnknown;
    std::string msg;
};
}  // namespace robot_ctrl

class ErrorLogQueue
{
public:
    static ErrorLogQueue& instance()
    {
        static ErrorLogQueue queue;
        return queue;
    }

    void pushError(uint16_t code, const std::string& msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        errors_.push({code, msg});
        last_error_ = {code, msg};
        has_last_error_ = true;
    }

    bool popError(robot_ctrl::ErrorRecord& record)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if(errors_.empty()) return false;
        record = errors_.front();
        errors_.pop();
        return true;
    }

    bool getLastError(robot_ctrl::ErrorRecord& record)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!has_last_error_) {
            return false;
        }
        record = last_error_;
        return true;
    }

    bool empty()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return errors_.empty();
    }

    bool clearLastErrorIfPrefix(const std::string& prefix)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!has_last_error_) {
            return false;
        }
        if (last_error_.msg.rfind(prefix, 0) != 0) {
            return false;
        }
        last_error_ = {};
        has_last_error_ = false;
        return true;
    }

    void clearAllErrors()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        while (!errors_.empty()) {
            errors_.pop();
        }
        last_error_ = {};
        has_last_error_ = false;
    }

private:
    ErrorLogQueue() {}
    std::queue<robot_ctrl::ErrorRecord> errors_;
    robot_ctrl::ErrorRecord last_error_;
    bool has_last_error_{false};
    std::mutex mutex_;
};
