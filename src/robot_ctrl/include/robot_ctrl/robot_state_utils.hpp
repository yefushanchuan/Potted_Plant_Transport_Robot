#pragma once

#include <algorithm>
#include <cctype>
#include <string>
#include <unordered_map>

#include "robot_ctrl/msg/robot_state.hpp"

namespace robot_ctrl
{
namespace robot_state
{

struct RobotStateInfo
{
  uint16_t value;
  const char * label;
};

// 统一维护所有状态定义，避免多处重复填写
#define ROBOT_CTRL_STATE_TABLE(X)              \
  X(UNKNOWN, STATE_UNKNOWN)                    \
  X(INIT, STATE_INIT)                          \
  X(INITCHECK, STATE_INITCHECK)                \
  X(INITERROR, STATE_INITERROR)                \
  X(IDLE, STATE_IDLE)                          \
  X(NAVIGATING, STATE_NAVIGATING)              \
  X(NAVERROR, STATE_NAVERROR)                  \
  X(DOCKING, STATE_DOCKING)                    \
  X(UNDOCKING, STATE_UNDOCKING)                \
  X(CHARGING, STATE_CHARGING)                  \
  X(CHARGEERROR, STATE_CHARGEERROR)            \
  X(STOPCHARGEERROR, STATE_STOPCHARGEERROR)    \
  X(IRRIGATING, STATE_IRRIGATING)              \
  X(IRRIGATIONERROR, STATE_IRRIGATIONERROR)    \
  X(PATROLING, STATE_PATROLING)                \
  X(MAPPING, STATE_MAPPING)                    \
  X(PHENOTYPING, STATE_PHENOTYPING)            \
  X(TRANSPORTING, STATE_TRANSPORTING)          \
  X(BATTERYLOW, STATE_BATTERYLOW)              \
  X(PATROLERROR, STATE_PATROLERROR)            \
  X(PHENOTYPEERROR, STATE_PHENOTYPEERROR)      \
  X(TRANSPORTERROR, STATE_TRANSPORTERROR)      \
  X(RESTART, STATE_RESTART)                    \
  X(RESTARTERROR, STATE_RESTARTERROR)          \
  X(STOP, STATE_STOP)

// 常见的兼容写法/别名统一放在这里
#define ROBOT_CTRL_STATE_ALIAS_TABLE(X)        \
  X(NAVERR, STATE_NAVERROR, "NAVERROR")        \
  X(IDEL, STATE_IDLE, "IDLE")

inline std::string normalize(const std::string & name)
{
  std::string normalized;
  normalized.reserve(name.size());
  for (unsigned char ch : name)
  {
    if (ch == 0x5F || ch == 0x2D || std::isspace(ch))
    {
      continue;
    }
    normalized.push_back(static_cast<char>(std::toupper(static_cast<int>(ch))));
  }
  if (normalized.empty())
  {
    normalized = "UNKNOWN";
  }
  return normalized;
}

inline const std::unordered_map<std::string, RobotStateInfo> & lookup()
{
  static const std::unordered_map<std::string, RobotStateInfo> map = []()
  {
    std::unordered_map<std::string, RobotStateInfo> table;
    table.reserve(32);
    auto add_entry = [&table](const char * key, uint16_t value, const char * label)
    {
      table.emplace(key, RobotStateInfo{value, label});
    };

#define ROBOT_CTRL_STATE_INSERT(name, constant) add_entry(#name, msg::RobotState::constant, #name);
    ROBOT_CTRL_STATE_TABLE(ROBOT_CTRL_STATE_INSERT)
#undef ROBOT_CTRL_STATE_INSERT

#define ROBOT_CTRL_STATE_ALIAS_INSERT(name, constant, label) add_entry(#name, msg::RobotState::constant, label);
    ROBOT_CTRL_STATE_ALIAS_TABLE(ROBOT_CTRL_STATE_ALIAS_INSERT)
#undef ROBOT_CTRL_STATE_ALIAS_INSERT

    return table;
  }();
  return map;
}

inline bool fromString(const std::string & name,
                       msg::RobotState & state_msg,
                       std::string * error = nullptr)
{
  const auto normalized = normalize(name);
  const auto & table = lookup();
  auto it = table.find(normalized);
  if (it == table.end())
  {
    if (error)
    {
      *error = normalized;
    }
    return false;
  }
  state_msg.state = it->second.value;
  state_msg.label = it->second.label;
  return true;
}

inline msg::RobotState fromStringOrDefault(const std::string & name,
                                           const msg::RobotState & fallback)
{
  msg::RobotState msg = fallback;
  fromString(name, msg);
  return msg;
}

}  // namespace robot_state
}  // namespace robot_ctrl
