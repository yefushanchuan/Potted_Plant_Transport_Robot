#pragma once
#include <cstdint>
#include <string>
#include <nlohmann/json.hpp>

namespace robot_ctrl::roompose {

enum class RoomType : std::uint8_t {
  ORIGIN = 0,
  NORMAL = 1,
  LINK = 2,
  CHARGE = 3,
  PATH = 4,
  MARK = 5,
  WAIT = 6,
  DOOR = 7,
  OTHER = 8
};

enum class Visibility : std::uint8_t {
  HIDDEN = 0,
  PUBLIC = 1,
  ADMIN = 2
};

static constexpr int kDefaultType = static_cast<int>(RoomType::NORMAL);
static constexpr int kDefaultVisibility = static_cast<int>(Visibility::PUBLIC);
static constexpr const char* kDefaultTags = "non";

inline int clampType(int v) {
  return (v >= static_cast<int>(RoomType::ORIGIN) &&
          v <= static_cast<int>(RoomType::OTHER))
             ? v
             : kDefaultType;
}

inline int clampVisibility(int v) {
  return (v >= static_cast<int>(Visibility::HIDDEN) &&
          v <= static_cast<int>(Visibility::ADMIN))
             ? v
             : kDefaultVisibility;
}

inline int parseType(const nlohmann::json& j, const char* key,
                     int default_val = kDefaultType) {
  if (!j.contains(key)) return default_val;
  try {
    if (j[key].is_number_integer()) {
      return clampType(j[key].get<int>());
    }
    if (j[key].is_string()) {
      return clampType(std::stoi(j[key].get<std::string>()));
    }
  } catch (...) {
  }
  return default_val;
}

inline int parseVisibility(const nlohmann::json& j, const char* key,
                           int default_val = kDefaultVisibility) {
  if (!j.contains(key)) return default_val;
  try {
    if (j[key].is_number_integer()) {
      return clampVisibility(j[key].get<int>());
    }
    if (j[key].is_string()) {
      return clampVisibility(std::stoi(j[key].get<std::string>()));
    }
  } catch (...) {
  }
  return default_val;
}

inline std::string parseTags(const nlohmann::json& j, const char* key,
                             const std::string& default_val = kDefaultTags) {
  if (!j.contains(key)) return default_val;
  try {
    if (j[key].is_string()) return j[key].get<std::string>();
  } catch (...) {
  }
  return default_val;
}

}  // namespace robot_ctrl::roompose
