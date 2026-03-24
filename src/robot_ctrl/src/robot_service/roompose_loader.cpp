#include <rclcpp/rclcpp.hpp>
#include <fstream>
#include <mutex>
#include <map>
#include <nlohmann/json.hpp>

#include "robot_ctrl/srv/get_roompose.hpp"
#include "robot_ctrl/roompose_defs.hpp"

using json = nlohmann::json;
namespace rp = robot_ctrl::roompose;

struct RoomPose {
  double x;
  double y;
  double yaw;
  int type;
  int visibility_level;
  std::string tags;
};

class RoomPoseLoaderNode : public rclcpp::Node {
public:
  RoomPoseLoaderNode()
  : Node("room_pose_loader_node")
  {
    declare_parameter<std::string>("roompose_file_path", "");
    get_parameter("roompose_file_path", json_file_path_);

    if (json_file_path_.empty()) {
      RCLCPP_ERROR(get_logger(), "Parameter 'roompose_file_path' not set.");
      rclcpp::shutdown();
      return;
    }

    loadJSON();

    service_ = create_service<robot_ctrl::srv::GetRoompose>(
      "/get_room_pose",
      std::bind(&RoomPoseLoaderNode::getRoomPoseCallback, this, std::placeholders::_1, std::placeholders::_2)
    );

    timer_ = create_wall_timer(
      std::chrono::seconds(5),
      std::bind(&RoomPoseLoaderNode::loadJSON, this)
    );
  }

private:
  std::string json_file_path_;
  std::map<std::string, RoomPose> room_pose_map_;
  std::mutex map_mutex_;

  rclcpp::Service<robot_ctrl::srv::GetRoompose>::SharedPtr service_;
  rclcpp::TimerBase::SharedPtr timer_;

  void getRoomPoseCallback(
    const std::shared_ptr<robot_ctrl::srv::GetRoompose::Request> req,
    std::shared_ptr<robot_ctrl::srv::GetRoompose::Response> res)
  {
    if (req->room_name.empty()) {
      // 标个失败
      res->success = false;
      return;
    }


    std::lock_guard<std::mutex> lock(map_mutex_);
    auto it = room_pose_map_.find(req->room_name);

    if (it != room_pose_map_.end()) {
      res->x = it->second.x;
      res->y = it->second.y;
      res->yaw = it->second.yaw;
      res->type = it->second.type;
      res->visibility_level = it->second.visibility_level;
      res->tags = it->second.tags;
      res->success = true;
      res->message = "Room found.";
    } else {
      res->success = false;
      res->message = "Room not found.";
    }
  }

  void loadJSON()
  {
    std::ifstream f(json_file_path_);
    if (!f.is_open()) {
      RCLCPP_ERROR(get_logger(), "Could not open JSON file: %s", json_file_path_.c_str());
      return;
    }

    json j;
    f >> j;

    std::map<std::string, RoomPose> new_map;

    for (auto &item : j["rooms"]) {
      std::string name = item["name"];
      RoomPose pose;
      pose.x = item["x"];
      pose.y = item["y"];
      pose.yaw = item["yaw"];
      pose.type = rp::parseType(item, "type");
      pose.visibility_level = rp::parseVisibility(item, "visibility_level");
      pose.tags = rp::parseTags(item, "tags");
      new_map[name] = pose;
    }

    std::lock_guard<std::mutex> lock(map_mutex_);
    room_pose_map_ = new_map;
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RoomPoseLoaderNode>());
  rclcpp::shutdown();
  return 0;
}
