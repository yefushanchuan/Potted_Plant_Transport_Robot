#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <robot_ctrl/srv/recorder_pose.hpp>
#include "robot_ctrl/roompose_defs.hpp"

#include <fstream>
#include <iomanip>
#include <nlohmann/json.hpp>

using json = nlohmann::json;
namespace rp = robot_ctrl::roompose;

struct RoomPose {
    int id;
    double x, y, yaw;
    std::string name;
    int type;
    int visibility_level;
    std::string tags;
};

class PoseRecorderNode : public rclcpp::Node {
public:
    PoseRecorderNode() : Node("pose_recorder_json") {
        declare_parameter<std::string>("roompose_file_path", "");
        get_parameter("roompose_file_path", json_file_path_);

        if (json_file_path_.empty()) {
            RCLCPP_ERROR(get_logger(), "Missing 'roompose_file_path'");
            rclcpp::shutdown();
        }

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        srv_ = create_service<robot_ctrl::srv::RecorderPose>(
            "/record_pose",
            std::bind(&PoseRecorderNode::recordPoseCallback, this, std::placeholders::_1, std::placeholders::_2)
        );
    }

private:
    std::string json_file_path_;
    rclcpp::Service<robot_ctrl::srv::RecorderPose>::SharedPtr srv_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    void recordPoseCallback(
        const std::shared_ptr<robot_ctrl::srv::RecorderPose::Request> req,
        std::shared_ptr<robot_ctrl::srv::RecorderPose::Response> res)
    {
        geometry_msgs::msg::TransformStamped t;
        try {
            t = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
        } catch (...) {
            res->success = false;
            res->message = "TF lookup failed";
            return;
        }

        double x = t.transform.translation.x;
        double y = t.transform.translation.y;

        tf2::Quaternion q;
        tf2::fromMsg(t.transform.rotation, q);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        auto poses = readJSON();

        bool found = false;
        for (auto &p : poses) {
            if (p.name == req->room_name) {
                p.x = x; p.y = y; p.yaw = yaw;
                p.type = rp::clampType(req->type);
                p.visibility_level = rp::clampVisibility(req->visibility_level);
                p.tags = req->tags.empty() ? rp::kDefaultTags : req->tags;
                found = true;
            }
        }

        if (!found) {
            int new_id = poses.empty() ? 0 : poses.back().id + 1;
            poses.push_back(RoomPose{
                new_id, x, y, yaw, req->room_name,
                rp::clampType(req->type),
                rp::clampVisibility(req->visibility_level),
                req->tags.empty() ? rp::kDefaultTags : req->tags
            });
        }

        writeJSON(poses);
        res->success = true;
        res->message = "Recorded";
    }

    std::vector<RoomPose> readJSON() {
        std::vector<RoomPose> poses;
        std::ifstream f(json_file_path_);
        if (!f.is_open()) return poses;

        json j; f >> j;

        for (auto &item : j["rooms"]) {
            RoomPose p;
            p.id = item["id"];
            p.name = item["name"];
            p.x = item["x"];
            p.y = item["y"];
            p.yaw = item["yaw"];
            p.type = rp::parseType(item, "type");
            p.visibility_level = rp::parseVisibility(item, "visibility_level");
            p.tags = rp::parseTags(item, "tags");
            poses.push_back(p);
        }
        return poses;
    }

    void writeJSON(const std::vector<RoomPose> &poses) {
        json j;
        j["rooms"] = json::array();

        for (auto &p : poses) {
            j["rooms"].push_back({
                {"id", p.id},
                {"name", p.name},
                {"x", p.x},
                {"y", p.y},
                {"yaw", p.yaw},
                {"type", p.type},
                {"visibility_level", p.visibility_level},
                {"tags", p.tags}
            });
        }

        std::ofstream f(json_file_path_, std::ios::trunc);
        f << std::setw(2) << j;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PoseRecorderNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
