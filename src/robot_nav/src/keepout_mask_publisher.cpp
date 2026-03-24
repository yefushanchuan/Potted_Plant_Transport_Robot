#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nlohmann/json.hpp>
#include <fstream>
#include <string>
#include <vector>
#include <cmath>
#include <filesystem>
#include <chrono>
#include <std_msgs/msg/string.hpp>

using std::placeholders::_1;
using json = nlohmann::json;
namespace fs = std::filesystem;

/**
 * @brief 禁行区域定义
 */
struct KeepoutZone {
    enum Type { RECTANGLE, POLYGON };
    Type type;
    std::string name;
    std::string frame_id;
    std::vector<geometry_msgs::msg::Point> points;
    
    KeepoutZone() : type(RECTANGLE), frame_id("map") {}
};

class KeepoutMaskPublisher : public rclcpp::Node
{
public:
    KeepoutMaskPublisher() : Node("keepout_mask_publisher")
    {
        // 参数声明
        declare_parameter<std::string>("map_topic", "/map");
        declare_parameter<std::string>("mask_topic", "/keepout_filter_mask");
        declare_parameter<std::string>("keepout_zones_file", "");
        declare_parameter<bool>("enable_keepout", true);
        
        map_topic_ = get_parameter("map_topic").as_string();
        mask_topic_ = get_parameter("mask_topic").as_string();
        zones_file_ = get_parameter("keepout_zones_file").as_string();
        enable_keepout_ = get_parameter("enable_keepout").as_bool();
        
        // 加载禁行区域
        if (!zones_file_.empty()) {
            if (loadKeepoutZones(zones_file_)) {
                mask_dirty_ = true;
            }
            try {
                last_zones_write_time_ = fs::last_write_time(zones_file_);
                zones_time_valid_ = true;
            } catch (const std::exception& e) {
                RCLCPP_WARN(get_logger(), "Cannot get keepout file time: %s", e.what());
                zones_time_valid_ = false;
            }
        } else {
            RCLCPP_WARN(get_logger(), "No zones file specified");
        }
        
        // ✅ 正确的 QoS：transient_local + reliable
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
        
        // ✅ 订阅地图也用正确的 QoS
        map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
            map_topic_, qos, std::bind(&KeepoutMaskPublisher::mapCallback, this, _1));
        
        // ✅ 发布 mask 用正确的 QoS
        mask_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(mask_topic_, qos);

        // 接收来自控制端的全量更新
        auto update_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
        zones_update_sub_ = create_subscription<std_msgs::msg::String>(
            "/keepout_zones/full_update", update_qos,
            std::bind(&KeepoutMaskPublisher::zonesUpdateCallback, this, _1));

        // 文件变化检测（兼容手动编辑）
        if (!zones_file_.empty()) {
            file_watch_timer_ = create_wall_timer(
                std::chrono::seconds(2),
                std::bind(&KeepoutMaskPublisher::checkZonesFileUpdate, this));
        }
        
        RCLCPP_INFO(get_logger(), "KeepoutMaskPublisher started");
        RCLCPP_INFO(get_logger(), "  - Zones: %zu", keepout_zones_.size());
    }

private:
    bool parseKeepoutConfig(const json& config, std::vector<KeepoutZone>& zones)
    {
        if (!config.contains("keepout_zones") || !config["keepout_zones"].is_array()) {
            return false;
        }

        zones.clear();
        for (const auto& zone_node : config["keepout_zones"]) {
            if (!zone_node.contains("name")) {
                continue;
            }

            KeepoutZone zone;
            zone.name = zone_node.value("name", "unnamed");
            zone.frame_id = zone_node.value("frame_id", "map");

            std::string type_str = zone_node.value("type", "rectangle");
            if (type_str == "rectangle") {
                zone.type = KeepoutZone::RECTANGLE;
            } else if (type_str == "polygon") {
                zone.type = KeepoutZone::POLYGON;
            } else {
                continue;
            }

            const json* points_array = nullptr;
            if (zone_node.contains("points") && zone_node["points"].is_array()) {
                points_array = &zone_node["points"];
            } else {
                const std::string legacy_key = (zone.type == KeepoutZone::RECTANGLE) ? "corners" : "vertices";
                if (zone_node.contains(legacy_key) && zone_node[legacy_key].is_array()) {
                    points_array = &zone_node[legacy_key];
                }
            }

            const auto min_points = (zone.type == KeepoutZone::POLYGON) ? 3u : 2u;
            if (!points_array || points_array->size() < min_points) {
                continue;
            }

            for (const auto& vertex : *points_array) {
                if (!vertex.contains("x") || !vertex.contains("y")) {
                    continue;
                }
                geometry_msgs::msg::Point p;
                p.x = vertex.value("x", 0.0);
                p.y = vertex.value("y", 0.0);
                p.z = 0.0;
                zone.points.push_back(p);
            }

            if (zone.points.size() >= min_points) {
                zones.push_back(zone);
            }
        }
        return true;
    }

    bool loadKeepoutZones(const std::string& filename)
    {
        try {
            std::ifstream file(filename);
            if (!file.is_open()) {
                RCLCPP_ERROR(get_logger(), "Cannot open file: %s", filename.c_str());
                return false;
            }

            json config;
            file >> config;
            file.close();

            std::vector<KeepoutZone> parsed;
            if (!parseKeepoutConfig(config, parsed)) {
                RCLCPP_WARN(get_logger(), "Invalid keepout_zones format");
                return false;
            }
            keepout_zones_ = std::move(parsed);
            return true;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Error loading zones: %s", e.what());
            return false;
        }
    }

    void saveKeepoutZones()
    {
        if (zones_file_.empty()) {
            return;
        }

        json root;
        root["keepout_zones"] = json::array();
        for (const auto& zone : keepout_zones_) {
            json node;
            node["name"] = zone.name;
            node["type"] = (zone.type == KeepoutZone::RECTANGLE) ? "rectangle" : "polygon";
            node["frame_id"] = zone.frame_id;
            node["points"] = json::array();
            for (const auto& p : zone.points) {
                node["points"].push_back({{"x", p.x}, {"y", p.y}});
            }
            root["keepout_zones"].push_back(node);
        }

        try {
            std::ofstream file(zones_file_);
            if (!file.is_open()) {
                RCLCPP_ERROR(get_logger(), "Cannot write file: %s", zones_file_.c_str());
                return;
            }
            file << root.dump(2);
            file.close();

            if (!zones_file_.empty()) {
                last_zones_write_time_ = fs::last_write_time(zones_file_);
                zones_time_valid_ = true;
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Error saving zones: %s", e.what());
        }
    }

    void worldToMap(double wx, double wy, const nav_msgs::msg::MapMetaData& info, int& mx, int& my)
    {
        mx = static_cast<int>((wx - info.origin.position.x) / info.resolution);
        my = static_cast<int>((wy - info.origin.position.y) / info.resolution);
    }

    bool pointInPolygon(double px, double py, const std::vector<geometry_msgs::msg::Point>& polygon)
    {
        bool inside = false;
        size_t n = polygon.size();
        for (size_t i = 0, j = n - 1; i < n; j = i++) {
            double xi = polygon[i].x, yi = polygon[i].y;
            double xj = polygon[j].x, yj = polygon[j].y;
            bool intersect = ((yi > py) != (yj > py)) &&
                             (px < (xj - xi) * (py - yi) / (yj - yi) + xi);
            if (intersect) inside = !inside;
        }
        return inside;
    }

    bool pointInRectangle(double px, double py, const KeepoutZone& zone)
    {
        if (zone.points.size() < 2) return false;
        double x1 = std::min(zone.points[0].x, zone.points[1].x);
        double x2 = std::max(zone.points[0].x, zone.points[1].x);
        double y1 = std::min(zone.points[0].y, zone.points[1].y);
        double y2 = std::max(zone.points[0].y, zone.points[1].y);
        return (px >= x1 && px <= x2 && py >= y1 && py <= y2);
    }

    void applyKeepoutZones(nav_msgs::msg::OccupancyGrid& mask)
    {
        // ✅ 修复1：默认所有区域可通行（0）
        std::fill(mask.data.begin(), mask.data.end(), 0);
        
        if (!enable_keepout_ || keepout_zones_.empty()) {
            return;
        }
        
        int width = mask.info.width;
        int height = mask.info.height;
        double resolution = mask.info.resolution;
        
        // ✅ 修复2：禁行区设置为 100（障碍物）
        for (const auto& zone : keepout_zones_) {
            for (int my = 0; my < height; ++my) {
                for (int mx = 0; mx < width; ++mx) {
                    double wx = mask.info.origin.position.x + mx * resolution;
                    double wy = mask.info.origin.position.y + my * resolution;
                    
                    bool in_zone = false;
                    if (zone.type == KeepoutZone::RECTANGLE) {
                        in_zone = pointInRectangle(wx, wy, zone);
                    } else if (zone.type == KeepoutZone::POLYGON) {
                        in_zone = pointInPolygon(wx, wy, zone.points);
                    }
                    
                    if (in_zone) {
                        mask.data[my * width + mx] = 100;  // ✅ 设置为障碍物！
                    }
                }
            }
        }
    }

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        last_map_ = msg;
        if (mask_dirty_) {
            publishMask();
        }
    }

    void zonesUpdateCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        try {
            json config = json::parse(msg->data);
            std::vector<KeepoutZone> parsed;
            if (!parseKeepoutConfig(config, parsed)) {
                RCLCPP_WARN(get_logger(), "Received invalid keepout_zones update");
                return;
            }
            keepout_zones_ = std::move(parsed);
            mask_dirty_ = true;
            saveKeepoutZones();
            publishMask();
            RCLCPP_INFO(get_logger(), "Keepout zones updated from MQTT (%zu zones)", keepout_zones_.size());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Failed to parse keepout update: %s", e.what());
        }
    }
    
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr mask_pub_;
    std::string map_topic_;
    std::string mask_topic_;
    bool enable_keepout_;
    bool mask_dirty_ = true;
    std::vector<KeepoutZone> keepout_zones_;
    std::string zones_file_;
    rclcpp::TimerBase::SharedPtr file_watch_timer_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr zones_update_sub_;
    nav_msgs::msg::OccupancyGrid::SharedPtr last_map_;
    fs::file_time_type last_zones_write_time_;
    bool zones_time_valid_ = false;

    void publishMask()
    {
        if (!last_map_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                 "Map not received yet, skip mask publish");
            return;
        }

        auto mask_msg = nav_msgs::msg::OccupancyGrid(*last_map_);
        mask_msg.header.stamp = now();
        mask_msg.header.frame_id = last_map_->header.frame_id;
        applyKeepoutZones(mask_msg);
        mask_pub_->publish(mask_msg);
        mask_dirty_ = false;
        RCLCPP_INFO(get_logger(), "Keepout mask published (%zu zones)", keepout_zones_.size());
    }

    void checkZonesFileUpdate()
    {
        if (zones_file_.empty()) {
            return;
        }
        try {
            auto current = fs::last_write_time(zones_file_);
            if (!zones_time_valid_ || last_zones_write_time_ != current) {
                zones_time_valid_ = true;
                last_zones_write_time_ = current;
                RCLCPP_INFO(get_logger(), "Keepout zones file changed, reloading");
                if (loadKeepoutZones(zones_file_)) {
                    mask_dirty_ = true;
                    publishMask();
                }
            }
        } catch (const std::exception& e) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                 "Keepout file check failed: %s", e.what());
        }
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KeepoutMaskPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
