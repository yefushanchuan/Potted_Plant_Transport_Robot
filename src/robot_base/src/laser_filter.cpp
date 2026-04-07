// C++17 standard.
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <limits>
#include <cmath>
#include <memory>
#include <utility>

namespace robot_base 
{

class LidarFilter2D : public rclcpp::Node
{
public:
  explicit LidarFilter2D(const rclcpp::NodeOptions & options);

private:
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  std::string source_topic_name_;
  std::string pub_topic_name_;
  double outlier_threshold_;

  void lidarCallback(sensor_msgs::msg::LaserScan::UniquePtr scan);
};

LidarFilter2D::LidarFilter2D(const rclcpp::NodeOptions & options)
: Node("lidar_filter", options)
{
  // 从参数服务器获取参数
  this->declare_parameter<std::string>("source_topic", "/scan");
  this->declare_parameter<std::string>("pub_topic", "/scan_filtered");
  this->declare_parameter<double>("outlier_threshold", 0.1);

  source_topic_name_ = this->get_parameter("source_topic").as_string();
  pub_topic_name_ = this->get_parameter("pub_topic").as_string();
  outlier_threshold_ = this->get_parameter("outlier_threshold").as_double();

  RCLCPP_INFO(this->get_logger(), "Lidar Filter initialized with:");
  RCLCPP_INFO(this->get_logger(), "Source topic: %s", source_topic_name_.c_str());
  RCLCPP_INFO(this->get_logger(), "Publish topic: %s", pub_topic_name_.c_str());
  RCLCPP_INFO(this->get_logger(), "Outlier threshold: %f", outlier_threshold_);

  scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(pub_topic_name_, 10);
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    source_topic_name_, 10,
    [this](sensor_msgs::msg::LaserScan::UniquePtr msg) {
      this->lidarCallback(std::move(msg));
    });
}

void LidarFilter2D::lidarCallback(sensor_msgs::msg::LaserScan::UniquePtr scan)
{
  size_t nRanges = scan->ranges.size();

  // 如果点数太少，无法进行近邻比较，直接发布原始数据
  if (nRanges < 3) {    //判断离群点 点数 可以调整
    scan_pub_->publish(std::move(scan));
    return;
  }

  // 对scan中的离群点进行剔除
  // 遍历备份数据 scan->ranges，从第二个点到倒数第二个点
  // 因为第一个点没有前一个点，最后一个点没有后一个点
  for (size_t i = 1; i < nRanges - 1; ++i) {
    float prev_range = scan->ranges[i - 1];
    float current_range = scan->ranges[i];
    float next_range = scan->ranges[i + 1];

    // 检查当前点是否有效 (在min_range和max_range之间，且不是inf/nan)
    // 使用 scan 的 range_min 和 range_max 进行有效性判断
    bool current_valid = std::isfinite(current_range) &&
      current_range >= scan->range_min &&
      current_range <= scan->range_max;

    if (!current_valid) {
      continue;       // 当前点本身无效，跳过
    }

    // 检查前后邻居点是否有效
    bool prev_valid = std::isfinite(prev_range) &&
      prev_range >= scan->range_min &&
      prev_range <= scan->range_max;

    bool next_valid = std::isfinite(next_range) &&
      next_range >= scan->range_min &&
      next_range <= scan->range_max;

    // 只有当当前点和其前后两个邻居点都有效时，才进行离群判断
    if (prev_valid && next_valid) {
      if (std::abs(current_range - prev_range) > outlier_threshold_ &&
        std::abs(current_range - next_range) > outlier_threshold_)
      {
        // 判定为离群点，将其无效化
        scan->ranges[i] = std::numeric_limits<float>::infinity();
        // 如果有强度信息，将其对应强度设为0
        if (!scan->intensities.empty() && i < scan->intensities.size()) {
          scan->intensities[i] = 0.0f;
        }
      }
    }
  }

  scan_pub_->publish(std::move(scan));
}
} // namespace robot_base

// 注册组件
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(robot_base::LidarFilter2D)
