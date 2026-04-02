// Copyright 2025 Lihan Chen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "pcd2pgm/pcd2pgm_online.hpp"

#include <cmath>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <utility>

#include "pcl/common/transforms.h"
#include "pcl/filters/radius_outlier_removal.h"
#include "pcl_conversions/pcl_conversions.h"

#if __has_include(<filesystem>)
#include <filesystem>
namespace fs = std::filesystem;
#else
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;
#endif

namespace pcd2pgm
{
Pcd2PgmOnlineNode::Pcd2PgmOnlineNode(const rclcpp::NodeOptions & options)
: Node("pcd2pgm_online", options)
{
  declareParameters();
  getParameters();

  rclcpp::QoS map_qos(10);
  map_qos.transient_local();
  map_qos.reliable();
  map_qos.keep_last(1);

  rclcpp::QoS cloud_qos = rclcpp::SensorDataQoS();

  map_publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>(map_topic_name_, map_qos);
  cloud_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("pcd_cloud", 10);

  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    input_cloud_topic_, cloud_qos,
    std::bind(&Pcd2PgmOnlineNode::cloudCallback, this, std::placeholders::_1));

  save_map_srv_ = create_service<std_srvs::srv::Trigger>(
    "map_save",
    std::bind(
      &Pcd2PgmOnlineNode::handleSaveMap, this,
      std::placeholders::_1, std::placeholders::_2));

  latest_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

  RCLCPP_INFO(
    get_logger(),
    "pcd2pgm_online ready. Subscribe: %s, publish map: %s, save service: %s",
    input_cloud_topic_.c_str(), map_topic_name_.c_str(),
    (std::string(get_fully_qualified_name()) + "/map_save").c_str());
}

void Pcd2PgmOnlineNode::declareParameters()
{
  declare_parameter("input_cloud_topic", "/fastlio2/world_cloud");
  declare_parameter("map_topic_name", "map");
  declare_parameter("map_frame_id", "map");
  declare_parameter("map_save_dir", "/tmp/pcd2pgm_maps");
  declare_parameter("map_save_prefix", "map");
  declare_parameter("map_save_use_timestamp", true);
  declare_parameter("occupied_thresh", 0.65);
  declare_parameter("free_thresh", 0.196);
  declare_parameter("unknown_pixel", 205);

  declare_parameter("thre_z_min", 0.5);
  declare_parameter("thre_z_max", 2.0);
  declare_parameter("flag_pass_through", false);
  declare_parameter("thre_radius", 0.5);
  declare_parameter("map_resolution", 0.05);
  declare_parameter("thres_point_count", 10);
  declare_parameter(
    "odom_to_lidar_odom", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
}

void Pcd2PgmOnlineNode::getParameters()
{
  get_parameter("input_cloud_topic", input_cloud_topic_);
  get_parameter("map_topic_name", map_topic_name_);
  get_parameter("map_frame_id", map_frame_id_);
  get_parameter("map_save_dir", map_save_dir_);
  get_parameter("map_save_prefix", map_save_prefix_);
  get_parameter("map_save_use_timestamp", map_save_use_timestamp_);
  get_parameter("occupied_thresh", occupied_thresh_);
  get_parameter("free_thresh", free_thresh_);
  get_parameter("unknown_pixel", unknown_pixel_);

  get_parameter("thre_z_min", thre_z_min_);
  get_parameter("thre_z_max", thre_z_max_);
  get_parameter("flag_pass_through", flag_pass_through_);
  get_parameter("thre_radius", thre_radius_);
  get_parameter("map_resolution", map_resolution_);
  get_parameter("thres_point_count", thres_point_count_);
  get_parameter("odom_to_lidar_odom", odom_to_lidar_odom_);
}

void Pcd2PgmOnlineNode::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  auto input_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::fromROSMsg(*msg, *input_cloud);
  if (input_cloud->points.empty()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Received empty point cloud.");
    return;
  }

  applyTransform(input_cloud);

  auto pass_cloud = passThroughFilter(
    input_cloud, thre_z_min_, thre_z_max_, flag_pass_through_);
  auto filtered_cloud = radiusOutlierFilter(
    pass_cloud, thre_radius_, thres_point_count_);

  nav_msgs::msg::OccupancyGrid map_msg;
  setMapTopicMsg(filtered_cloud, map_msg);

  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    latest_map_ = map_msg;
    latest_cloud_ = filtered_cloud;
  }

  sensor_msgs::msg::PointCloud2 output;
  pcl::toROSMsg(*filtered_cloud, output);
  output.header.frame_id = map_frame_id_;
  output.header.stamp = now();
  cloud_publisher_->publish(output);

  map_publisher_->publish(map_msg);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Pcd2PgmOnlineNode::passThroughFilter(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & input_cloud,
  double thre_low, double thre_high, bool flag_in)
{
  auto filtered_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::PassThrough<pcl::PointXYZ> passthrough;
  passthrough.setInputCloud(input_cloud);
  passthrough.setFilterFieldName("z");
  passthrough.setFilterLimits(thre_low, thre_high);
  passthrough.setNegative(flag_in);
  passthrough.filter(*filtered_cloud);
  return filtered_cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Pcd2PgmOnlineNode::radiusOutlierFilter(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & input_cloud,
  double radius, int thre_count)
{
  auto filtered_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::RadiusOutlierRemoval<pcl::PointXYZ> radius_outlier;
  radius_outlier.setInputCloud(input_cloud);
  radius_outlier.setRadiusSearch(radius);
  radius_outlier.setMinNeighborsInRadius(thre_count);
  radius_outlier.filter(*filtered_cloud);
  return filtered_cloud;
}

void Pcd2PgmOnlineNode::setMapTopicMsg(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud, nav_msgs::msg::OccupancyGrid & msg)
{
  msg.header.stamp = now();
  msg.header.frame_id = map_frame_id_;

  msg.info.map_load_time = now();
  msg.info.resolution = map_resolution_;

  if (cloud->points.empty()) {
    RCLCPP_WARN(get_logger(), "Point cloud is empty!");
    return;
  }

  double x_min = std::numeric_limits<double>::max();
  double x_max = std::numeric_limits<double>::lowest();
  double y_min = std::numeric_limits<double>::max();
  double y_max = std::numeric_limits<double>::lowest();

  for (const auto & point : cloud->points) {
    x_min = std::min(x_min, static_cast<double>(point.x));
    x_max = std::max(x_max, static_cast<double>(point.x));
    y_min = std::min(y_min, static_cast<double>(point.y));
    y_max = std::max(y_max, static_cast<double>(point.y));
  }

  msg.info.origin.position.x = x_min;
  msg.info.origin.position.y = y_min;
  msg.info.origin.position.z = 0.0;
  msg.info.origin.orientation.x = 0.0;
  msg.info.origin.orientation.y = 0.0;
  msg.info.origin.orientation.z = 0.0;
  msg.info.origin.orientation.w = 1.0;

  msg.info.width = std::ceil((x_max - x_min) / map_resolution_);
  msg.info.height = std::ceil((y_max - y_min) / map_resolution_);
  msg.data.assign(msg.info.width * msg.info.height, 0);

  for (const auto & point : cloud->points) {
    const int i = std::floor((point.x - x_min) / map_resolution_);
    const int j = std::floor((point.y - y_min) / map_resolution_);
    if (i >= 0 && i < static_cast<int>(msg.info.width) &&
      j >= 0 && j < static_cast<int>(msg.info.height))
    {
      msg.data[i + j * msg.info.width] = 100;
    }
  }
}

void Pcd2PgmOnlineNode::applyTransform(pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud)
{
  if (odom_to_lidar_odom_.size() != 6) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "odom_to_lidar_odom size != 6, skip transform.");
    return;
  }

  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  transform.translation() << odom_to_lidar_odom_[0], odom_to_lidar_odom_[1], odom_to_lidar_odom_[2];
  transform.rotate(Eigen::AngleAxisf(odom_to_lidar_odom_[3], Eigen::Vector3f::UnitX()));
  transform.rotate(Eigen::AngleAxisf(odom_to_lidar_odom_[4], Eigen::Vector3f::UnitY()));
  transform.rotate(Eigen::AngleAxisf(odom_to_lidar_odom_[5], Eigen::Vector3f::UnitZ()));
  pcl::transformPointCloud(*cloud, *cloud, transform.inverse());
}

void Pcd2PgmOnlineNode::handleSaveMap(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  nav_msgs::msg::OccupancyGrid map_snapshot;
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    map_snapshot = latest_map_;
  }

  if (map_snapshot.data.empty()) {
    response->success = false;
    response->message = "Map is empty, nothing to save.";
    return;
  }

  std::string yaml_path;
  std::string pgm_path;
  std::string error;
  if (!saveMapFiles(map_snapshot, yaml_path, pgm_path, error)) {
    response->success = false;
    response->message = error;
    return;
  }

  response->success = true;
  response->message = "Saved map: " + yaml_path;
}

bool Pcd2PgmOnlineNode::saveMapFiles(
  const nav_msgs::msg::OccupancyGrid & map,
  std::string & yaml_path,
  std::string & pgm_path,
  std::string & error)
{
  try {
    fs::create_directories(map_save_dir_);
  } catch (const std::exception & e) {
    error = std::string("Failed to create map_save_dir: ") + e.what();
    return false;
  }

  const std::string map_name = buildMapName();
  pgm_path = (fs::path(map_save_dir_) / (map_name + ".pgm")).string();
  yaml_path = (fs::path(map_save_dir_) / (map_name + ".yaml")).string();

  if (!writePgm(map, pgm_path, error)) {
    return false;
  }
  if (!writeYaml(map, yaml_path, map_name + ".pgm", error)) {
    return false;
  }
  return true;
}

bool Pcd2PgmOnlineNode::writePgm(
  const nav_msgs::msg::OccupancyGrid & map,
  const std::string & pgm_path,
  std::string & error)
{
  std::ofstream pgm(pgm_path, std::ios::binary | std::ios::trunc);
  if (!pgm.is_open()) {
    error = "Cannot open pgm file: " + pgm_path;
    return false;
  }

  const int width = static_cast<int>(map.info.width);
  const int height = static_cast<int>(map.info.height);
  pgm << "P5\n" << width << " " << height << "\n255\n";

  for (int y = 0; y < height; ++y) {
    const int map_y = (height - 1) - y;
    for (int x = 0; x < width; ++x) {
      const int idx = x + map_y * width;
      const int8_t val = map.data[idx];
      uint8_t pixel = 0;
      if (val < 0) {
        pixel = static_cast<uint8_t>(unknown_pixel_);
      } else {
        const double occ = static_cast<double>(val) / 100.0;
        const int gray = static_cast<int>(std::round(255.0 * (1.0 - occ)));
        pixel = static_cast<uint8_t>(std::max(0, std::min(255, gray)));
      }
      pgm.write(reinterpret_cast<const char *>(&pixel), sizeof(uint8_t));
    }
  }

  if (!pgm.good()) {
    error = "Write pgm failed: " + pgm_path;
    return false;
  }
  return true;
}

bool Pcd2PgmOnlineNode::writeYaml(
  const nav_msgs::msg::OccupancyGrid & map,
  const std::string & yaml_path,
  const std::string & pgm_filename,
  std::string & error)
{
  std::ofstream yaml(yaml_path, std::ios::out | std::ios::trunc);
  if (!yaml.is_open()) {
    error = "Cannot open yaml file: " + yaml_path;
    return false;
  }

  yaml << "image: " << pgm_filename << "\n";
  yaml << "resolution: " << std::fixed << std::setprecision(6) << map.info.resolution << "\n";
  yaml << "origin: ["
       << std::fixed << std::setprecision(6)
       << map.info.origin.position.x << ", "
       << map.info.origin.position.y << ", 0.0]\n";
  yaml << "negate: 0\n";
  yaml << "occupied_thresh: " << std::fixed << std::setprecision(3) << occupied_thresh_ << "\n";
  yaml << "free_thresh: " << std::fixed << std::setprecision(3) << free_thresh_ << "\n";

  if (!yaml.good()) {
    error = "Write yaml failed: " + yaml_path;
    return false;
  }
  return true;
}

std::string Pcd2PgmOnlineNode::buildMapName() const
{
  if (!map_save_use_timestamp_) {
    return map_save_prefix_.empty() ? "map" : map_save_prefix_;
  }

  const auto now_ns = now().nanoseconds();
  const std::time_t tt = static_cast<std::time_t>(now_ns / 1000000000);
  std::tm tm{};
  localtime_r(&tt, &tm);
  std::ostringstream oss;
  if (!map_save_prefix_.empty()) {
    oss << map_save_prefix_ << "_";
  }
  oss << std::put_time(&tm, "%Y%m%d_%H%M%S");
  return oss.str();
}

}  // namespace pcd2pgm

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(pcd2pgm::Pcd2PgmOnlineNode)
