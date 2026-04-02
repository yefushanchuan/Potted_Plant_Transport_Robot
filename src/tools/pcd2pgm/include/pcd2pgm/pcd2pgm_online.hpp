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

#ifndef PCD2PGM__PCD2PGM_ONLINE_HPP_
#define PCD2PGM__PCD2PGM_ONLINE_HPP_

#include <mutex>
#include <string>
#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "pcl/filters/passthrough.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace pcd2pgm
{
class Pcd2PgmOnlineNode : public rclcpp::Node
{
public:
  explicit Pcd2PgmOnlineNode(const rclcpp::NodeOptions & options);

private:
  void declareParameters();
  void getParameters();

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void handleSaveMap(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  pcl::PointCloud<pcl::PointXYZ>::Ptr passThroughFilter(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & input_cloud,
    double thre_low, double thre_high, bool flag_in);
  pcl::PointCloud<pcl::PointXYZ>::Ptr radiusOutlierFilter(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & input_cloud,
    double radius, int thre_count);
  void setMapTopicMsg(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
    nav_msgs::msg::OccupancyGrid & msg);
  void applyTransform(pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud);

  bool saveMapFiles(
    const nav_msgs::msg::OccupancyGrid & map,
    std::string & yaml_path,
    std::string & pgm_path,
    std::string & error);
  bool writePgm(
    const nav_msgs::msg::OccupancyGrid & map,
    const std::string & pgm_path,
    std::string & error);
  bool writeYaml(
    const nav_msgs::msg::OccupancyGrid & map,
    const std::string & yaml_path,
    const std::string & pgm_filename,
    std::string & error);
  std::string buildMapName() const;

  std::string input_cloud_topic_;
  std::string map_topic_name_;
  std::string map_frame_id_;
  std::string map_save_dir_;
  std::string map_save_prefix_;
  bool map_save_use_timestamp_{true};
  double occupied_thresh_{0.65};
  double free_thresh_{0.196};
  int unknown_pixel_{205};

  float thre_z_min_{0.0f};
  float thre_z_max_{1.0f};
  float thre_radius_{0.1f};
  bool flag_pass_through_{false};
  float map_resolution_{0.05f};
  int thres_point_count_{3};
  std::vector<double> odom_to_lidar_odom_;

  std::mutex map_mutex_;
  nav_msgs::msg::OccupancyGrid latest_map_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr latest_cloud_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_publisher_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_map_srv_;
};
}  // namespace pcd2pgm

#endif  // PCD2PGM__PCD2PGM_ONLINE_HPP_
