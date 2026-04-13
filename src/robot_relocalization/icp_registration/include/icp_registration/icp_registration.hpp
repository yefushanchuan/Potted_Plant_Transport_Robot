#ifndef ICP_REGISTRATION_HPP
#define ICP_REGISTRATION_HPP

// std
#include <filesystem>

// ros
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

// pcl
#include <pcl/filters/voxel_grid.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/point_cloud.h>
#include <pcl/registration/icp_nl.h>

namespace icp {

using PointType      = pcl::PointXYZINormal;
using PointCloudXYZI = pcl::PointCloud<pcl::PointXYZI>;
using PointCloudXYZIN = pcl::PointCloud<pcl::PointXYZINormal>;

class IcpNode : public rclcpp::Node {
 public:
  IcpNode(const rclcpp::NodeOptions &options);
  ~IcpNode();

 private:
  void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

  static PointCloudXYZIN::Ptr addNorm(PointCloudXYZI::Ptr cloud);
  Eigen::Matrix4d multiAlignSync(PointCloudXYZI::Ptr source, const Eigen::Matrix4d &init_guess);

  void getStaticTf();

  // ROS2
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr reloc_pose_pub_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Voxel filters
  pcl::VoxelGrid<pcl::PointXYZI> voxel_rough_filter_;
  pcl::VoxelGrid<pcl::PointXYZI> voxel_refine_filter_;

  // ICP
  int rough_iter_;
  int refine_iter_;
  pcl::IterativeClosestPointWithNormals<PointType, PointType> icp_rough_;
  pcl::IterativeClosestPointWithNormals<PointType, PointType> icp_refine_;

  // Data
  PointCloudXYZI::Ptr cloud_in_;
  PointCloudXYZIN::Ptr refine_map_;
  PointCloudXYZIN::Ptr rough_map_;
  std::filesystem::path pcd_path_;

  geometry_msgs::msg::TransformStamped map_to_odom_tf_;
  geometry_msgs::msg::Pose initial_pose_msg_;

  // Parameters
  std::string map_frame_id_;
  std::string odom_frame_id_;
  std::string base_frame_id_;
  std::string laser_frame_id_;
  std::string pose_topic_;

  bool publish_pose_;
  bool success_;
  bool has_cloud_ = false;
  bool publish_tf_;
  bool has_calculated_pose_ = false;
  bool use_yaml_initial_pose_;
  bool has_sensor_tf_ = false;

  double score_;
  double thresh_;
  double xy_offset_;
  double yaw_resolution_;
  int yaw_steps_;

  Eigen::Matrix4f base_to_sensor_T_;

  rclcpp::TimerBase::SharedPtr init_tf_timer_;
  rclcpp::TimerBase::SharedPtr tf_timer_;
};

}  // namespace icp

#endif  // ICP_REGISTRATION_HPP
