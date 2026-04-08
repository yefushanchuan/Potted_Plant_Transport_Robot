#include "icp_registration/icp_registration.hpp"
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <chrono>
#include <geometry_msgs/msg/detail/pose_with_covariance_stamped__struct.hpp>
#include <iostream>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/qos.hpp>
#include <stdexcept>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/create_timer_ros.h>

namespace icp {

IcpNode::IcpNode(const rclcpp::NodeOptions &options)
    : Node("icp_registration", options), rough_iter_(10), refine_iter_(5),
      first_scan_(true), success_(false), is_ready_(false),
      rough_source_(new pcl::PointCloud<pcl::PointXYZI>),
      refine_source_(new pcl::PointCloud<pcl::PointXYZI>),
      rough_source_norm_(new PointCloudXYZIN),
      refine_source_norm_(new PointCloudXYZIN),
      align_point_(new PointCloudXYZIN),
      cloud_in_(new pcl::PointCloud<pcl::PointXYZI>),
      normals_buffer_(new pcl::PointCloud<pcl::Normal>),
      kdtree_buffer_(new pcl::search::KdTree<pcl::PointXYZI>) {
  // this->declare_parameter("use_sim_time", false);
  double rough_leaf_size = this->declare_parameter("rough_leaf_size", 0.4);
  double refine_leaf_size = this->declare_parameter("refine_leaf_size", 0.1);
  voxel_rough_filter_.setLeafSize(rough_leaf_size, rough_leaf_size,
                                  rough_leaf_size);
  voxel_refine_filter_.setLeafSize(refine_leaf_size, refine_leaf_size,
                                   refine_leaf_size);

  map_filename_ = this->declare_parameter("map_filename", std::string(""));
  if (!std::filesystem::exists(map_filename_)) {
    RCLCPP_ERROR(this->get_logger(), "Invalid map_filename path: %s (Did you pass it via launch?)", map_filename_.c_str());
    throw std::runtime_error("Invalid map_filename path");
  }

  // Read the pcd file
  pcl::PCDReader reader;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZI>);
  reader.read(map_filename_, *cloud);
  voxel_refine_filter_.setInputCloud(cloud);
  voxel_refine_filter_.filter(*cloud);

  // Add normal to the pointcloud
  refine_map_ = addNorm(cloud);
  pcl::PointCloud<pcl::PointXYZI>::Ptr point_rough(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr filterd_point_rough(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::copyPointCloud(*refine_map_, *point_rough);
  voxel_rough_filter_.setInputCloud(point_rough);
  voxel_rough_filter_.filter(*filterd_point_rough);
  rough_map_ = addNorm(filterd_point_rough);

  icp_rough_.setMaximumIterations(rough_iter_);
  icp_rough_.setInputTarget(rough_map_);

  icp_refine_.setMaximumIterations(refine_iter_);
  icp_refine_.setInputTarget(refine_map_);

  RCLCPP_INFO(this->get_logger(), "pcd point size: %ld, %ld",
              refine_map_->size(), rough_map_->size());

  // Parameters
  map_frame_id_ =
      this->declare_parameter("map_frame_id", std::string("map"));
  odom_frame_id_ =
      this->declare_parameter("odom_frame_id", std::string("odom"));
  laser_frame_id_ =
      this->declare_parameter("laser_frame_id", std::string("laser"));
  base_frame_id_ =
      this->declare_parameter("base_frame_id", std::string("base_footprint"));
  publish_tf_ =
      this->declare_parameter("publish_tf", false);
  auto_init_ =
      this->declare_parameter("auto_init", false);
  pose_topic_ =
      this->declare_parameter("pose_topic", std::string("/icp_pose"));
  publish_pose_ =
      this->declare_parameter("publish_pose", true);
  rviz_pose_topic_ =
      this->declare_parameter("rviz_pose_topic", std::string("/initialpose"));
  thresh_ = this->declare_parameter("thresh", 0.15);
  xy_offset_ = this->declare_parameter("xy_offset", 0.2);
  yaw_offset_ = this->declare_parameter("yaw_offset", 30.0) * M_PI / 180.0;
  yaw_resolution_ =
      this->declare_parameter("yaw_resolution", 10.0) * M_PI / 180.0;
  yaw_steps_ = std::round(yaw_offset_ / yaw_resolution_);
  std::vector<double> initial_pose_vec = this->declare_parameter(
      "initial_pose", std::vector<double>{0, 0, 0, 0, 0, 0});
  try {
    initial_pose_.position.x = initial_pose_vec.at(0);
    initial_pose_.position.y = initial_pose_vec.at(1);
    initial_pose_.position.z = initial_pose_vec.at(2);
    tf2::Quaternion q;
    q.setRPY(initial_pose_vec.at(3), initial_pose_vec.at(4),
             initial_pose_vec.at(5));
    initial_pose_.orientation.x = q.x();
    initial_pose_.orientation.y = q.y();
    initial_pose_.orientation.z = q.z();
    initial_pose_.orientation.w = q.w();  
  } catch (const std::out_of_range &ex) {
    RCLCPP_ERROR(this->get_logger(),
                 "initial_pose is not a vector with 6 elements, what():%s",
                 ex.what());
  }

  // Set up the pointcloud subscriber
  std::string pointcloud_topic = this->declare_parameter(
      "pointcloud_topic", std::string("/livox/lidar"));
  RCLCPP_INFO(this->get_logger(), "pointcloud_topic: %s",
              pointcloud_topic.c_str());
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
  pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      pointcloud_topic, qos,
      std::bind(&IcpNode::pointcloudCallback, this, std::placeholders::_1));
  // Set up the initial pose subscriber
  initial_pose_sub_ =
      create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
          rviz_pose_topic_, qos,
          [this](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
            initialPoseCallback(msg);
          });

  // Set up the transform broadcaster
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      this->get_node_base_interface(), this->get_node_timers_interface());
  tf_buffer_->setCreateTimerInterface(timer_interface);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Set up the map publisher
  map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/icp_map_pointcloud", rclcpp::QoS(1).transient_local());
  if (publish_pose_) {
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(pose_topic_, rclcpp::QoS(10));
    RCLCPP_INFO(this->get_logger(), "Publishing ICP pose: %s (frame=%s, base=%s)",
                pose_topic_.c_str(), map_frame_id_.c_str(), base_frame_id_.c_str());
  }

  if (publish_tf_) {
    // 使用 ROS2 的 Timer 替代 std::thread
    timer_ = create_wall_timer(std::chrono::milliseconds(10), [this]() {
      std::lock_guard<std::mutex> lock(mutex_);
      if (is_ready_) {
        map_to_odom_.header.stamp = this->now();
        map_to_odom_.header.frame_id = map_frame_id_;
        map_to_odom_.child_frame_id = odom_frame_id_;
        tf_broadcaster_->sendTransform(map_to_odom_);
      }
    });
  }

  // Publish the map once
  sensor_msgs::msg::PointCloud2 map_msg;
  pcl::toROSMsg(*cloud, map_msg);
  map_msg.header.frame_id = map_frame_id_;
  map_msg.header.stamp = now();
  map_pub_->publish(map_msg);
  RCLCPP_INFO(this->get_logger(), "Published global map with %ld points", cloud->size());

  RCLCPP_INFO(this->get_logger(), "icp_registration initialized");
}

IcpNode::~IcpNode() {
}

void IcpNode::initialPoseCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
  
  std::lock_guard<std::mutex> lock(mutex_);
  latest_rviz_guess_ = msg;
  wait_for_new_cloud_ = true; // 打开阀门
  
  RCLCPP_INFO(this->get_logger(), "Received Initial Pose request. Waiting for the NEXT Lidar frame...");
}

void IcpNode::pointcloudCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  
  // 1. 处理第一次扫描时的 auto_init 逻辑
  if (first_scan_) {
    first_scan_ = false;
    if (auto_init_) {
      RCLCPP_INFO(this->get_logger(), "Auto-init is ON! Triggering ICP with prior pose from YAML...");
      std::lock_guard<std::mutex> lock(mutex_);
      latest_rviz_guess_ = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
      latest_rviz_guess_->header = msg->header;
      latest_rviz_guess_->pose.pose = initial_pose_; // 使用 YAML 里的配置
      wait_for_new_cloud_ = true; // 自动打开阀门
    } else {
      RCLCPP_INFO_ONCE(this->get_logger(), "Auto-init is OFF. IcpNode is SLEEPING until Human clicks RViz [%s]...", rviz_pose_topic_.c_str());
    }
  }

  // 2. 如果阀门关着，直接 return！完全不消耗 CPU 处理点云
  bool should_process = false;
  {
      std::lock_guard<std::mutex> lock(mutex_);
      should_process = wait_for_new_cloud_;
  }
  
  if (!should_process) {
      return; 
  }

  RCLCPP_INFO(this->get_logger(), "Captured ONE fresh Lidar frame. Starting Heavy ICP Alignment...");
  
  // 抓取并转换点云
  pcl::fromROSMsg(*msg, *cloud_in_);

  // 立刻关闭阀门！保证下一帧雷达数据继续被拦截，避免重复运算
  {
      std::lock_guard<std::mutex> lock(mutex_);
      wait_for_new_cloud_ = false;
  }

  // 3. 提取刚刚人类给的（或 YAML里的）粗略初始位姿
  Eigen::Vector3d pos(latest_rviz_guess_->pose.pose.position.x, 
                      latest_rviz_guess_->pose.pose.position.y, 
                      latest_rviz_guess_->pose.pose.position.z);
  Eigen::Quaterniond q(latest_rviz_guess_->pose.pose.orientation.w, 
                       latest_rviz_guess_->pose.pose.orientation.x,
                       latest_rviz_guess_->pose.pose.orientation.y, 
                       latest_rviz_guess_->pose.pose.orientation.z);
                       
  // 4. 获取人类在 RViz 给的底盘位姿 (Map -> Base)
  Eigen::Matrix4d base_guess = Eigen::Matrix4d::Identity();
  base_guess.block<3, 3>(0, 0) = q.toRotationMatrix();
  base_guess.block<3, 1>(0, 3) = pos;

  // 5. 获取 Base 到 Laser 的静态外参 (Base -> Laser)
  Eigen::Matrix4d base_to_laser = Eigen::Matrix4d::Identity();
  try {
    auto transform_b2l = tf_buffer_->lookupTransform(base_frame_id_, laser_frame_id_, tf2::TimePointZero);
    Eigen::Vector3d t_b2l(transform_b2l.transform.translation.x,
                          transform_b2l.transform.translation.y,
                          transform_b2l.transform.translation.z);
    Eigen::Quaterniond q_b2l(transform_b2l.transform.rotation.w, transform_b2l.transform.rotation.x,
                             transform_b2l.transform.rotation.y, transform_b2l.transform.rotation.z);
    base_to_laser.block<3, 3>(0, 0) = q_b2l.toRotationMatrix();
    base_to_laser.block<3, 1>(0, 3) = t_b2l;
  } catch (tf2::TransformException &ex) {
    RCLCPP_ERROR(this->get_logger(), "Failed to get Static TF (base->laser): %s", ex.what());
    std::lock_guard<std::mutex> lock(mutex_);
    wait_for_new_cloud_ = true; 
    return;
  }

  // 6. 计算雷达的初始猜想位姿: Map_to_Laser = Map_to_Base * Base_to_Laser
  Eigen::Matrix4d initial_guess = base_guess * base_to_laser;

  // 7. 执行多重暴力 ICP 对齐 (极耗 CPU)
  Eigen::Matrix4d map_to_laser = multiAlignSync(cloud_in_, initial_guess);
  
  if (!success_) {
    RCLCPP_ERROR(this->get_logger(), 
                 "ICP matching failed! Discarding this frame and waiting for next...");
    
    // 重新打开阀门，等待下一帧雷达数据继续尝试
    std::lock_guard<std::mutex> lock(mutex_);
    wait_for_new_cloud_ = true; 
    return; // 立即中止，绝不发布错误坐标！
  } 
  
  RCLCPP_INFO(this->get_logger(), "ICP matching SUCCESS!");

  // 8. 数学直接求逆，消除冗余的第二次 TF 查询！！！
  Eigen::Matrix4d laser_to_base = base_to_laser.inverse();
  Eigen::Matrix4d map_to_base = map_to_laser * laser_to_base;

  // 强制 2D 约束
  // 消除 Z, Roll, Pitch，只保留 X, Y, Yaw
  double yaw = std::atan2(map_to_base(1, 0), map_to_base(0, 0));
  Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
  Eigen::Quaterniond q_2d(yawAngle);
  map_to_base.block<3,3>(0,0) = q_2d.toRotationMatrix();
  map_to_base(2, 3) = 0.0; // 强制 Z = 0
    
  // 9. 发布逻辑
  if (publish_tf_) {
    std::lock_guard<std::mutex> lock(mutex_);
    map_to_odom_.transform.translation.x = map_to_base(0, 3);
    map_to_odom_.transform.translation.y = map_to_base(1, 3);
    map_to_odom_.transform.translation.z = map_to_base(2, 3);
    Eigen::Matrix3d rotation = map_to_base.block<3, 3>(0, 0);
    Eigen::Quaterniond q_eigen(rotation);
    map_to_odom_.transform.rotation.w = q_eigen.w();
    map_to_odom_.transform.rotation.x = q_eigen.x();
    map_to_odom_.transform.rotation.y = q_eigen.y();
    map_to_odom_.transform.rotation.z = q_eigen.z();
    is_ready_ = true; 
  }
  
  if (publish_pose_) {
    geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
    pose_msg.header.stamp = msg->header.stamp; // 使用当前雷达帧的时间戳
    pose_msg.header.frame_id = map_frame_id_;
    
    pose_msg.pose.pose.position.x = map_to_base(0, 3);
    pose_msg.pose.pose.position.y = map_to_base(1, 3);
    pose_msg.pose.pose.position.z = map_to_base(2, 3);
    
    Eigen::Matrix3d rotation = map_to_base.block<3, 3>(0, 0);
    Eigen::Quaterniond q_out(rotation);
    pose_msg.pose.pose.orientation.w = q_out.w();
    pose_msg.pose.pose.orientation.x = q_out.x();
    pose_msg.pose.pose.orientation.y = q_out.y();
    pose_msg.pose.pose.orientation.z = q_out.z();

    pose_pub_->publish(pose_msg);
    RCLCPP_INFO(this->get_logger(), "Successfully Published EXACT INITIAL POSE to Tracker! IcpNode is now going back to sleep.");
  }
}
Eigen::Matrix4d IcpNode::multiAlignSync(PointCloudXYZI::Ptr source,
                                        const Eigen::Matrix4d &init_guess) {
  // 辅助lambda：旋转矩阵转RPY
  auto rotate2rpy = [](const Eigen::Matrix3d &rot) -> Eigen::Vector3d {
    double roll = std::atan2(rot(2, 1), rot(2, 2));
    double pitch = std::asin(-rot(2, 0));
    double yaw = std::atan2(rot(1, 0), rot(0, 0));
    return Eigen::Vector3d(roll, pitch, yaw);
  };

  success_ = false;
  
  // 提取初始位姿的位置和姿态
  Eigen::Vector3d xyz = init_guess.block<3, 1>(0, 3);
  Eigen::Matrix3d rotation = init_guess.block<3, 3>(0, 0);
  Eigen::Vector3d rpy = rotate2rpy(rotation);
  
  // 预计算Roll和Pitch角度（Yaw在循环内变化）
  Eigen::AngleAxisf rollAngle(rpy(0), Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf pitchAngle(rpy(1), Eigen::Vector3f::UnitY());
  
  // 生成候选位姿网格（XY平移 + Yaw旋转）
  std::vector<Eigen::Matrix4f> candidates;
  candidates.reserve(9 * (2 * yaw_steps_ + 1));  // 预分配避免扩容
  
  for (int i = -1; i <= 1; i++) {
    for (int j = -1; j <= 1; j++) {
      for (int k = -yaw_steps_; k <= yaw_steps_; k++) {
        Eigen::Vector3f pos(xyz(0) + i * xy_offset_, 
                            xyz(1) + j * xy_offset_, 
                            xyz(2));
        Eigen::AngleAxisf yawAngle(rpy(2) + k * yaw_resolution_, 
                                   Eigen::Vector3f::UnitZ());
        
        Eigen::Matrix4f temp_pose = Eigen::Matrix4f::Identity();
        temp_pose.block<3, 3>(0, 0) = 
            (rollAngle * pitchAngle * yawAngle).toRotationMatrix();
        temp_pose.block<3, 1>(0, 3) = pos;
        candidates.push_back(temp_pose);
      }
    }
  }

  RCLCPP_INFO(this->get_logger(), 
              "Initial guess: [%.2f, %.2f, %.2f], RPY: [%.2f, %.2f, %.2f], "
              "candidates: %zu",
              xyz(0), xyz(1), xyz(2), rpy(0), rpy(1), rpy(2), 
              candidates.size());

  // 复用类成员缓冲区进行体素滤波
  rough_source_->clear();
  refine_source_->clear();
  
  voxel_rough_filter_.setInputCloud(source);
  voxel_rough_filter_.filter(*rough_source_);
  
  voxel_refine_filter_.setInputCloud(source);
  voxel_refine_filter_.filter(*refine_source_);

  // 复用类成员计算法线
  addNormInPlace(rough_source_, rough_source_norm_);
  addNormInPlace(refine_source_, refine_source_norm_);
  
  // 清空对齐结果缓冲区
  align_point_->clear();

  // 粗匹配：遍历所有候选位姿，找最优解
  Eigen::Matrix4f best_rough_transform = Eigen::Matrix4f::Identity();
  double best_rough_score = std::numeric_limits<double>::max();
  bool rough_converge = false;
  
  auto tic = std::chrono::system_clock::now();
  
  icp_rough_.setInputSource(rough_source_norm_);
  
  for (const Eigen::Matrix4f &init_pose : candidates) {
    icp_rough_.align(*align_point_, init_pose);
    
    if (!icp_rough_.hasConverged())
      continue;
      
    double rough_score = icp_rough_.getFitnessScore();
    
    if (rough_score > 2 * thresh_)
      continue;
      
    if (rough_score < best_rough_score) {
      best_rough_score = rough_score;
      rough_converge = true;
      best_rough_transform = icp_rough_.getFinalTransformation();
    }
  }

  if (!rough_converge) {
    RCLCPP_WARN(this->get_logger(), 
                "Rough ICP failed to converge for all %zu candidates", 
                candidates.size());
    return Eigen::Matrix4d::Zero();
  }

  // 精匹配：基于粗匹配最优结果细化
  icp_refine_.setInputSource(refine_source_norm_);
  icp_refine_.align(*align_point_, best_rough_transform);
  
  score_ = icp_refine_.getFitnessScore();

  if (!icp_refine_.hasConverged() || score_ > thresh_) {
    RCLCPP_WARN(this->get_logger(), 
                "Refine ICP failed: converged=%d, score=%.4f (thresh=%.4f)",
                icp_refine_.hasConverged(), score_, thresh_);
    return Eigen::Matrix4d::Zero();
  }
    
  success_ = true;
  
  auto toc = std::chrono::system_clock::now();
  std::chrono::duration<double> duration = toc - tic;
  
  RCLCPP_INFO(this->get_logger(), 
              "ICP alignment succeeded: %.2f ms, fitness score: %.4f",
              duration.count() * 1000, score_);

  return icp_refine_.getFinalTransformation().cast<double>();
}

PointCloudXYZIN::Ptr
IcpNode::addNorm(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) {
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<pcl::PointXYZI>::Ptr searchTree(
      new pcl::search::KdTree<pcl::PointXYZI>);
  searchTree->setInputCloud(cloud);

  pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> normalEstimator;
  normalEstimator.setInputCloud(cloud);
  normalEstimator.setSearchMethod(searchTree);
  normalEstimator.setKSearch(15);
  normalEstimator.compute(*normals);
  
  PointCloudXYZIN::Ptr out(new PointCloudXYZIN);
  pcl::concatenateFields(*cloud, *normals, *out);
  return out;
}

void IcpNode::addNormInPlace(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                             PointCloudXYZIN::Ptr output) {
  // 清空法线缓冲区（保留内存容量）
  normals_buffer_->clear();
  
  // 复用 KD-Tree（设置新输入云）
  kdtree_buffer_->setInputCloud(cloud);
  
  // 法线估计
  pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> ne;
  ne.setInputCloud(cloud);
  ne.setSearchMethod(kdtree_buffer_);
  ne.setKSearch(15);
  ne.compute(*normals_buffer_);
  
  // 清空输出缓冲区（保留内存容量）
  output->clear();
  
  // 合并点云和法线到输出
  pcl::concatenateFields(*cloud, *normals_buffer_, *output);
}

} // namespace icp

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(icp::IcpNode)
