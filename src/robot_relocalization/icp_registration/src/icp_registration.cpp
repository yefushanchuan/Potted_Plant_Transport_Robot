#include "icp_registration/icp_registration.hpp"

#include <Eigen/src/Geometry/Quaternion.h>
#include <Eigen/src/Geometry/Transform.h>

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
    : Node("icp_registration", options), rough_iter_(10), refine_iter_(5) {
    
    cloud_in_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(
        new pcl::PointCloud<pcl::PointXYZI>);
    
    double rough_leaf_size  = this->declare_parameter("rough_leaf_size", 0.4);
    double refine_leaf_size = this->declare_parameter("refine_leaf_size", 0.1);
    
    voxel_rough_filter_.setLeafSize(rough_leaf_size, rough_leaf_size,
                                    rough_leaf_size);
    voxel_refine_filter_.setLeafSize(refine_leaf_size, refine_leaf_size,
                                     refine_leaf_size);

    pcd_path_ = this->declare_parameter("pcd_path", std::string(""));
    if (!std::filesystem::exists(pcd_path_)) {
        RCLCPP_ERROR(this->get_logger(), "Invalid pcd path: %s", pcd_path_.c_str());
        throw std::runtime_error("Invalid pcd path");
    }

    // Read the pcd file
    pcl::PCDReader reader;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    reader.read(pcd_path_, *cloud);
    voxel_refine_filter_.setInputCloud(cloud);
    voxel_refine_filter_.filter(*cloud);

    // Add normal to the pointcloud
    refine_map_ = addNorm(cloud);
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr point_rough(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterd_point_rough(new pcl::PointCloud<pcl::PointXYZI>);
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
    map_frame_id_  = this->declare_parameter("map_frame_id", std::string("map"));
    odom_frame_id_ = this->declare_parameter("odom_frame_id", std::string("odom"));
    laser_frame_id_ = this->declare_parameter("laser_frame_id", std::string("laser"));
    base_frame_id_  = this->declare_parameter("base_frame_id", std::string("base_link"));
    pose_topic_    = this->declare_parameter("pose_topic", std::string("/icp_pose"));
    publish_pose_  = this->declare_parameter("publish_pose", true);
    thresh_        = this->declare_parameter("thresh", 0.15);
    xy_offset_     = this->declare_parameter("xy_offset", 0.2);
    
    double yaw_offset_deg     = this->declare_parameter("yaw_offset", 30.0);
    double yaw_resolution_deg = this->declare_parameter("yaw_resolution", 10.0);
    yaw_steps_    = std::round(yaw_offset_deg / yaw_resolution_deg);
    yaw_resolution_ = yaw_resolution_deg * M_PI / 180.0;

    std::vector<double> initial_pose_vec = this->declare_parameter(
        "initial_pose", std::vector<double>{0, 0, 0, 0, 0, 0});
    try {
        initial_pose_.position.x = initial_pose_vec.at(0);
        initial_pose_.position.y = initial_pose_vec.at(1);
        initial_pose_.position.z = initial_pose_vec.at(2);
        tf2::Quaternion q;
        q.setRPY(initial_pose_vec.at(3), initial_pose_vec.at(4),
                 initial_pose_vec.at(5));
    } catch (const std::out_of_range &ex) {
        RCLCPP_ERROR(this->get_logger(),
                     "initial_pose is not a vector with 6 elements, what():%s",
                     ex.what());
    }

    publish_tf_ = this->declare_parameter("publish_tf", false);

    if (publish_tf_) {
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        tf_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            [this]() {
                if (has_calculated_pose_) {
                    map_to_odom_tf_.header.stamp = this->now();
                    tf_broadcaster_->sendTransform(map_to_odom_tf_);
                }
            });
        RCLCPP_INFO(this->get_logger(), "Node A: TF 发布已启用 (独立运行模式)");
    } else {
        RCLCPP_INFO(this->get_logger(), "Node A: TF 发布已禁用 (作为重定位服务模块)");
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
    std::string rviz_initial_pose_topic = this->declare_parameter(
        "rviz_initial_pose_topic", std::string("/initialpose"));
    initial_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        rviz_initial_pose_topic, qos,
        [this](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
            initialPoseCallback(msg);
        });

    // Set up the transform broadcaster
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        this->get_node_base_interface(), this->get_node_timers_interface());
    tf_buffer_->setCreateTimerInterface(timer_interface);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Set up the map publisher
    map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "/map_pointcloud", rclcpp::QoS(1).transient_local());
    
    if (publish_pose_) {
        reloc_pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            pose_topic_, rclcpp::QoS(10));
        RCLCPP_INFO(this->get_logger(), 
                    "Publishing ICP pose: %s (frame=%s, base=%s)",
                    pose_topic_.c_str(), map_frame_id_.c_str(), base_frame_id_.c_str());
    }

    // Publish the map once
    sensor_msgs::msg::PointCloud2 map_msg;
    pcl::toROSMsg(*cloud, map_msg);
    map_msg.header.frame_id = map_frame_id_;
    map_msg.header.stamp    = now();
    map_pub_->publish(map_msg);
    RCLCPP_INFO(this->get_logger(), "Published global map with %ld points", cloud->size());

    RCLCPP_INFO(this->get_logger(), "icp_registration initialized");
}

IcpNode::~IcpNode() {}

void IcpNode::pointcloudCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    pcl::fromROSMsg(*msg, *cloud_in_);
    has_cloud_ = true;
}

void IcpNode::initialPoseCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    
    if (!has_cloud_) {
        RCLCPP_WARN(this->get_logger(), "还没收到雷达点云，无法执行暴力重定位！");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "收到初始猜测，开始唤醒暴力 ICP 搜索...");

    // 1. 读取 RViz 给出的猜测位姿
    Eigen::Vector3d pos(msg->pose.pose.position.x, 
                        msg->pose.pose.position.y, 
                        msg->pose.pose.position.z);
    Eigen::Quaterniond q(msg->pose.pose.orientation.w, 
                         msg->pose.pose.orientation.x,
                         msg->pose.pose.orientation.y, 
                         msg->pose.pose.orientation.z);
    Eigen::Matrix4d guess_map_to_base = Eigen::Matrix4d::Identity();
    guess_map_to_base.block<3, 3>(0, 0) = q.toRotationMatrix();
    guess_map_to_base.block<3, 1>(0, 3) = pos;

    Eigen::Matrix4d base_to_laser = Eigen::Matrix4d::Identity();
    try {
        // 查询 base_footprint 到 front_laser_link 的变换 (T_base_laser)
        auto transform = tf_buffer_->lookupTransform(
            base_frame_id_, laser_frame_id_, tf2::TimePointZero);
        Eigen::Vector3d t_bl(transform.transform.translation.x, 
                             transform.transform.translation.y, 
                             transform.transform.translation.z);
        Eigen::Quaterniond q_bl(transform.transform.rotation.w, 
                                transform.transform.rotation.x,
                                transform.transform.rotation.y, 
                                transform.transform.rotation.z);
        base_to_laser.block<3, 3>(0, 0) = q_bl.toRotationMatrix();
        base_to_laser.block<3, 1>(0, 3) = t_bl;
    } catch (tf2::TransformException &ex) {
        RCLCPP_ERROR(this->get_logger(), "获取初始猜测时 TF (base->laser) 查询失败: %s", ex.what());
        return;
    }

    // 计算真正的 map -> laser 初始猜测： T_map_laser = T_map_base * T_base_laser
    Eigen::Matrix4d initial_guess_laser = guess_map_to_base * base_to_laser;

    // 2. 进行多假设暴力匹配 (传入的是雷达在地图中的猜测位姿)
    Eigen::Matrix4d map_to_laser = multiAlignSync(cloud_in_, initial_guess_laser);
    if (!success_) {
        RCLCPP_ERROR(this->get_logger(), "Node A 暴力重定位失败，周围环境可能不匹配！");
        return;
    }

    // 3. 将 map -> laser 转换回 map -> base_footprint (这部分你原本的代码是对的)
    Eigen::Matrix4d laser_to_base = Eigen::Matrix4d::Identity();
    try {
        auto transform = tf_buffer_->lookupTransform(
            laser_frame_id_, base_frame_id_, tf2::TimePointZero);
        Eigen::Vector3d t(transform.transform.translation.x, 
                          transform.transform.translation.y, 
                          transform.transform.translation.z);
        Eigen::Quaterniond q_tf(transform.transform.rotation.w, 
                                  transform.transform.rotation.x,
                                  transform.transform.rotation.y, 
                                  transform.transform.rotation.z);
        laser_to_base.block<3, 3>(0, 0) = q_tf.toRotationMatrix();
        laser_to_base.block<3, 1>(0, 3) = t;
    } catch (tf2::TransformException &ex) {
        RCLCPP_ERROR(this->get_logger(), "TF 外参查询失败: %s", ex.what());
        return;
    }

    // T_map_base = T_map_laser * T_laser_base
    Eigen::Matrix4d map_to_base = map_to_laser * laser_to_base;

    // 4. 将最终结果打包，发布给 Node B
    geometry_msgs::msg::PoseWithCovarianceStamped exact_pose_msg;
    exact_pose_msg.header.stamp    = now();
    exact_pose_msg.header.frame_id = map_frame_id_;

    exact_pose_msg.pose.pose.position.x = map_to_base(0, 3);
    exact_pose_msg.pose.pose.position.y = map_to_base(1, 3);
    exact_pose_msg.pose.pose.position.z = map_to_base(2, 3);

    Eigen::Quaterniond q_res(map_to_base.block<3, 3>(0, 0));
    exact_pose_msg.pose.pose.orientation.w = q_res.w();
    exact_pose_msg.pose.pose.orientation.x = q_res.x();
    exact_pose_msg.pose.pose.orientation.y = q_res.y();
    exact_pose_msg.pose.pose.orientation.z = q_res.z();

    reloc_pose_pub_->publish(exact_pose_msg);
    RCLCPP_INFO(this->get_logger(), 
                "🎯 暴力计算完成！已发布精确位姿给主节点 Node B: [%.2f, %.2f, %.2f]",
                map_to_base(0, 3), map_to_base(1, 3), map_to_base(2, 3));

    if (publish_tf_) {
        try {
            auto odom_to_base_msg = tf_buffer_->lookupTransform(
                odom_frame_id_, base_frame_id_, tf2::TimePointZero);
            
            Eigen::Vector3d t_ob(odom_to_base_msg.transform.translation.x, 
                                   odom_to_base_msg.transform.translation.y, 
                                   odom_to_base_msg.transform.translation.z);
            Eigen::Quaterniond q_ob(odom_to_base_msg.transform.rotation.w, 
                                     odom_to_base_msg.transform.rotation.x, 
                                     odom_to_base_msg.transform.rotation.y, 
                                     odom_to_base_msg.transform.rotation.z);
            Eigen::Matrix4d odom_to_base = Eigen::Matrix4d::Identity();
            odom_to_base.block<3, 3>(0, 0) = q_ob.toRotationMatrix();
            odom_to_base.block<3, 1>(0, 3) = t_ob;

            // map -> odom = map -> base * (odom -> base)^-1
            Eigen::Matrix4d map_to_odom = map_to_base * odom_to_base.inverse();

            map_to_odom_tf_.header.frame_id    = map_frame_id_;
            map_to_odom_tf_.child_frame_id     = odom_frame_id_;
            map_to_odom_tf_.transform.translation.x = map_to_odom(0, 3);
            map_to_odom_tf_.transform.translation.y = map_to_odom(1, 3);
            map_to_odom_tf_.transform.translation.z = map_to_odom(2, 3);
            
            Eigen::Quaterniond q_mo(map_to_odom.block<3, 3>(0, 0));
            map_to_odom_tf_.transform.rotation.w = q_mo.w();
            map_to_odom_tf_.transform.rotation.x = q_mo.x();
            map_to_odom_tf_.transform.rotation.y = q_mo.y();
            map_to_odom_tf_.transform.rotation.z = q_mo.z();

            has_calculated_pose_ = true;
        } catch (tf2::TransformException &ex) {
            RCLCPP_ERROR(this->get_logger(), "计算 map->odom TF 失败: %s", ex.what());
        }
    }
}

Eigen::Matrix4d IcpNode::multiAlignSync(PointCloudXYZI::Ptr source,
                                        const Eigen::Matrix4d &init_guess) {
    
    static auto rotate2rpy = [](Eigen::Matrix3d &rot) -> Eigen::Vector3d {
        double roll  = std::atan2(rot(2, 1), rot(2, 2));
        double pitch = asin(-rot(2, 0));
        double yaw   = std::atan2(rot(1, 0), rot(0, 0));
        return Eigen::Vector3d(roll, pitch, yaw);
    };

    success_ = false;
    Eigen::Vector3d xyz      = init_guess.block<3, 1>(0, 3);
    Eigen::Matrix3d rotation = init_guess.block<3, 3>(0, 0);
    Eigen::Vector3d rpy      = rotate2rpy(rotation);
    
    Eigen::AngleAxisf rollAngle(rpy(0), Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf pitchAngle(rpy(1), Eigen::Vector3f::UnitY());
    
    std::vector<Eigen::Matrix4f> candidates;
    Eigen::Matrix4f temp_pose;

    RCLCPP_INFO(this->get_logger(), "initial guess: %f, %f, %f, %f, %f, %f",
                xyz(0), xyz(1), xyz(2), rpy(0), rpy(1), rpy(2));

    for (int i = -1; i <= 1; i++) {
        for (int j = -1; j <= 1; j++) {
            for (int k = -yaw_steps_; k <= yaw_steps_; k++) {
                Eigen::Vector3f pos(xyz(0) + i * xy_offset_, 
                                    xyz(1) + j * xy_offset_,
                                    xyz(2));
                Eigen::AngleAxisf yawAngle(rpy(2) + k * yaw_resolution_,
                                           Eigen::Vector3f::UnitZ());
                temp_pose.setIdentity();
                temp_pose.block<3, 3>(0, 0) = (yawAngle * pitchAngle * rollAngle).toRotationMatrix();
                temp_pose.block<3, 1>(0, 3) = pos;
                candidates.push_back(temp_pose);
            }
        }
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr rough_source(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr refine_source(new pcl::PointCloud<pcl::PointXYZI>);

    voxel_rough_filter_.setInputCloud(source);
    voxel_rough_filter_.filter(*rough_source);
    voxel_refine_filter_.setInputCloud(source);
    voxel_refine_filter_.filter(*refine_source);

    PointCloudXYZIN::Ptr rough_source_norm  = addNorm(rough_source);
    PointCloudXYZIN::Ptr refine_source_norm = addNorm(refine_source);
    PointCloudXYZIN::Ptr align_point(new PointCloudXYZIN);

    Eigen::Matrix4f best_rough_transform;
    double best_rough_score = 10.0;
    bool rough_converge     = false;
    
    auto tic = std::chrono::system_clock::now();
    
    for (Eigen::Matrix4f &init_pose : candidates) {
        icp_rough_.setInputSource(rough_source_norm);
        icp_rough_.align(*align_point, init_pose);
        if (!icp_rough_.hasConverged())
            continue;
        double rough_score = icp_rough_.getFitnessScore();
        if (rough_score > 2 * thresh_)
            continue;
        if (rough_score < best_rough_score) {
            best_rough_score    = rough_score;
            rough_converge      = true;
            best_rough_transform = icp_rough_.getFinalTransformation();
        }
    }

    if (!rough_converge)
        return Eigen::Matrix4d::Zero();

    icp_refine_.setInputSource(refine_source_norm);
    icp_refine_.align(*align_point, best_rough_transform);
    score_ = icp_refine_.getFitnessScore();

    if (!icp_refine_.hasConverged())
        return Eigen::Matrix4d::Zero();
    if (score_ > thresh_)
        return Eigen::Matrix4d::Zero();
    
    success_ = true;
    
    auto toc      = std::chrono::system_clock::now();
    std::chrono::duration<double> duration = toc - tic;
    
    RCLCPP_INFO(this->get_logger(), "align used: %f ms", duration.count() * 1000);
    RCLCPP_INFO(this->get_logger(), "score: %f", score_);

    return icp_refine_.getFinalTransformation().cast<double>();
}

PointCloudXYZIN::Ptr IcpNode::addNorm(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) {
    
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

}  // namespace icp

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(icp::IcpNode)
