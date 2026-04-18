#include "icp_registration/icp_registration.hpp"

#include <Eigen/Geometry>

#include <chrono>
#include <iostream>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <rclcpp/qos.hpp>
#include <stdexcept>
#include <tf2_ros/create_timer_ros.h>

namespace icp {

IcpNode::IcpNode(const rclcpp::NodeOptions &options)
    : Node("icp_registration", options), rough_iter_(20), refine_iter_(5) {
    cloud_in_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(
        new pcl::PointCloud<pcl::PointXYZI>);

    double rough_leaf_size = this->declare_parameter("rough_leaf_size", 0.4);
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
    icp_rough_.setMaxCorrespondenceDistance(5.0); 

    icp_refine_.setMaximumIterations(refine_iter_);
    icp_refine_.setInputTarget(refine_map_);
    icp_refine_.setMaxCorrespondenceDistance(3.0);

    RCLCPP_INFO(this->get_logger(), "pcd point size: %ld, %ld",
                refine_map_->size(), rough_map_->size());

    // Parameters
    map_frame_id_ = this->declare_parameter("map_frame_id", std::string("map"));
    odom_frame_id_ = this->declare_parameter("odom_frame_id", std::string("odom"));
    base_frame_id_ = this->declare_parameter("base_frame_id", std::string("base_footprint"));
    laser_frame_id_ = this->declare_parameter("laser_frame_id", std::string("front_laser_link"));
    pose_topic_ = this->declare_parameter("pose_topic", std::string("/icp_pose"));
    publish_pose_ = this->declare_parameter("publish_pose", true);
    thresh_ = this->declare_parameter("thresh", 0.15);
    xy_offset_ = this->declare_parameter("xy_offset", 0.2);

    double yaw_offset_deg = this->declare_parameter("yaw_offset", 30.0);
    double yaw_resolution_deg = this->declare_parameter("yaw_resolution", 10.0);
    yaw_steps_ = std::round(yaw_offset_deg / yaw_resolution_deg);
    yaw_resolution_ = yaw_resolution_deg * M_PI / 180.0;

    std::vector<double> initial_pose_vec = this->declare_parameter(
        "initial_pose", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    use_yaml_initial_pose_ = this->declare_parameter("use_yaml_initial_pose", true);

    try {
        initial_pose_msg_.position.x = initial_pose_vec.at(0);
        initial_pose_msg_.position.y = initial_pose_vec.at(1);
        initial_pose_msg_.position.z = initial_pose_vec.at(2);

        tf2::Quaternion q;
        q.setRPY(initial_pose_vec.at(3), initial_pose_vec.at(4), initial_pose_vec.at(5));

        initial_pose_msg_.orientation.x = q.x();
        initial_pose_msg_.orientation.y = q.y();
        initial_pose_msg_.orientation.z = q.z();
        initial_pose_msg_.orientation.w = q.w();

        RCLCPP_INFO(this->get_logger(), "已加载 YAML 初始发车点位姿: X=%.2f Y=%.2f Yaw=%.2f",
                    initial_pose_msg_.position.x, initial_pose_msg_.position.y, initial_pose_vec.at(5));
    } catch (const std::out_of_range &ex) {
        RCLCPP_ERROR(this->get_logger(), "initial_pose 参数不合法: %s", ex.what());
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
        RCLCPP_INFO(this->get_logger(), "Node A: TF 发布已启用");
    }

    // 设置 TF buffer 和 listener
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        this->get_node_base_interface(), this->get_node_timers_interface());
    tf_buffer_->setCreateTimerInterface(timer_interface);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    init_tf_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&IcpNode::getStaticTf, this));

    // Set up the pointcloud subscriber
    std::string pointcloud_topic = this->declare_parameter(
        "pointcloud_topic", std::string("/livox/lidar"));
    RCLCPP_INFO(this->get_logger(), "pointcloud_topic: %s", pointcloud_topic.c_str());

    pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        pointcloud_topic, rclcpp::QoS(1),
        std::bind(&IcpNode::pointcloudCallback, this, std::placeholders::_1));

    // Set up the initial pose subscriber
    std::string rviz_initial_pose_topic = this->declare_parameter(
        "rviz_initial_pose_topic", std::string("/initialpose"));
    initial_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        rviz_initial_pose_topic, rclcpp::QoS(1),
        [this](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
            initialPoseCallback(msg);
        });

    // Set up the map publisher
    map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "/map_pointcloud", rclcpp::QoS(1).transient_local());

    if (publish_pose_) {
        reloc_pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            pose_topic_, rclcpp::QoS(10));
    }

    // Publish the map once
    sensor_msgs::msg::PointCloud2 map_msg;
    pcl::toROSMsg(*cloud, map_msg);
    map_msg.header.frame_id = map_frame_id_;
    map_msg.header.stamp = now();
    map_pub_->publish(map_msg);

    RCLCPP_INFO(this->get_logger(), "icp_registration initialized");
}

IcpNode::~IcpNode() {}

void IcpNode::pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    if (!has_sensor_tf_) {
        return;
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_raw(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg, *cloud_raw);

    pcl::PointCloud<pcl::PointXYZI>::Ptr current_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::transformPointCloud(*cloud_raw, *current_cloud, base_to_sensor_T_);

    cloud_in_ = current_cloud;  // 更新全局指针
    has_cloud_ = true;

    if (use_yaml_initial_pose_) {
        RCLCPP_INFO_ONCE(this->get_logger(), "🚀 检测到点云，启动 YAML 参数自动重定位...");

        Eigen::Vector3d pos(initial_pose_msg_.position.x,
                            initial_pose_msg_.position.y,
                            initial_pose_msg_.position.z);
        Eigen::Quaterniond q_init(initial_pose_msg_.orientation.w,
                                  initial_pose_msg_.orientation.x,
                                  initial_pose_msg_.orientation.y,
                                  initial_pose_msg_.orientation.z);
        Eigen::Matrix4d guess_map_to_base = Eigen::Matrix4d::Identity();
        guess_map_to_base.block<3, 3>(0, 0) = q_init.toRotationMatrix();
        guess_map_to_base.block<3, 1>(0, 3) = pos;

        // 传入深拷贝/安全的点云
        Eigen::Matrix4d map_to_base = multiAlignSync(cloud_in_, guess_map_to_base);

        if (success_) {
            geometry_msgs::msg::PoseWithCovarianceStamped exact_pose_msg;
            exact_pose_msg.header.stamp = now();
            exact_pose_msg.header.frame_id = map_frame_id_;
            exact_pose_msg.pose.pose.position.x = map_to_base(0, 3);
            exact_pose_msg.pose.pose.position.y = map_to_base(1, 3);
            exact_pose_msg.pose.pose.position.z = map_to_base(2, 3);
            Eigen::Quaterniond q_res(map_to_base.block<3, 3>(0, 0));
            exact_pose_msg.pose.pose.orientation.w = q_res.w();
            exact_pose_msg.pose.pose.orientation.x = q_res.x();
            exact_pose_msg.pose.pose.orientation.y = q_res.y();
            exact_pose_msg.pose.pose.orientation.z = q_res.z();

            if (publish_pose_) reloc_pose_pub_->publish(exact_pose_msg);
            RCLCPP_INFO(this->get_logger(), "✅ 自动重定位成功！已发送至主节点");

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

                    Eigen::Matrix4d map_to_odom = map_to_base * odom_to_base.inverse();

                    map_to_odom_tf_.header.frame_id = map_frame_id_;
                    map_to_odom_tf_.child_frame_id = odom_frame_id_;
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
                    RCLCPP_ERROR(this->get_logger(), "自动发车计算 map->odom TF 失败: %s", ex.what());
                }
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "⚠️ 自动重定位发散！当前环境可能与 YAML 设定出生点不符。请在 RViz 使用 2D Pose Estimate 工具手动标定。");
        }
        use_yaml_initial_pose_ = false;
    }
}

void IcpNode::initialPoseCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    if (!has_cloud_) {
        RCLCPP_WARN(this->get_logger(), "还没收到并转换雷达点云，无法执行暴力重定位！");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "收到初始猜测，开始唤醒暴力 ICP 搜索...");

    // 1. 读取 RViz 给出的猜测位姿 (这本身就是 map -> base_footprint 的纯平位姿)
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

    Eigen::Matrix4d map_to_base = multiAlignSync(cloud_in_, guess_map_to_base);

    if (!success_) {
        RCLCPP_ERROR(this->get_logger(), "Node A 暴力重定位失败，周围环境可能不匹配！");
        return;
    }

    // 2. 将最终结果打包，发布给 Node B
    geometry_msgs::msg::PoseWithCovarianceStamped exact_pose_msg;
    exact_pose_msg.header.stamp = now();
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

            Eigen::Matrix4d map_to_odom = map_to_base * odom_to_base.inverse();

            map_to_odom_tf_.header.frame_id = map_frame_id_;
            map_to_odom_tf_.child_frame_id = odom_frame_id_;
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
        double roll = std::atan2(rot(2, 1), rot(2, 2));
        double pitch = asin(-rot(2, 0));
        double yaw = std::atan2(rot(1, 0), rot(0, 0));
        return Eigen::Vector3d(roll, pitch, yaw);
    };

    success_ = false;
    Eigen::Vector3d xyz = init_guess.block<3, 1>(0, 3);
    Eigen::Matrix3d rotation = init_guess.block<3, 3>(0, 0);
    Eigen::Vector3d rpy = rotate2rpy(rotation);

    Eigen::AngleAxisf rollAngle(rpy(0), Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf pitchAngle(rpy(1), Eigen::Vector3f::UnitY());

    std::vector<Eigen::Matrix4f> candidates;
    Eigen::Matrix4f temp_pose;

    RCLCPP_INFO(this->get_logger(), "开始匹配 | Guess: X=%.2f Y=%.2f Yaw=%.2f",
                xyz(0), xyz(1), rpy(2) * 180.0 / M_PI);

    for (int i = -2; i <= 2; i++) {
        for (int j = -2; j <= 2; j++) {
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

    PointCloudXYZIN::Ptr rough_source_norm = addNorm(rough_source);
    PointCloudXYZIN::Ptr refine_source_norm = addNorm(refine_source);
    PointCloudXYZIN::Ptr align_point(new PointCloudXYZIN);

    Eigen::Matrix4f best_rough_transform;
    double best_rough_score = 10.0;
    bool rough_converge = false;

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
            best_rough_score = rough_score;
            rough_converge = true;
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

    auto toc = std::chrono::system_clock::now();
    std::chrono::duration<double> duration = toc - tic;

    RCLCPP_INFO(this->get_logger(), "✅ 匹配成功! 耗时: %.2f ms | Score: %.3f",
                duration.count() * 1000, score_);

    return icp_refine_.getFinalTransformation().cast<double>();
}

void IcpNode::getStaticTf() {
    try {
        // 直接使用 YAML 里的 laser_frame_id_ 查询
        auto transform = tf_buffer_->lookupTransform(
            base_frame_id_, laser_frame_id_, tf2::TimePointZero);

        Eigen::Quaternionf q(transform.transform.rotation.w,
                             transform.transform.rotation.x,
                             transform.transform.rotation.y,
                             transform.transform.rotation.z);
        Eigen::Vector3f t(transform.transform.translation.x,
                          transform.transform.translation.y,
                          transform.transform.translation.z);

        base_to_sensor_T_ = Eigen::Matrix4f::Identity();
        base_to_sensor_T_.block<3, 3>(0, 0) = q.toRotationMatrix();
        base_to_sensor_T_.block<3, 1>(0, 3) = t;

        has_sensor_tf_ = true;  // 标志位置为 true

        RCLCPP_INFO(this->get_logger(), "✅ 成功抓取并缓存静态外参 TF: %s -> %s",
                    base_frame_id_.c_str(), laser_frame_id_.c_str());

        init_tf_timer_->cancel();

    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 100,
                             "等待静态外参 TF 树建立...");
    }
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
