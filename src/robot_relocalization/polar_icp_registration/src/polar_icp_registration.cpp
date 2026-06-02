#include "polar_icp_registration/polar_icp_registration.hpp"

#include <Eigen/Geometry>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <numeric>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/qos.hpp>
#include <stdexcept>
#include <tf2_ros/create_timer_ros.h>

namespace polar_icp {

// ── 辅助函数：Eigen::Quaterniond → geometry_msgs Quaternion ──
static geometry_msgs::msg::Quaternion eigenQuatToMsg(const Eigen::Quaterniond &q) {
    geometry_msgs::msg::Quaternion msg;
    msg.x = q.x();
    msg.y = q.y();
    msg.z = q.z();
    msg.w = q.w();
    return msg;
}

// ══════════════════════════════════════════════════════════════
//  关键帧数据库加载
// ══════════════════════════════════════════════════════════════

void PolarIcpNode::loadKeyframeDatabase(const std::string &db_dir) {
    namespace fs = std::filesystem;
    std::string bin_path = db_dir + "/keyframes.bin";
    std::string clouds_dir = db_dir + "/clouds";

    if (!fs::exists(bin_path) || !fs::is_directory(clouds_dir)) {
        RCLCPP_WARN(this->get_logger(), "关键帧数据库不存在: %s, 使用单地图模式", db_dir.c_str());
        return;
    }

    std::ifstream f(bin_path, std::ios::binary);
    uint32_t count = 0;
    int num_bins = 0;
    f.read(reinterpret_cast<char *>(&count), sizeof(count));
    f.read(reinterpret_cast<char *>(&num_bins), sizeof(num_bins));

    if (num_bins != ring_bins_) {
        RCLCPP_WARN(this->get_logger(), "关键帧 bin 数(%d) != 配置(%d), 使用关键帧 bin 数",
                     num_bins, ring_bins_);
    }

    keyframes_.resize(count);
    for (uint32_t i = 0; i < count; i++) {
        auto &kf = keyframes_[i];

        // 读位姿
        double x, y, z, qw, qx, qy, qz;
        f.read(reinterpret_cast<char *>(&x), sizeof(double));
        f.read(reinterpret_cast<char *>(&y), sizeof(double));
        f.read(reinterpret_cast<char *>(&z), sizeof(double));
        f.read(reinterpret_cast<char *>(&qw), sizeof(double));
        f.read(reinterpret_cast<char *>(&qx), sizeof(double));
        f.read(reinterpret_cast<char *>(&qy), sizeof(double));
        f.read(reinterpret_cast<char *>(&qz), sizeof(double));

        kf.pose = Eigen::Matrix4d::Identity();
        kf.pose.block<3, 3>(0, 0) = Eigen::Quaterniond(qw, qx, qy, qz).toRotationMatrix();
        kf.pose(0, 3) = x;
        kf.pose(1, 3) = y;
        kf.pose(2, 3) = z;

        // 读环特征
        kf.ring.num_bins = num_bins;
        kf.ring.bin_angle = 2.0f * M_PI / num_bins;
        kf.ring.max_rho.resize(num_bins);
        f.read(reinterpret_cast<char *>(kf.ring.max_rho.data()), num_bins * sizeof(float));

        // 读点云
        std::string pcd_path = clouds_dir + "/" + std::to_string(i) + ".pcd";
        kf.cloud.reset(new CloudXYZI);
        if (pcl::io::loadPCDFile(pcd_path, *kf.cloud) < 0) {
            RCLCPP_ERROR(this->get_logger(), "无法加载关键帧点云: %s", pcd_path.c_str());
            keyframes_.clear();
            return;
        }

        // 构建八叉树（用于精配准）
        float res = octree_resolutions_.empty() ? 0.5f : static_cast<float>(octree_resolutions_.back());
        kf.octree.reset(new pcl::octree::OctreePointCloudSearch<PointXYZI>(res));
        kf.octree->setInputCloud(kf.cloud);
        kf.octree->addPointsFromInputCloud();

        // 预计算法线（避免精配准时重复计算）
        kf.cloud_with_normals = addNormals(kf.cloud);
    }
    f.close();

    use_keyframes_ = true;
    RCLCPP_INFO(this->get_logger(), "已加载 %zu 个关键帧", keyframes_.size());
}

// ══════════════════════════════════════════════════════════════
//  构造函数
// ══════════════════════════════════════════════════════════════

PolarIcpNode::PolarIcpNode(const rclcpp::NodeOptions &options)
    : Node("polar_icp_registration", options),
      cloud_in_(new CloudXYZI),
      raw_map_(new CloudXYZI),
      refine_map_(new CloudXYZIN) {

    // ── 加载 PCD 地图 ──
    std::string pcd_path = this->declare_parameter("pcd_path", std::string(""));
    if (!std::filesystem::exists(pcd_path)) {
        RCLCPP_ERROR(this->get_logger(), "Invalid pcd path: %s", pcd_path.c_str());
        throw std::runtime_error("Invalid pcd path");
    }
    pcl::PCDReader reader;
    reader.read(pcd_path, *raw_map_);
    RCLCPP_INFO(this->get_logger(), "Loaded map: %zu points", raw_map_->size());

    // ── 体素滤波参数 ──
    double refine_leaf = this->declare_parameter("refine_leaf_size", 0.1);
    voxel_refine_filter_.setLeafSize(refine_leaf, refine_leaf, refine_leaf);

    // ── 精配准地图（带法线，非关键帧模式用） ──
    CloudXYZI::Ptr refine_cloud(new CloudXYZI);
    voxel_refine_filter_.setInputCloud(raw_map_);
    voxel_refine_filter_.filter(*refine_cloud);
    refine_map_ = addNormals(refine_cloud);

    // ── 极坐标环参数 ──
    ring_bins_       = this->declare_parameter("ring_bins", 360);
    bev_z_min_       = this->declare_parameter("bev_z_min", -0.5);
    bev_z_max_       = this->declare_parameter("bev_z_max", 2.0);
    bev_crop_radius_ = this->declare_parameter("bev_crop_radius", 30.0);

    // ── 多分辨率 ICP 参数 ──
    refine_iter_      = this->declare_parameter("refine_iter", 5);
    thresh_           = this->declare_parameter("thresh", 1.5);
    drift_threshold_  = this->declare_parameter("drift_threshold", 3.0);
    octree_resolutions_ = this->declare_parameter(
        "octree_resolutions", std::vector<double>{2.0, 1.0, 0.5, 0.2});

    // ── 关键帧数据库 ──
    keyframe_db_path_ = this->declare_parameter("keyframe_db", std::string(""));
    if (!keyframe_db_path_.empty()) {
        loadKeyframeDatabase(keyframe_db_path_);
    }

    // ── 非关键帧模式：预计算地图环特征和八叉树层级 ──
    if (!use_keyframes_) {
        map_ring_ = extractPolarRing(raw_map_);
        // buildOctreeLevels 已移除，直接用 refine_map_ 做精配准
    }

    // ── 精配准 ICP 配置 ──
    icp_refine_.setMaximumIterations(refine_iter_);
    icp_refine_.setInputTarget(refine_map_);
    icp_refine_.setMaxCorrespondenceDistance(this->declare_parameter("refine_max_corr_dist", 1.0));

    // ── TF / 话题参数 ──
    map_frame_id_   = this->declare_parameter("map_frame_id", std::string("map"));
    odom_frame_id_  = this->declare_parameter("odom_frame_id", std::string("odom"));
    base_frame_id_  = this->declare_parameter("base_frame_id", std::string("base_footprint"));
    laser_frame_id_ = this->declare_parameter("laser_frame_id", std::string("front_laser_link"));
    pose_topic_     = this->declare_parameter("pose_topic", std::string("/icp_pose"));
    publish_pose_   = this->declare_parameter("publish_pose", true);
    publish_tf_     = this->declare_parameter("publish_tf", false);

    // ── YAML 自动发车位姿 ──
    std::vector<double> init_vec = this->declare_parameter(
        "initial_pose", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    use_yaml_initial_pose_ = this->declare_parameter("use_yaml_initial_pose", false);
    try {
        initial_pose_msg_.position.x = init_vec.at(0);
        initial_pose_msg_.position.y = init_vec.at(1);
        initial_pose_msg_.position.z = init_vec.at(2);
        tf2::Quaternion q;
        q.setRPY(init_vec.at(3), init_vec.at(4), init_vec.at(5));
        initial_pose_msg_.orientation.x = q.x();
        initial_pose_msg_.orientation.y = q.y();
        initial_pose_msg_.orientation.z = q.z();
        initial_pose_msg_.orientation.w = q.w();
        RCLCPP_INFO(this->get_logger(), "YAML initial pose: X=%.2f Y=%.2f Yaw=%.2f",
                     init_vec[0], init_vec[1], init_vec[5]);
    } catch (const std::out_of_range &ex) {
        RCLCPP_ERROR(this->get_logger(), "initial_pose invalid: %s", ex.what());
    }

    // ── TF ──
    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    auto timer_if = std::make_shared<tf2_ros::CreateTimerROS>(
        this->get_node_base_interface(), this->get_node_timers_interface());
    tf_buffer_->setCreateTimerInterface(timer_if);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    init_tf_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&PolarIcpNode::getStaticTf, this));

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
    }

    // ── 订阅 / 发布 ──
    std::string cloud_topic = this->declare_parameter("pointcloud_topic", std::string("/livox/lidar"));
    pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        cloud_topic, rclcpp::QoS(1),
        std::bind(&PolarIcpNode::pointcloudCallback, this, std::placeholders::_1));

    std::string rviz_topic = this->declare_parameter("rviz_initial_pose_topic", std::string("/initialpose"));
    initial_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        rviz_topic, rclcpp::QoS(1),
        [this](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
            initialPoseCallback(msg);
        });

    map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "/map_pointcloud", rclcpp::QoS(1).transient_local());

    if (publish_pose_) {
        reloc_pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            pose_topic_, rclcpp::QoS(10));
    }

    // ── 发布地图 ──
    sensor_msgs::msg::PointCloud2 map_msg;
    pcl::toROSMsg(*raw_map_, map_msg);
    map_msg.header.frame_id = map_frame_id_;
    map_msg.header.stamp = now();
    map_pub_->publish(map_msg);

    RCLCPP_INFO(this->get_logger(), "polar_icp_registration initialized | "
                "keyframes=%zu ring_bins=%d bev_z=[%.1f, %.1f] crop_radius=%.0f",
                keyframes_.size(), ring_bins_, bev_z_min_, bev_z_max_, bev_crop_radius_);
}

PolarIcpNode::~PolarIcpNode() {}

// ══════════════════════════════════════════════════════════════
//  静态 TF
// ══════════════════════════════════════════════════════════════

void PolarIcpNode::getStaticTf() {
    try {
        auto tf = tf_buffer_->lookupTransform(
            base_frame_id_, laser_frame_id_, tf2::TimePointZero);
        Eigen::Quaternionf q(tf.transform.rotation.w,
                             tf.transform.rotation.x,
                             tf.transform.rotation.y,
                             tf.transform.rotation.z);
        Eigen::Vector3f t(tf.transform.translation.x,
                          tf.transform.translation.y,
                          tf.transform.translation.z);
        base_to_sensor_T_ = Eigen::Matrix4f::Identity();
        base_to_sensor_T_.block<3, 3>(0, 0) = q.toRotationMatrix();
        base_to_sensor_T_.block<3, 1>(0, 3) = t;
        has_sensor_tf_ = true;
        RCLCPP_INFO(this->get_logger(), "Got static TF: %s -> %s",
                     base_frame_id_.c_str(), laser_frame_id_.c_str());
        init_tf_timer_->cancel();
    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                              "Waiting for static TF...");
    }
}

// ══════════════════════════════════════════════════════════════
//  极坐标环特征
// ══════════════════════════════════════════════════════════════

PolarRing PolarIcpNode::extractPolarRing(const CloudXYZI::Ptr &cloud) const {
    PolarRing ring;
    ring.num_bins = ring_bins_;
    ring.bin_angle = 2.0f * M_PI / ring_bins_;
    ring.max_rho.assign(ring_bins_, 0.0f);

    int bev_count = 0;
    for (const auto &pt : cloud->points) {
        if (pt.z < bev_z_min_ || pt.z > bev_z_max_) continue;
        bev_count++;
        float rho   = std::sqrt(pt.x * pt.x + pt.y * pt.y);
        float theta = std::atan2(pt.y, pt.x);
        if (theta < 0) theta += 2.0f * M_PI;
        int bin = static_cast<int>(theta / ring.bin_angle) % ring_bins_;
        if (rho > ring.max_rho[bin]) {
            ring.max_rho[bin] = rho;
        }
    }
    RCLCPP_DEBUG(this->get_logger(), "[Ring] total=%zu bev_pass=%d bins=%d",
                 cloud->size(), bev_count, ring_bins_);
    return ring;
}

float PolarIcpNode::computeYawOffset(const PolarRing &src, const PolarRing &tgt) const {
    float best_corr = -1.0f;
    int best_shift  = 0;

    for (int shift = 0; shift < ring_bins_; ++shift) {
        float corr = 0.0f;
        for (int i = 0; i < ring_bins_; ++i) {
            int j = (i + shift) % ring_bins_;
            corr += src.max_rho[i] * tgt.max_rho[j];
        }
        if (corr > best_corr) {
            best_corr = corr;
            best_shift = shift;
        }
    }

    float yaw = 2.0f * M_PI - (2.0f * M_PI * best_shift / ring_bins_);
    while (yaw >  M_PI) yaw -= 2.0f * M_PI;
    while (yaw < -M_PI) yaw += 2.0f * M_PI;
    RCLCPP_DEBUG(this->get_logger(), "[Yaw] best_shift=%d/%d corr=%.1f yaw=%.2f°",
                 best_shift, ring_bins_, best_corr, yaw * 180.0f / M_PI);
    return yaw;
}

Eigen::Vector2f PolarIcpNode::computeTranslation(const CloudXYZI::Ptr &src,
                                                  const CloudXYZI::Ptr &tgt,
                                                  float yaw_rad) const {
    CloudXYZI::Ptr rotated(new CloudXYZI);
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    T.block<3, 3>(0, 0) = Eigen::AngleAxisf(yaw_rad, Eigen::Vector3f::UnitZ()).toRotationMatrix();
    pcl::transformPointCloud(*src, *rotated, T);

    pcl::KdTreeFLANN<PointXYZI> kdtree;
    kdtree.setInputCloud(tgt);

    float dx = 0.0f, dy = 0.0f;
    for (int iter = 0; iter < 3; ++iter) {
        float sum_ex = 0.0f, sum_ey = 0.0f;
        int count = 0;

        for (const auto &pt : rotated->points) {
            PointXYZI query;
            query.x = pt.x + dx;
            query.y = pt.y + dy;
            query.z = pt.z;

            std::vector<int> idx(1);
            std::vector<float> dist(1);
            if (kdtree.nearestKSearch(query, 1, idx, dist) > 0) {
                if (dist[0] < 4.0f) {
                    sum_ex += tgt->points[idx[0]].x - query.x;
                    sum_ey += tgt->points[idx[0]].y - query.y;
                    count++;
                }
            }
        }
        if (count < 10) {
            RCLCPP_DEBUG(this->get_logger(), "[Trans] iter=%d count=%d < 10, break", iter, count);
            break;
        }
        float mean_ex = sum_ex / count;
        float mean_ey = sum_ey / count;
        dx += mean_ex;
        dy += mean_ey;
        RCLCPP_DEBUG(this->get_logger(), "[Trans] iter=%d count=%d mean=(%.3f, %.3f) cum=(%.3f, %.3f)",
                     iter, count, mean_ex, mean_ey, dx, dy);
    }
    RCLCPP_DEBUG(this->get_logger(), "[Trans] result=(%.3f, %.3f)", dx, dy);
    return Eigen::Vector2f(dx, dy);
}

// ══════════════════════════════════════════════════════════════
//  关键帧查找：找环特征最相似的关键帧
// ══════════════════════════════════════════════════════════════

int PolarIcpNode::findBestKeyframe(const PolarRing &src_ring) const {
    int best_idx = -1;
    float best_corr = -1.0f;

    // 预计算 source 环的模长
    float src_norm = 0.0f;
    for (int b = 0; b < ring_bins_; b++) {
        src_norm += src_ring.max_rho[b] * src_ring.max_rho[b];
    }
    src_norm = std::sqrt(src_norm) + 1e-6f;

    // 记录所有关键帧的相关值用于 debug
    std::vector<std::pair<float, int>> all_corrs;
    all_corrs.reserve(keyframes_.size());

    for (size_t i = 0; i < keyframes_.size(); i++) {
        const auto &kf_ring = keyframes_[i].ring;

        // 余弦相似度：消除绝对 ρ 值差异，只比较形状
        float dot = 0.0f, kf_norm = 0.0f;
        for (int b = 0; b < ring_bins_; b++) {
            dot     += src_ring.max_rho[b] * kf_ring.max_rho[b];
            kf_norm += kf_ring.max_rho[b] * kf_ring.max_rho[b];
        }
        float corr = dot / (src_norm * (std::sqrt(kf_norm) + 1e-6f));
        all_corrs.emplace_back(corr, static_cast<int>(i));

        if (corr > best_corr) {
            best_corr = corr;
            best_idx = static_cast<int>(i);
        }
    }

    // 打印 top-3 匹配结果
    std::sort(all_corrs.begin(), all_corrs.end(),
              [](const auto &a, const auto &b) { return a.first > b.first; });
    int top_n = std::min(3, static_cast<int>(all_corrs.size()));
    for (int k = 0; k < top_n; k++) {
        RCLCPP_DEBUG(this->get_logger(), "[KF] top%d: #%d corr=%.4f",
                     k + 1, all_corrs[k].second, all_corrs[k].first);
    }

    return best_idx;
}

// ══════════════════════════════════════════════════════════════
//  精配准（关键帧模式）
// ══════════════════════════════════════════════════════════════

Eigen::Matrix4d PolarIcpNode::multiResICP(const CloudXYZI::Ptr &source,
                                           const Eigen::Matrix4d &init_guess,
                                           const KeyframeData &kf) {
    Eigen::Matrix4f current_T = init_guess.cast<float>();

    // 用关键帧预计算法线的点云做精配准
    CloudXYZIN::Ptr tgt_norm = kf.cloud_with_normals;

    // 从粗到精逐层
    for (size_t level = 0; level < octree_resolutions_.size(); level++) {
        float res = static_cast<float>(octree_resolutions_[level]);

        CloudXYZI::Ptr src_down(new CloudXYZI);
        pcl::VoxelGrid<PointXYZI> vg;
        vg.setLeafSize(res, res, res);
        vg.setInputCloud(source);
        vg.filter(*src_down);

        if (src_down->size() < 10) continue;

        CloudXYZIN::Ptr src_norm = addNormals(src_down);

        pcl::IterativeClosestPointWithNormals<PointXYZIN, PointXYZIN> icp;
        icp.setMaximumIterations(refine_iter_);
        icp.setInputSource(src_norm);
        icp.setInputTarget(tgt_norm);
        icp.setMaxCorrespondenceDistance(res * 2.0f);

        CloudXYZIN::Ptr aligned(new CloudXYZIN);
        icp.align(*aligned, current_T);

        if (icp.hasConverged()) {
            current_T = icp.getFinalTransformation();
            RCLCPP_DEBUG(this->get_logger(), "[MultiResICP] level=%zu res=%.2f src=%zu converged, fitness=%.4f",
                         level, res, src_down->size(), icp.getFitnessScore());
        } else {
            RCLCPP_DEBUG(this->get_logger(), "[MultiResICP] level=%zu res=%.2f src=%zu NOT converged",
                         level, res, src_down->size());
        }
    }

    return current_T.cast<double>();
}

// ══════════════════════════════════════════════════════════════
//  主流程：粗配准 → 精配准
// ══════════════════════════════════════════════════════════════

Eigen::Matrix4d PolarIcpNode::coarseToFineAlign(CloudXYZI::Ptr source,
                                                 const Eigen::Matrix4d &init_guess) {
    success_ = false;
    auto tic = std::chrono::system_clock::now();

    Eigen::Vector3d xyz = init_guess.block<3, 1>(0, 3);
    Eigen::Matrix3d rot = init_guess.block<3, 3>(0, 0);
    Eigen::Vector3d rpy = rot.eulerAngles(2, 1, 0);
    double init_yaw = rpy[0];

    if (use_keyframes_) {
        // ── 关键帧模式（不依赖初始猜测，全局搜索） ──
        // 1. 先把 source 平移到原点附近（提取环特征需要同一参考点）
        //    用关键帧位姿作为参考，但此时还不知道哪个关键帧，所以先用粗略方式：
        //    遍历所有关键帧，找环特征最相似的
        //    注意：这里用 source 原始坐标提取环特征，keyframe 的环特征也是从各自中心提取
        //    两者的 ρ 值尺度不同，但相对分布相似，点积仍能找到最匹配的
        PolarRing src_ring = extractPolarRing(source);

        // 2. 找最佳匹配关键帧
        int best_kf_idx = findBestKeyframe(src_ring);
        if (best_kf_idx < 0) {
            RCLCPP_ERROR(this->get_logger(), "未找到匹配的关键帧");
            return Eigen::Matrix4d::Zero();
        }
        const auto &best_kf = keyframes_[best_kf_idx];
        RCLCPP_INFO(this->get_logger(), "最佳关键帧: #%d", best_kf_idx);

        // 3. 把 source 平移到关键帧位置，使环特征从同一参考点计算
        CloudXYZI::Ptr source_at_kf(new CloudXYZI);
        Eigen::Matrix4f T_to_kf = Eigen::Matrix4f::Identity();
        T_to_kf(0, 3) = static_cast<float>(best_kf.pose(0, 3));
        T_to_kf(1, 3) = static_cast<float>(best_kf.pose(1, 3));
        pcl::transformPointCloud(*source, *source_at_kf, T_to_kf);

        // 4. 在关键帧坐标系下重新计算环特征和 yaw 偏移
        PolarRing src_ring_at_kf = extractPolarRing(source_at_kf);
        float yaw_offset = computeYawOffset(src_ring_at_kf, best_kf.ring);

        // 从关键帧位姿提取 yaw
        Eigen::Matrix3d kf_rot = best_kf.pose.block<3, 3>(0, 0);
        Eigen::Vector3d kf_rpy = kf_rot.eulerAngles(2, 1, 0);
        float coarse_yaw = static_cast<float>(kf_rpy[0]) + yaw_offset;
        while (coarse_yaw >  M_PI) coarse_yaw -= 2.0f * M_PI;
        while (coarse_yaw < -M_PI) coarse_yaw += 2.0f * M_PI;

        // 5. 平移估计（在关键帧坐标系下，source_at_kf 和 best_kf.cloud 同坐标系）
        Eigen::Vector2f translation = computeTranslation(source_at_kf, best_kf.cloud, coarse_yaw);

        // 组装粗配准结果
        Eigen::Matrix4d T_coarse = Eigen::Matrix4d::Identity();
        T_coarse.block<3, 3>(0, 0) = Eigen::AngleAxisd(coarse_yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        T_coarse(0, 3) = best_kf.pose(0, 3) + translation[0];
        T_coarse(1, 3) = best_kf.pose(1, 3) + translation[1];
        T_coarse(2, 3) = best_kf.pose(2, 3);

        auto toc_coarse = std::chrono::system_clock::now();
        double coarse_ms = std::chrono::duration<double, std::milli>(toc_coarse - tic).count();
        RCLCPP_INFO(this->get_logger(),
                    "[Coarse] kf=#%d yaw_offset=%.1f° translation=(%.2f, %.2f) | %.0fms",
                    best_kf_idx, yaw_offset * 180.0 / M_PI,
                    translation[0], translation[1], coarse_ms);

        // 5. 精配准
        Eigen::Matrix4d T_final = multiResICP(source, T_coarse, best_kf);

        // 6. fitness 验证（无 drift 检查，因为没有可靠的初始猜测）
        CloudXYZIN::Ptr src_norm = addNormals(source);
        icp_refine_.setInputSource(src_norm);
        icp_refine_.setInputTarget(best_kf.cloud_with_normals);
        CloudXYZIN::Ptr aligned(new CloudXYZIN);
        icp_refine_.align(*aligned, T_final.cast<float>());

        if (!icp_refine_.hasConverged()) {
            RCLCPP_ERROR(this->get_logger(), "[Refine] failed: not converged");
            return Eigen::Matrix4d::Zero();
        }

        double fitness = icp_refine_.getFitnessScore();
        T_final = icp_refine_.getFinalTransformation().cast<double>();

        if (fitness > thresh_) {
            RCLCPP_ERROR(this->get_logger(), "[Refine] failed: fitness=%.3f > %.1f", fitness, thresh_);
            return Eigen::Matrix4d::Zero();
        }

        success_ = true;
        auto toc = std::chrono::system_clock::now();
        RCLCPP_INFO(this->get_logger(), "✅ KF reloc success! fitness=%.3f total=%.0fms",
                     fitness, std::chrono::duration<double, std::milli>(toc - tic).count());
        return T_final;

    } else {
        // ── 单地图模式（兼容无关键帧） ──
        CloudXYZI::Ptr source_in_map(new CloudXYZI);
        Eigen::Matrix4f T_shift = Eigen::Matrix4f::Identity();
        T_shift(0, 3) = static_cast<float>(xyz[0]);
        T_shift(1, 3) = static_cast<float>(xyz[1]);
        pcl::transformPointCloud(*source, *source_in_map, T_shift);

        float r2 = bev_crop_radius_ * bev_crop_radius_;
        CloudXYZI::Ptr map_cropped(new CloudXYZI);
        for (const auto &pt : raw_map_->points) {
            float ddx = pt.x - xyz[0];
            float ddy = pt.y - xyz[1];
            if (ddx * ddx + ddy * ddy < r2) {
                map_cropped->push_back(pt);
            }
        }

        PolarRing map_ring_local = extractPolarRing(map_cropped);
        PolarRing src_ring = extractPolarRing(source_in_map);

        float yaw_offset = computeYawOffset(src_ring, map_ring_local);
        float coarse_yaw = static_cast<float>(init_yaw) + yaw_offset;
        while (coarse_yaw >  M_PI) coarse_yaw -= 2.0f * M_PI;
        while (coarse_yaw < -M_PI) coarse_yaw += 2.0f * M_PI;

        Eigen::Vector2f translation = computeTranslation(source_in_map, map_cropped, coarse_yaw);
        float coarse_x = static_cast<float>(xyz[0]) + translation[0];
        float coarse_y = static_cast<float>(xyz[1]) + translation[1];

        Eigen::Matrix4d T_coarse = Eigen::Matrix4d::Identity();
        T_coarse.block<3, 3>(0, 0) = (Eigen::AngleAxisd(coarse_yaw, Eigen::Vector3d::UnitZ()) *
                                       Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY()) *
                                       Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitX())).toRotationMatrix();
        T_coarse(0, 3) = coarse_x;
        T_coarse(1, 3) = coarse_y;
        T_coarse(2, 3) = xyz[2];

        // 精配准（用全局地图）
        Eigen::Matrix4d T_final = T_coarse;
        for (size_t level = 0; level < octree_resolutions_.size(); level++) {
            float res = static_cast<float>(octree_resolutions_[level]);
            CloudXYZI::Ptr src_down(new CloudXYZI);
            pcl::VoxelGrid<PointXYZI> vg;
            vg.setLeafSize(res, res, res);
            vg.setInputCloud(source);
            vg.filter(*src_down);
            if (src_down->size() < 10) continue;

            CloudXYZIN::Ptr src_norm = addNormals(src_down);
            CloudXYZIN::Ptr tgt_norm = addNormals(map_cropped);

            pcl::IterativeClosestPointWithNormals<PointXYZIN, PointXYZIN> icp;
            icp.setMaximumIterations(refine_iter_);
            icp.setInputSource(src_norm);
            icp.setInputTarget(tgt_norm);
            icp.setMaxCorrespondenceDistance(res * 2.0f);

            CloudXYZIN::Ptr aligned(new CloudXYZIN);
            icp.align(*aligned, T_final.cast<float>());
            if (icp.hasConverged()) T_final = icp.getFinalTransformation().cast<double>();
        }

        // fitness 验证
        CloudXYZIN::Ptr src_norm = addNormals(source);
        icp_refine_.setInputSource(src_norm);
        icp_refine_.setInputTarget(refine_map_);
        CloudXYZIN::Ptr aligned(new CloudXYZIN);
        icp_refine_.align(*aligned, T_final.cast<float>());

        if (!icp_refine_.hasConverged()) {
            RCLCPP_ERROR(this->get_logger(), "[Refine] failed");
            return Eigen::Matrix4d::Zero();
        }

        double fitness = icp_refine_.getFitnessScore();
        T_final = icp_refine_.getFinalTransformation().cast<double>();

        if (fitness > thresh_) {
            RCLCPP_ERROR(this->get_logger(), "[Refine] fitness=%.3f > %.1f", fitness, thresh_);
            return Eigen::Matrix4d::Zero();
        }

        double dx = T_final(0, 3) - xyz[0];
        double dy = T_final(1, 3) - xyz[1];
        double drift = std::sqrt(dx * dx + dy * dy);
        if (drift > drift_threshold_) {
            RCLCPP_WARN(this->get_logger(), "[Drift] %.2fm > %.2fm", drift, drift_threshold_);
            return Eigen::Matrix4d::Zero();
        }

        success_ = true;
        auto toc = std::chrono::system_clock::now();
        RCLCPP_INFO(this->get_logger(), "✅ Map reloc success! fitness=%.3f total=%.0fms",
                     fitness, std::chrono::duration<double, std::milli>(toc - tic).count());
        return T_final;
    }
}

// ══════════════════════════════════════════════════════════════
//  点云回调
// ══════════════════════════════════════════════════════════════

void PolarIcpNode::pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    if (!has_sensor_tf_) return;

    CloudXYZI::Ptr raw(new CloudXYZI);
    pcl::fromROSMsg(*msg, *raw);

    CloudXYZI::Ptr transformed(new CloudXYZI);
    pcl::transformPointCloud(*raw, *transformed, base_to_sensor_T_);
    RCLCPP_DEBUG(this->get_logger(), "[Cloud] raw=%zu transformed=%zu", raw->size(), transformed->size());
    {
        std::lock_guard<std::mutex> lock(cloud_mutex_);
        cloud_in_ = transformed;
        has_cloud_ = true;
    }

    if (use_yaml_initial_pose_) {
        RCLCPP_INFO_ONCE(this->get_logger(), "Auto-relocating from YAML pose...");

        Eigen::Vector3d pos(initial_pose_msg_.position.x,
                            initial_pose_msg_.position.y,
                            initial_pose_msg_.position.z);
        Eigen::Quaterniond q(initial_pose_msg_.orientation.w,
                             initial_pose_msg_.orientation.x,
                             initial_pose_msg_.orientation.y,
                             initial_pose_msg_.orientation.z);
        Eigen::Matrix4d guess = Eigen::Matrix4d::Identity();
        guess.block<3, 3>(0, 0) = q.toRotationMatrix();
        guess.block<3, 1>(0, 3) = pos;

        Eigen::Matrix4d result = coarseToFineAlign(transformed, guess);

        if (success_) {
            geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
            pose_msg.header.stamp = now();
            pose_msg.header.frame_id = map_frame_id_;
            pose_msg.pose.pose.position.x = result(0, 3);
            pose_msg.pose.pose.position.y = result(1, 3);
            pose_msg.pose.pose.position.z = result(2, 3);
            Eigen::Quaterniond q_res(result.block<3, 3>(0, 0));
            pose_msg.pose.pose.orientation = eigenQuatToMsg(q_res);

            if (publish_pose_) reloc_pose_pub_->publish(pose_msg);

            if (publish_tf_) {
                try {
                    auto odom_tf = tf_buffer_->lookupTransform(
                        odom_frame_id_, base_frame_id_, tf2::TimePointZero);
                    Eigen::Matrix4d odom_to_base = Eigen::Matrix4d::Identity();
                    odom_to_base.block<3, 3>(0, 0) = Eigen::Quaterniond(
                        odom_tf.transform.rotation.w, odom_tf.transform.rotation.x,
                        odom_tf.transform.rotation.y, odom_tf.transform.rotation.z).toRotationMatrix();
                    odom_to_base(0, 3) = odom_tf.transform.translation.x;
                    odom_to_base(1, 3) = odom_tf.transform.translation.y;
                    odom_to_base(2, 3) = odom_tf.transform.translation.z;

                    Eigen::Matrix4d map_to_odom = result * odom_to_base.inverse();
                    Eigen::Quaterniond q_mo(map_to_odom.block<3, 3>(0, 0));
                    map_to_odom_tf_.header.frame_id = map_frame_id_;
                    map_to_odom_tf_.child_frame_id = odom_frame_id_;
                    map_to_odom_tf_.transform.translation.x = map_to_odom(0, 3);
                    map_to_odom_tf_.transform.translation.y = map_to_odom(1, 3);
                    map_to_odom_tf_.transform.translation.z = map_to_odom(2, 3);
                    map_to_odom_tf_.transform.rotation = eigenQuatToMsg(q_mo);
                    has_calculated_pose_ = true;
                } catch (tf2::TransformException &ex) {
                    RCLCPP_ERROR(this->get_logger(), "TF failed: %s", ex.what());
                }
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "⚠️ Auto-reloc failed");
        }
        use_yaml_initial_pose_ = false;
    }
}

// ══════════════════════════════════════════════════════════════
//  初始位姿回调
// ══════════════════════════════════════════════════════════════

void PolarIcpNode::initialPoseCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    // 拷贝点云引用，避免长时间持锁
    CloudXYZI::Ptr cloud_copy;
    {
        std::lock_guard<std::mutex> lock(cloud_mutex_);
        if (!has_cloud_) {
            RCLCPP_WARN(this->get_logger(), "No pointcloud yet!");
            return;
        }
        cloud_copy = cloud_in_;
    }

    RCLCPP_INFO(this->get_logger(), "Received initial guess, starting polar ICP...");

    Eigen::Vector3d pos(msg->pose.pose.position.x,
                        msg->pose.pose.position.y,
                        msg->pose.pose.position.z);
    Eigen::Quaterniond q(msg->pose.pose.orientation.w,
                         msg->pose.pose.orientation.x,
                         msg->pose.pose.orientation.y,
                         msg->pose.pose.orientation.z);
    Eigen::Matrix4d guess = Eigen::Matrix4d::Identity();
    guess.block<3, 3>(0, 0) = q.toRotationMatrix();
    guess.block<3, 1>(0, 3) = pos;

    Eigen::Matrix4d result = coarseToFineAlign(cloud_copy, guess);

    if (!success_) {
        RCLCPP_ERROR(this->get_logger(), "❌ Relocalization failed!");
        return;
    }

    geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
    pose_msg.header.stamp = now();
    pose_msg.header.frame_id = map_frame_id_;
    pose_msg.pose.pose.position.x = result(0, 3);
    pose_msg.pose.pose.position.y = result(1, 3);
    pose_msg.pose.pose.position.z = result(2, 3);
    Eigen::Quaterniond q_res(result.block<3, 3>(0, 0));
    pose_msg.pose.pose.orientation = eigenQuatToMsg(q_res);

    reloc_pose_pub_->publish(pose_msg);
    RCLCPP_INFO(this->get_logger(), "🎯 Reloc done! [%.2f, %.2f, %.2f]",
                result(0, 3), result(1, 3), result(2, 3));

    if (publish_tf_) {
        try {
            auto odom_tf = tf_buffer_->lookupTransform(
                odom_frame_id_, base_frame_id_, tf2::TimePointZero);
            Eigen::Matrix4d odom_to_base = Eigen::Matrix4d::Identity();
            odom_to_base.block<3, 3>(0, 0) = Eigen::Quaterniond(
                odom_tf.transform.rotation.w, odom_tf.transform.rotation.x,
                odom_tf.transform.rotation.y, odom_tf.transform.rotation.z).toRotationMatrix();
            odom_to_base(0, 3) = odom_tf.transform.translation.x;
            odom_to_base(1, 3) = odom_tf.transform.translation.y;
            odom_to_base(2, 3) = odom_tf.transform.translation.z;

            Eigen::Matrix4d map_to_odom = result * odom_to_base.inverse();
            Eigen::Quaterniond q_mo(map_to_odom.block<3, 3>(0, 0));
            map_to_odom_tf_.header.frame_id = map_frame_id_;
            map_to_odom_tf_.child_frame_id = odom_frame_id_;
            map_to_odom_tf_.transform.translation.x = map_to_odom(0, 3);
            map_to_odom_tf_.transform.translation.y = map_to_odom(1, 3);
            map_to_odom_tf_.transform.translation.z = map_to_odom(2, 3);
            map_to_odom_tf_.transform.rotation = eigenQuatToMsg(q_mo);
            has_calculated_pose_ = true;
        } catch (tf2::TransformException &ex) {
            RCLCPP_ERROR(this->get_logger(), "TF failed: %s", ex.what());
        }
    }
}

// ══════════════════════════════════════════════════════════════
//  法线计算
// ══════════════════════════════════════════════════════════════

CloudXYZIN::Ptr PolarIcpNode::addNormals(const CloudXYZI::Ptr &cloud, int k) {
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<PointXYZI>::Ptr tree(new pcl::search::KdTree<PointXYZI>);
    tree->setInputCloud(cloud);

    pcl::NormalEstimation<PointXYZI, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    ne.setSearchMethod(tree);
    ne.setKSearch(k);
    ne.compute(*normals);

    CloudXYZIN::Ptr out(new CloudXYZIN);
    pcl::concatenateFields(*cloud, *normals, *out);
    return out;
}

}  // namespace polar_icp

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(polar_icp::PolarIcpNode)
