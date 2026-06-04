#include "dual_descriptor_relocalization/dual_descriptor_node.hpp"

#include <filesystem>
#include <algorithm>
#include <numeric>
#include <cmath>

namespace dual_descriptor_relocalization {

// ══════════════════════════════════════════════════════════════
//  构造/析构
// ══════════════════════════════════════════════════════════════

DualDescriptorNode::DualDescriptorNode(const rclcpp::NodeOptions& options)
    : Node("dual_descriptor_node", options)
{
    loadParameters();

    // TF
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // 定时获取静态 TF
    auto init_tf_timer = this->create_wall_timer(
        std::chrono::milliseconds(500),
        [this, init_tf_timer]() {
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
                init_tf_timer->cancel();
            } catch (tf2::TransformException& ex) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                     "Waiting for static TF: %s", ex.what());
            }
        });

    // 加载关键帧数据库
    loadKeyframeDatabase();
    buildKDTrees();

    // 发布/订阅/服务
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        pose_topic_, rclcpp::QoS(1));

    // 服务：供老节点触发重新定位
    trigger_service_ = this->create_service<std_srvs::srv::Trigger>(
        trigger_service_name_,
        std::bind(&DualDescriptorNode::triggerCallback, this, std::placeholders::_1, std::placeholders::_2));

    // 启动时默认休眠，不订阅点云
    sleeping_ = true;

    RCLCPP_INFO(this->get_logger(), "DualDescriptorNode initialized (sleeping) | "
                "keyframes=%zu polar_bins=%d sc=[%dx%d] candidates=%d trigger_service=%s",
                keyframes_.size(), polar_bins_, sc_num_ring_, sc_num_sector_, num_candidates_,
                trigger_service_name_.c_str());
}

DualDescriptorNode::~DualDescriptorNode() {}

// ══════════════════════════════════════════════════════════════
//  参数加载
// ══════════════════════════════════════════════════════════════

void DualDescriptorNode::loadParameters() {
    // 关键帧数据库
    keyframe_db_path_ = this->declare_parameter("keyframe_db", std::string(""));

    // 话题
    pointcloud_topic_ = this->declare_parameter("pointcloud_topic", std::string("/livox/lidar"));
    pose_topic_ = this->declare_parameter("pose_topic", std::string("/relocalization_pose"));

    // 坐标系
    map_frame_id_ = this->declare_parameter("map_frame_id", std::string("map"));
    base_frame_id_ = this->declare_parameter("base_frame_id", std::string("base_footprint"));
    laser_frame_id_ = this->declare_parameter("laser_frame_id", std::string("front_laser_link"));

    // 极坐标环参数
    polar_bins_ = this->declare_parameter("polar_bins", 360);
    polar_bev_z_min_ = this->declare_parameter("polar_bev_z_min", 0.2);
    polar_bev_z_max_ = this->declare_parameter("polar_bev_z_max", 2.0);

    // SC 矩阵参数
    sc_num_ring_ = this->declare_parameter("sc_num_ring", 20);
    sc_num_sector_ = this->declare_parameter("sc_num_sector", 60);
    sc_max_radius_ = this->declare_parameter("sc_max_radius", 80.0f);
    sc_bev_z_min_ = this->declare_parameter("sc_bev_z_min", 0.1f);

    // 搜索参数
    num_candidates_ = this->declare_parameter("num_candidates", 3);

    // 服务参数
    trigger_service_name_ = this->declare_parameter("trigger_service_name", "/relocalization_trigger");

    // ICP 参数
    icp_resolutions_ = this->declare_parameter("icp_resolutions", std::vector<double>{1.0, 0.5, 0.2});
    icp_iterations_ = this->declare_parameter("icp_iterations", 5);
    icp_max_corr_dist_ = this->declare_parameter("icp_max_corr_dist", 1.0f);
    icp_fitness_threshold_ = this->declare_parameter("icp_fitness_threshold", 0.5f);

    RCLCPP_DEBUG(this->get_logger(), "Parameters loaded");
}

// ══════════════════════════════════════════════════════════════
//  关键帧数据库加载
// ══════════════════════════════════════════════════════════════

void DualDescriptorNode::loadKeyframeDatabase() {
    if (keyframe_db_path_.empty()) {
        RCLCPP_ERROR(this->get_logger(), "keyframe_db path is empty!");
        return;
    }

    namespace fs = std::filesystem;
    std::string bin_path = keyframe_db_path_ + "/keyframes.bin";
    std::string clouds_dir = keyframe_db_path_ + "/clouds";

    if (!fs::exists(bin_path) || !fs::is_directory(clouds_dir)) {
        RCLCPP_ERROR(this->get_logger(), "Keyframe database not found: %s", keyframe_db_path_.c_str());
        return;
    }

    // 读取 keyframes.bin
    std::ifstream f(bin_path, std::ios::binary);
    if (!f.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Cannot open %s", bin_path.c_str());
        return;
    }

    int num_keyframes = 0;
    f.read(reinterpret_cast<char*>(&num_keyframes), sizeof(int));
    RCLCPP_INFO(this->get_logger(), "Loading %d keyframes...", num_keyframes);

    keyframes_.resize(num_keyframes);
    for (int i = 0; i < num_keyframes; i++) {
        auto& kf = keyframes_[i];
        kf.id = i;

        // 读取位姿 (6DoF: x, y, z, roll, pitch, yaw)
        float pose_data[6];
        f.read(reinterpret_cast<char*>(pose_data), 6 * sizeof(float));
        Eigen::AngleAxisf roll(Eigen::AngleAxisf(pose_data[3], Eigen::Vector3f::UnitX()));
        Eigen::AngleAxisf pitch(Eigen::AngleAxisf(pose_data[4], Eigen::Vector3f::UnitY()));
        Eigen::AngleAxisf yaw(Eigen::AngleAxisf(pose_data[5], Eigen::Vector3f::UnitZ()));
        kf.pose = Eigen::Matrix4f::Identity();
        kf.pose.block<3, 3>(0, 0) = (yaw * pitch * roll).toRotationMatrix();
        kf.pose(0, 3) = pose_data[0];
        kf.pose(1, 3) = pose_data[1];
        kf.pose(2, 3) = pose_data[2];

        // 读取极坐标环特征（预先计算的）
        kf.polar_ring.num_bins = polar_bins_;
        kf.polar_ring.bin_angle = 2.0f * M_PI / polar_bins_;
        kf.polar_ring.max_rho.resize(polar_bins_);
        f.read(reinterpret_cast<char*>(kf.polar_ring.max_rho.data()), polar_bins_ * sizeof(float));

        // 加载点云
        std::string pcd_path = clouds_dir + "/" + std::to_string(i) + ".pcd";
        kf.cloud.reset(new CloudT);
        if (pcl::io::loadPCDFile(pcd_path, *kf.cloud) < 0) {
            RCLCPP_WARN(this->get_logger(), "Cannot load keyframe cloud: %s", pcd_path.c_str());
            continue;
        }

        // 提取 SC 矩阵
        kf.sc_matrix = extractSCMatrix(kf.cloud);

        // 提取 Ring Key
        kf.ring_key = extractRingKey(kf.sc_matrix);
    }

    f.close();
    RCLCPP_INFO(this->get_logger(), "Loaded %zu keyframes", keyframes_.size());
}

// ══════════════════════════════════════════════════════════════
//  构建 KD-Tree
// ══════════════════════════════════════════════════════════════

void DualDescriptorNode::buildKDTrees() {
    if (keyframes_.empty()) return;

    // 构建极坐标环特征的 KD-Tree
    // 将环特征转换为点云格式（每个关键帧是一个点，坐标是环特征值）
    polar_features_cloud_.reset(new CloudT);
    polar_features_cloud_->resize(keyframes_.size());

    for (size_t i = 0; i < keyframes_.size(); i++) {
        // 用环特征的前 3 个维度作为坐标（归一化）
        float sum = 0.0f;
        for (float v : keyframes_[i].polar_ring.max_rho) sum += v;
        float mean = sum / polar_bins_;

        // 归一化
        float norm = 0.0f;
        for (float v : keyframes_[i].polar_ring.max_rho) {
            norm += (v - mean) * (v - mean);
        }
        norm = std::sqrt(norm) + 1e-6f;

        // 存储归一化后的环特征（用前 3 个 bin 作为坐标，实际搜索时用自定义距离）
        polar_features_cloud_->points[i].x = keyframes_[i].polar_ring.max_rho[0] / norm;
        polar_features_cloud_->points[i].y = keyframes_[i].polar_ring.max_rho[1] / norm;
        polar_features_cloud_->points[i].z = keyframes_[i].polar_ring.max_rho[2] / norm;
        polar_features_cloud_->points[i].intensity = static_cast<float>(i);
    }
    polar_kdtree_.setInputCloud(polar_features_cloud_);

    // 构建 SC Ring Key 的 KD-Tree
    sc_features_cloud_.reset(new CloudT);
    sc_features_cloud_->resize(keyframes_.size());

    for (size_t i = 0; i < keyframes_.size(); i++) {
        // 用 Ring Key 的前 3 个维度作为坐标
        sc_features_cloud_->points[i].x = keyframes_[i].ring_key(0);
        sc_features_cloud_->points[i].y = keyframes_[i].ring_key(1);
        sc_features_cloud_->points[i].z = keyframes_[i].ring_key(2);
        sc_features_cloud_->points[i].intensity = static_cast<float>(i);
    }
    sc_kdtree_.setInputCloud(sc_features_cloud_);

    RCLCPP_DEBUG(this->get_logger(), "KD-Trees built: polar=%zu, sc=%zu",
                 polar_features_cloud_->size(), sc_features_cloud_->size());
}

// ══════════════════════════════════════════════════════════════
//  极坐标环特征提取
// ══════════════════════════════════════════════════════════════

PolarRing DualDescriptorNode::extractPolarRing(const CloudT::Ptr& cloud) const {
    PolarRing ring;
    ring.num_bins = polar_bins_;
    ring.bin_angle = 2.0f * M_PI / polar_bins_;
    ring.max_rho.assign(polar_bins_, 0.0f);

    for (const auto& pt : cloud->points) {
        // BEV 投影：过滤地面和天花板
        if (pt.z < polar_bev_z_min_ || pt.z > polar_bev_z_max_) continue;

        float rho = std::sqrt(pt.x * pt.x + pt.y * pt.y);
        float theta = std::atan2(pt.y, pt.x);
        if (theta < 0) theta += 2.0f * M_PI;

        int bin = static_cast<int>(theta / ring.bin_angle) % polar_bins_;
        if (rho > ring.max_rho[bin]) {
            ring.max_rho[bin] = rho;
        }
    }

    return ring;
}

// ══════════════════════════════════════════════════════════════
//  SC 矩阵特征提取
// ══════════════════════════════════════════════════════════════

SCMatrix DualDescriptorNode::extractSCMatrix(const CloudT::Ptr& cloud) const {
    SCMatrix sc;
    sc.num_ring = sc_num_ring_;
    sc.num_sector = sc_num_sector_;
    sc.max_radius = sc_max_radius_;
    sc.data = Eigen::MatrixXf::Constant(sc_num_ring_, sc_num_sector_, -1000.0f);

    for (const auto& pt : cloud->points) {
        // 只过滤地面，不过滤天花板（保留高度信息）
        if (pt.z < sc_bev_z_min_) continue;

        float rho = std::sqrt(pt.x * pt.x + pt.y * pt.y);
        if (rho > sc_max_radius_) continue;

        float theta = std::atan2(pt.y, pt.x);
        float theta_deg = (theta * 180.0f / M_PI + 180.0f);  // [0, 360)

        int ring_idx = std::min(sc_num_ring_ - 1, static_cast<int>(rho / sc_max_radius_ * sc_num_ring_));
        int sector_idx = std::min(sc_num_sector_ - 1, static_cast<int>(theta_deg / 360.0f * sc_num_sector_));

        if (pt.z > sc.data(ring_idx, sector_idx)) {
            sc.data(ring_idx, sector_idx) = pt.z;  // 保留最大高度
        }
    }

    // 将 -1000 替换为 0
    sc.data = (sc.data.array() < -999).select(0.0f, sc.data);

    return sc;
}

// ══════════════════════════════════════════════════════════════
//  SC Ring Key 提取
// ══════════════════════════════════════════════════════════════

Eigen::VectorXf DualDescriptorNode::extractRingKey(const SCMatrix& sc) const {
    // 对 SC 矩阵的每一行取平均值
    Eigen::VectorXf ring_key(sc_num_ring_);
    for (int i = 0; i < sc_num_ring_; i++) {
        ring_key(i) = sc.data.row(i).mean();
    }
    return ring_key;
}

// ══════════════════════════════════════════════════════════════
//  搜索候选
// ══════════════════════════════════════════════════════════════

std::vector<int> DualDescriptorNode::searchPolarCandidates(const PolarRing& query, int top_k) const {
    // 计算查询特征的归一化
    float sum = 0.0f;
    for (float v : query.max_rho) sum += v;
    float mean = sum / polar_bins_;

    float norm = 0.0f;
    for (float v : query.max_rho) {
        norm += (v - mean) * (v - mean);
    }
    norm = std::sqrt(norm) + 1e-6f;

    PointT query_point;
    query_point.x = query.max_rho[0] / norm;
    query_point.y = query.max_rho[1] / norm;
    query_point.z = query.max_rho[2] / norm;

    // KD-Tree 搜索
    std::vector<int> indices(top_k);
    std::vector<float> dists(top_k);
    int found = polar_kdtree_.nearestKSearch(query_point, top_k, indices, dists);

    std::vector<int> result;
    for (int i = 0; i < found; i++) {
        int kf_id = static_cast<int>(polar_features_cloud_->points[indices[i]].intensity);
        result.push_back(kf_id);
    }

    RCLCPP_DEBUG(this->get_logger(), "Polar search: found %d candidates", found);
    return result;
}

std::vector<int> DualDescriptorNode::searchSCCandidates(const Eigen::VectorXf& query, int top_k) const {
    PointT query_point;
    query_point.x = query(0);
    query_point.y = query(1);
    query_point.z = query(2);

    // KD-Tree 搜索
    std::vector<int> indices(top_k);
    std::vector<float> dists(top_k);
    int found = sc_kdtree_.nearestKSearch(query_point, top_k, indices, dists);

    std::vector<int> result;
    for (int i = 0; i < found; i++) {
        int kf_id = static_cast<int>(sc_features_cloud_->points[indices[i]].intensity);
        result.push_back(kf_id);
    }

    RCLCPP_DEBUG(this->get_logger(), "SC search: found %d candidates", found);
    return result;
}

// ══════════════════════════════════════════════════════════════
//  投票机制
// ══════════════════════════════════════════════════════════════

std::vector<Candidate> DualDescriptorNode::voteCandidates(
    const std::vector<int>& polar_candidates,
    const std::vector<int>& sc_candidates) const
{
    // 统计每个候选的票数
    std::map<int, Candidate> candidate_map;

    // 极坐标环候选
    for (int id : polar_candidates) {
        candidate_map[id].keyframe_id = id;
        candidate_map[id].votes += 1;
    }

    // SC 候选
    for (int id : sc_candidates) {
        candidate_map[id].keyframe_id = id;
        candidate_map[id].votes += 1;
    }

    // 转换为 vector
    std::vector<Candidate> candidates;
    for (auto& [id, c] : candidate_map) {
        candidates.push_back(c);
    }

    // 按票数降序排序
    std::sort(candidates.begin(), candidates.end(),
              [](const Candidate& a, const Candidate& b) { return a.votes > b.votes; });

    RCLCPP_DEBUG(this->get_logger(), "Voting: %zu unique candidates, top votes=%d",
                 candidates.size(), candidates.empty() ? 0 : candidates[0].votes);

    return candidates;
}

// ══════════════════════════════════════════════════════════════
//  ICP 精配准
// ══════════════════════════════════════════════════════════════

float DualDescriptorNode::icpRefine(const CloudT::Ptr& source, const CloudT::Ptr& target,
                                     Eigen::Matrix4f& result) const {
    if (source->empty() || target->empty()) return 1e6f;

    // 多分辨率 ICP
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();

    for (size_t res_idx = 0; res_idx < icp_resolutions_.size(); res_idx++) {
        float resolution = icp_resolutions_[res_idx];

        // 体素降采样
        pcl::VoxelGrid<PointT> voxel;
        voxel.setLeafSize(resolution, resolution, resolution);

        CloudT::Ptr src_filtered(new CloudT);
        CloudT::Ptr tgt_filtered(new CloudT);
        voxel.setInputCloud(source);
        voxel.filter(*src_filtered);
        voxel.setInputCloud(target);
        voxel.filter(*tgt_filtered);

        if (src_filtered->empty() || tgt_filtered->empty()) continue;

        // ICP
        pcl::IterativeClosestPoint<PointT, PointT> icp;
        icp.setInputSource(src_filtered);
        icp.setInputTarget(tgt_filtered);
        icp.setMaxCorrespondenceDistance(resolution * 2.0f);
        icp.setMaximumIterations(icp_iterations_);
        icp.setTransformationEpsilon(1e-6);
        icp.setEuclideanFitnessEpsilon(1e-6);

        CloudT aligned;
        icp.align(aligned, T);

        if (icp.hasConverged()) {
            T = icp.getFinalTransformation();
            RCLCPP_DEBUG(this->get_logger(), "ICP res=%.2f converged, fitness=%.4f",
                         resolution, icp.getFitnessScore());
        } else {
            RCLCPP_DEBUG(this->get_logger(), "ICP res=%.2f NOT converged", resolution);
        }
    }

    result = T;
    float fitness = 0.0f;

    // 计算最终 fitness
    CloudT::Ptr aligned_final(new CloudT);
    pcl::transformPointCloud(*source, *aligned_final, T);

    pcl::KdTreeFLANN<PointT> kdtree;
    kdtree.setInputCloud(target);

    float sum_dist = 0.0f;
    int count = 0;
    for (const auto& pt : aligned_final->points) {
        std::vector<int> idx(1);
        std::vector<float> dist(1);
        if (kdtree.nearestKSearch(pt, 1, idx, dist) > 0) {
            sum_dist += dist[0];
            count++;
        }
    }
    fitness = (count > 0) ? sum_dist / count : 1e6f;

    return fitness;
}

// ══════════════════════════════════════════════════════════════
//  主流程：点云回调
// ══════════════════════════════════════════════════════════════

void DualDescriptorNode::pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    // 休眠状态下不处理点云
    if (sleeping_) {
        return;
    }

    if (!has_sensor_tf_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "No sensor TF yet");
        return;
    }

    if (keyframes_.empty()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "No keyframes loaded");
        return;
    }

    // 转换点云
    CloudT::Ptr raw(new CloudT);
    pcl::fromROSMsg(*msg, *raw);

    CloudT::Ptr source(new CloudT);
    pcl::transformPointCloud(*raw, *source, base_to_sensor_T_);

    RCLCPP_DEBUG(this->get_logger(), "Pointcloud received: %zu points", source->size());

    // 提取双描述子
    PolarRing src_polar = extractPolarRing(source);
    SCMatrix src_sc = extractSCMatrix(source);
    Eigen::VectorXf src_ring_key = extractRingKey(src_sc);

    // 用两个描述子分别搜索候选
    auto polar_candidates = searchPolarCandidates(src_polar, num_candidates_);
    auto sc_candidates = searchSCCandidates(src_ring_key, num_candidates_);

    // 投票机制融合
    auto candidates = voteCandidates(polar_candidates, sc_candidates);

    if (candidates.empty()) {
        RCLCPP_WARN(this->get_logger(), "No candidates found");
        return;
    }

    // 对每个候选做 ICP 验证
    float best_fitness = 1e6f;
    Eigen::Matrix4f best_pose = Eigen::Matrix4f::Identity();
    int best_kf_id = -1;

    for (auto& c : candidates) {
        int kf_id = c.keyframe_id;
        if (kf_id < 0 || kf_id >= static_cast<int>(keyframes_.size())) continue;

        const auto& kf = keyframes_[kf_id];
        if (!kf.cloud || kf.cloud->empty()) continue;

        // 计算相似度（用于日志）
        c.polar_sim = cosineSimilarity(src_polar.max_rho, kf.polar_ring.max_rho);
        c.sc_sim = cosineSimilarity(src_ring_key, kf.ring_key);

        // ICP 精配准
        Eigen::Matrix4f icp_pose;
        float fitness = icpRefine(source, kf.cloud, icp_pose);
        c.icp_fitness = fitness;

        RCLCPP_DEBUG(this->get_logger(), "Candidate %d: votes=%d polar_sim=%.3f sc_sim=%.3f fitness=%.4f",
                     kf_id, c.votes, c.polar_sim, c.sc_sim, fitness);

        if (fitness < best_fitness) {
            best_fitness = fitness;
            best_pose = kf.pose * icp_pose;  // 关键帧位姿 × ICP 修正
            best_kf_id = kf_id;
        }
    }

    // 检查 fitness 阈值
    if (best_fitness > icp_fitness_threshold_) {
        RCLCPP_WARN(this->get_logger(), "Relocalization failed: best_fitness=%.4f > threshold=%.4f",
                    best_fitness, icp_fitness_threshold_);
        return;
    }

    // 输出位姿
    RCLCPP_INFO(this->get_logger(), "Relocalization success: kf=%d fitness=%.4f",
                best_kf_id, best_fitness);
    publishPose(best_pose, 1.0f / (best_fitness + 1e-6f));

    // 定位成功，进入休眠
    enterSleep();
}

// ══════════════════════════════════════════════════════════════
//  发布位姿
// ══════════════════════════════════════════════════════════════

void DualDescriptorNode::publishPose(const Eigen::Matrix4f& pose, float confidence) {
    geometry_msgs::msg::PoseWithCovarianceStamped msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = map_frame_id_;

    // 位置
    msg.pose.pose.position.x = pose(0, 3);
    msg.pose.pose.position.y = pose(1, 3);
    msg.pose.pose.position.z = pose(2, 3);

    // 姿态（旋转矩阵 → 四元数）
    Eigen::Matrix3f R = pose.block<3, 3>(0, 0);
    Eigen::Quaternionf q(R);
    msg.pose.pose.orientation.x = q.x();
    msg.pose.pose.orientation.y = q.y();
    msg.pose.pose.orientation.z = q.z();
    msg.pose.pose.orientation.w = q.w();

    // 协方差（用 confidence 缩放）
    float cov_scale = 1.0f / (confidence + 1e-6f);
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            if (i == j) {
                msg.pose.covariance[i * 6 + j] = cov_scale * 0.01f;
            } else {
                msg.pose.covariance[i * 6 + j] = 0.0f;
            }
        }
    }

    pose_pub_->publish(msg);
    RCLCPP_DEBUG(this->get_logger(), "Pose published: (%.2f, %.2f, %.2f) confidence=%.2f",
                 pose(0, 3), pose(1, 3), pose(2, 3), confidence);
}

// ══════════════════════════════════════════════════════════════
//  休眠/唤醒
// ══════════════════════════════════════════════════════════════

void DualDescriptorNode::enterSleep() {
    sleeping_ = true;
    // 停止订阅点云，释放资源
    pointcloud_sub_.reset();
    RCLCPP_INFO(this->get_logger(), "Entering sleep mode, waiting for trigger...");
}

void DualDescriptorNode::wakeUp() {
    sleeping_ = false;
    // 重新订阅点云
    pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        pointcloud_topic_, rclcpp::QoS(1),
        std::bind(&DualDescriptorNode::pointcloudCallback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Woke up, subscribing to pointcloud...");
}

void DualDescriptorNode::triggerCallback(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response)
{
    (void)request;  // 未使用

    if (!sleeping_) {
        response->success = false;
        response->message = "Already awake, relocalization in progress";
        RCLCPP_WARN(this->get_logger(), "Trigger ignored: already awake");
        return;
    }

    // 唤醒
    wakeUp();

    response->success = true;
    response->message = "Relocalization triggered";
    RCLCPP_INFO(this->get_logger(), "Trigger received, waking up");
}

// ══════════════════════════════════════════════════════════════
//  工具函数
// ══════════════════════════════════════════════════════════════

float DualDescriptorNode::cosineSimilarity(const std::vector<float>& a, const std::vector<float>& b) const {
    if (a.size() != b.size() || a.empty()) return 0.0f;

    float dot = 0.0f, norm_a = 0.0f, norm_b = 0.0f;
    for (size_t i = 0; i < a.size(); i++) {
        dot += a[i] * b[i];
        norm_a += a[i] * a[i];
        norm_b += b[i] * b[i];
    }
    return dot / (std::sqrt(norm_a) * std::sqrt(norm_b) + 1e-6f);
}

float DualDescriptorNode::cosineSimilarity(const Eigen::VectorXf& a, const Eigen::VectorXf& b) const {
    return a.dot(b) / (a.norm() * b.norm() + 1e-6f);
}

CloudT::Ptr DualDescriptorNode::cropCloud(const CloudT::Ptr& cloud, const Eigen::Vector3f& center,
                                           float radius) const {
    CloudT::Ptr cropped(new CloudT);
    float radius_sq = radius * radius;

    for (const auto& pt : cloud->points) {
        float dx = pt.x - center(0);
        float dy = pt.y - center(1);
        float dz = pt.z - center(2);
        if (dx * dx + dy * dy + dz * dz < radius_sq) {
            cropped->push_back(pt);
        }
    }
    return cropped;
}

}  // namespace dual_descriptor_relocalization
