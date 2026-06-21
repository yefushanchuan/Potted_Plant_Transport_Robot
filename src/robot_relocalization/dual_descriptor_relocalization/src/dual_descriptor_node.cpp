#include "dual_descriptor_relocalization/dual_descriptor_node.hpp"

#include <cstdint>
#include <fstream>
#include <filesystem>
#include <algorithm>
#include <numeric>
#include <cmath>

#include <pcl/kdtree/kdtree_flann.h>

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
    init_tf_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        [this]() {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "Trying to get TF: %s -> %s",
                                 base_frame_id_.c_str(), laser_frame_id_.c_str());
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
            } catch (tf2::TransformException& ex) {
                // 打印详细的 TF 错误信息
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                     "TF lookup failed: %s -> %s: %s",
                                     base_frame_id_.c_str(), laser_frame_id_.c_str(), ex.what());
                // 检查 TF buffer 中有哪些 frames
                std::vector<std::string> frames = tf_buffer_->getAllFrameNames();
                std::string all_frames;
                for (const auto& f : frames) all_frames += f + " ";
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                     "Available frames: [%s]", all_frames.c_str());
            }
        });

    // 加载关键帧数据库
    loadKeyframeDatabase();

    // 发布/订阅/服务
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        pose_topic_, rclcpp::QoS(1));

    // 服务：供老节点触发重新定位
    trigger_service_ = this->create_service<std_srvs::srv::Trigger>(
        trigger_service_name_,
        std::bind(&DualDescriptorNode::triggerCallback, this, std::placeholders::_1, std::placeholders::_2));

    // 开机自动搜索，直接订阅点云
    wakeUp();

    RCLCPP_INFO(this->get_logger(), "DualDescriptorNode initialized (active, power-on search) | "
                "keyframes=%zu trigger_service=%s",
                keyframes_.size(), trigger_service_name_.c_str());
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

    // 搜索参数
    top_k_ = this->declare_parameter("top_k", 10);
    sc_weight_ = this->declare_parameter("sc_weight", 1.0f);
    fft_weight_ = this->declare_parameter("fft_weight", 1.0f);
    yaw_method_ = this->declare_parameter("yaw_method", std::string("sc"));
    polar_search_half_ = this->declare_parameter("polar_search_half", 10);

    // 服务参数
    trigger_service_name_ = this->declare_parameter("trigger_service_name", "/relocalization_trigger");

    // ICP 参数
    icp_resolutions_ = this->declare_parameter("icp_resolutions", std::vector<double>{1.0, 0.5, 0.2});
    icp_iterations_ = this->declare_parameter("icp_iterations", 15);
    icp_fitness_threshold_ = this->declare_parameter("icp_fitness_threshold", 0.5f);

    RCLCPP_DEBUG(this->get_logger(), "Parameters loaded");
}

// ══════════════════════════════════════════════════════════════
//  关键帧数据库加载
// ══════════════════════════════════════════════════════════════

void DualDescriptorNode::loadKeyframeDatabase() {
    if (keyframe_db_path_.empty()) {
        RCLCPP_ERROR(this->get_logger(), "keyframe_db path is empty!");
        throw std::runtime_error("keyframe_db path is empty!");
    }

    // 展开 ~ 为 home 目录
    if (keyframe_db_path_.size() > 0 && keyframe_db_path_[0] == '~') {
        const char* home = std::getenv("HOME");
        if (home) {
            keyframe_db_path_ = std::string(home) + keyframe_db_path_.substr(1);
        }
    }

    namespace fs = std::filesystem;
    std::string bin_path = keyframe_db_path_ + "/keyframes.bin";
    std::string clouds_dir = keyframe_db_path_ + "/clouds";

    if (!fs::exists(bin_path)) {
        RCLCPP_ERROR(this->get_logger(), "keyframes.bin not found: %s", bin_path.c_str());
        throw std::runtime_error("keyframes.bin not found: " + bin_path);
    }

    std::ifstream f(bin_path, std::ios::binary);
    if (!f.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Cannot open %s", bin_path.c_str());
        throw std::runtime_error("Cannot open: " + bin_path);
    }

    uint32_t magic = 0, count = 0;
    int32_t file_ring_bins = 0, file_sc_ring = 0, file_sc_sector = 0;
    float file_sc_max_radius = 0.0f;
    float file_polar_bev_z_min = 0.2f, file_polar_bev_z_max = 3.0f, file_sc_bev_z_min = 0.1f;
    f.read(reinterpret_cast<char*>(&magic), sizeof(magic));
    if (magic != 0x44445343) {
        RCLCPP_ERROR(this->get_logger(), "Invalid magic: 0x%X", magic);
        throw std::runtime_error("Invalid keyframe database format");
    }
    f.read(reinterpret_cast<char*>(&count), sizeof(count));
    f.read(reinterpret_cast<char*>(&file_ring_bins), sizeof(file_ring_bins));
    f.read(reinterpret_cast<char*>(&file_sc_ring), sizeof(file_sc_ring));
    f.read(reinterpret_cast<char*>(&file_sc_sector), sizeof(file_sc_sector));
    f.read(reinterpret_cast<char*>(&file_sc_max_radius), sizeof(file_sc_max_radius));
    f.read(reinterpret_cast<char*>(&file_polar_bev_z_min), sizeof(float));
    f.read(reinterpret_cast<char*>(&file_polar_bev_z_max), sizeof(float));
    f.read(reinterpret_cast<char*>(&file_sc_bev_z_min), sizeof(float));

    polar_bins_ = file_ring_bins;
    sc_num_ring_ = file_sc_ring;
    sc_num_sector_ = file_sc_sector;
    sc_max_radius_ = file_sc_max_radius;
    polar_bev_z_min_ = file_polar_bev_z_min;
    polar_bev_z_max_ = file_polar_bev_z_max;
    sc_bev_z_min_ = file_sc_bev_z_min;

    RCLCPP_INFO(this->get_logger(),
                "Feature params from DB: keyframes=%u polar_bins=%d sc=[%dx%d] sc_radius=%.1f "
                "polar_z=[%.2f,%.2f] sc_z_min=%.2f",
                count, polar_bins_, sc_num_ring_, sc_num_sector_, sc_max_radius_,
                polar_bev_z_min_, polar_bev_z_max_, sc_bev_z_min_);

    keyframes_.resize(count);
    for (uint32_t i = 0; i < count; i++) {
        auto& kf = keyframes_[i];
        kf.id = i;

        double dx, dy, dz, dqw, dqx, dqy, dqz;
        f.read(reinterpret_cast<char*>(&dx), sizeof(double));
        f.read(reinterpret_cast<char*>(&dy), sizeof(double));
        f.read(reinterpret_cast<char*>(&dz), sizeof(double));
        f.read(reinterpret_cast<char*>(&dqw), sizeof(double));
        f.read(reinterpret_cast<char*>(&dqx), sizeof(double));
        f.read(reinterpret_cast<char*>(&dqy), sizeof(double));
        f.read(reinterpret_cast<char*>(&dqz), sizeof(double));

        Eigen::Quaternionf q(static_cast<float>(dqw), static_cast<float>(dqx),
                             static_cast<float>(dqy), static_cast<float>(dqz));
        kf.pose = Eigen::Matrix4f::Identity();
        kf.pose.block<3, 3>(0, 0) = q.toRotationMatrix();
        kf.pose(0, 3) = static_cast<float>(dx);
        kf.pose(1, 3) = static_cast<float>(dy);
        kf.pose(2, 3) = static_cast<float>(dz);

        // Polar Ring 原始环 (离线, yaw 精搜用)
        kf.polar_ring.resize(polar_bins_);
        f.read(reinterpret_cast<char*>(kf.polar_ring.data()),
               polar_bins_ * sizeof(float));

        // Polar FFT 幅度 (离线)
        int fft_size = polar_bins_ / 2;
        kf.polar_fft_mag.resize(fft_size);
        f.read(reinterpret_cast<char*>(kf.polar_fft_mag.data()),
               fft_size * sizeof(float));

        // SC 完整矩阵 (离线, 列对齐用)
        int sc_flat_size = file_sc_ring * file_sc_sector;
        kf.sc_matrix.resize(sc_flat_size);
        f.read(reinterpret_cast<char*>(kf.sc_matrix.data()),
               sc_flat_size * sizeof(float));

        // 加载点云 (ICP 需要)
        std::string pcd_path = clouds_dir + "/" + std::to_string(i) + ".pcd";
        kf.cloud.reset(new CloudT);
        if (pcl::io::loadPCDFile(pcd_path, *kf.cloud) < 0) {
            RCLCPP_WARN(this->get_logger(), "Cannot load keyframe cloud: %s", pcd_path.c_str());
            continue;
        }
    }
    f.close();

    keyframes_.erase(std::remove_if(keyframes_.begin(), keyframes_.end(),
        [](const Keyframe& kf) { return !kf.cloud || kf.cloud->empty(); }),
        keyframes_.end());

    RCLCPP_INFO(this->get_logger(), "Loaded %zu valid keyframes", keyframes_.size());
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
    sc.matrix = Eigen::MatrixXf::Constant(sc_num_ring_, sc_num_sector_, -1000.0f);

    for (const auto& pt : cloud->points) {
        // 只过滤地面，不过滤天花板（保留高度信息）
        if (pt.z < sc_bev_z_min_) continue;

        float rho = std::sqrt(pt.x * pt.x + pt.y * pt.y);
        if (rho > sc_max_radius_) continue;

        float theta = std::atan2(pt.y, pt.x);
        if (theta < 0) theta += 2.0f * M_PI;  // [0, 2π)

        int ring_idx = std::min(sc_num_ring_ - 1, static_cast<int>(rho / sc_max_radius_ * sc_num_ring_));
        int sector_idx = std::min(sc_num_sector_ - 1, static_cast<int>(theta / (2.0f * M_PI) * sc_num_sector_));

        if (pt.z > sc.matrix(ring_idx, sector_idx)) {
            sc.matrix(ring_idx, sector_idx) = pt.z;  // 保留最大高度
        }
    }

    // 将 -1000 替换为 0
    sc.matrix = (sc.matrix.array() < -999).select(0.0f, sc.matrix);

    return sc;
}

// ══════════════════════════════════════════════════════════════
//  SC 列对齐 (粗 yaw + 匹配分数)
// ══════════════════════════════════════════════════════════════

int DualDescriptorNode::scColumnAlign(const Eigen::MatrixXf& src,
                                       const std::vector<float>& tgt_mat,
                                       float& best_dist) const {
    int best_k = 0;
    best_dist = 1e9f;
    for (int k = 0; k < sc_num_sector_; k++) {
        float dist = 0.0f;
        for (int r = 0; r < sc_num_ring_; r++) {
            for (int c = 0; c < sc_num_sector_; c++) {
                float sv = src(r, c);
                float tv = tgt_mat[r * sc_num_sector_ + (c + k) % sc_num_sector_];
                dist += std::abs(sv - tv);
            }
        }
        if (dist < best_dist) { best_dist = dist; best_k = k; }
    }
    return best_k;
}

// ══════════════════════════════════════════════════════════════
//  PolarRing 精搜 yaw (在指定范围内循环移位)
// ══════════════════════════════════════════════════════════════

int DualDescriptorNode::polarRingFineAlign(const std::vector<float>& src_ring,
                                            const std::vector<float>& tgt_ring,
                                            int center_bin, int search_half,
                                            float& best_dist) const {
    int N = static_cast<int>(src_ring.size());
    if (N == 0 || tgt_ring.size() != static_cast<size_t>(N)) {
        best_dist = 1e9f;
        return center_bin;
    }
    int best_k = center_bin;
    best_dist = 1e9f;
    for (int offset = -search_half; offset <= search_half; offset++) {
        int k = (center_bin + offset + N) % N;
        float dist = 0.0f;
        for (int n = 0; n < N; n++) {
            dist += std::abs(src_ring[n] - tgt_ring[(n + k) % N]);
        }
        if (dist < best_dist) { best_dist = dist; best_k = k; }
    }
    return best_k;
}

// ══════════════════════════════════════════════════════════════
//  FFT 幅度提取 (在线)
// ══════════════════════════════════════════════════════════════

void DualDescriptorNode::extractFFTMagnitude(const std::vector<float>& ring,
                                              std::vector<float>& mag) const {
    int N = ring.size();
    mag.resize(N / 2);
    for (int k = 0; k < N / 2; k++) {
        float real = 0.0f, imag = 0.0f;
        for (int n = 0; n < N; n++) {
            float angle = -2.0f * M_PI * k * n / N;
            real += ring[n] * std::cos(angle);
            imag += ring[n] * std::sin(angle);
        }
        mag[k] = std::sqrt(real * real + imag * imag) / N;
    }
}

// ══════════════════════════════════════════════════════════════
//  ICP 精配准
// ══════════════════════════════════════════════════════════════

float DualDescriptorNode::icpRefine(const CloudT::Ptr& source, const CloudT::Ptr& target,
                                     Eigen::Matrix4f& result,
                                     const Eigen::Matrix4f& init_guess) const {
    if (source->empty() || target->empty()) return 1e6f;

        // 多分辨率 ICP
        Eigen::Matrix4f T = init_guess;

        // 每级对应不同 correspondence 距离
        std::vector<float> max_corr_dists = {5.0f, 2.0f, 1.0f};

        for (size_t res_idx = 0; res_idx < icp_resolutions_.size(); res_idx++) {
            float resolution = icp_resolutions_[res_idx];
            float max_corr = max_corr_dists[std::min(res_idx, max_corr_dists.size() - 1)];

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
            icp.setMaxCorrespondenceDistance(max_corr);
            icp.setMaximumIterations(icp_iterations_);
            icp.setTransformationEpsilon(1e-6);
            icp.setEuclideanFitnessEpsilon(1e-6);

            CloudT aligned;
            icp.align(aligned, T);

            if (icp.hasConverged()) {
                T = icp.getFinalTransformation();
                RCLCPP_DEBUG(this->get_logger(), "ICP res=%.2f corr=%.1f converged, fitness=%.4f",
                             resolution, max_corr, icp.getFitnessScore());
            } else {
                RCLCPP_DEBUG(this->get_logger(), "ICP res=%.2f corr=%.1f NOT converged", resolution, max_corr);
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

    CloudT::Ptr raw(new CloudT);
    pcl::fromROSMsg(*msg, *raw);

    CloudT::Ptr source(new CloudT);
    pcl::transformPointCloud(*raw, *source, base_to_sensor_T_);

    RCLCPP_DEBUG(this->get_logger(), "Pointcloud received: %zu points", source->size());

    // 提取在线特征
    PolarRing src_polar = extractPolarRing(source);
    std::vector<float> src_polar_fft;
    extractFFTMagnitude(src_polar.max_rho, src_polar_fft);

    SCMatrix src_sc = extractSCMatrix(source);

    // ── 全量扫描 (SC 列对齐 + FFT 余弦) ──
    int normalizer = sc_num_ring_ * sc_num_sector_;
    std::vector<Candidate> candidates;
    candidates.reserve(keyframes_.size());

    for (size_t i = 0; i < keyframes_.size(); i++) {
        const auto& kf = keyframes_[i];
        if (!kf.cloud || kf.cloud->empty()) continue;

        // SC 列对齐 → 最佳移位 + 匹配分数
        float sc_dist;
        int best_shift = scColumnAlign(src_sc.matrix, kf.sc_matrix, sc_dist);
        float sc_score = 1.0f / (1.0f + sc_dist / normalizer);  // 归一化到 [0,1]

        // FFT 余弦
        float fft_sim = cosineSimilarity(src_polar_fft, kf.polar_fft_mag);

        Candidate c;
        c.keyframe_id = i;
        c.fft_sim = fft_sim;
        c.combined_sim = sc_weight_ * sc_score + fft_weight_ * fft_sim;
        c.sc_best_shift = best_shift;
        c.sc_align_score = sc_score;

        // yaw 估计：根据 yaw_method 选择 SC 或 PolarRing
        if (yaw_method_ == "polar_ring") {
            float polar_dist;
            int fine_shift = polarRingFineAlign(src_polar.max_rho, kf.polar_ring,
                                                0, polar_search_half_, polar_dist);
            c.fine_yaw_deg = fine_shift * (360.0f / polar_bins_);
        } else {
            c.fine_yaw_deg = best_shift * (360.0f / sc_num_sector_);
        }

        candidates.push_back(c);
    }

    if (candidates.empty()) {
        RCLCPP_WARN(this->get_logger(), "No valid keyframes");
        return;
    }

    // ── DEBUG: 打印所有关键帧的得分和位置 ──
    RCLCPP_INFO(this->get_logger(), "===== ALL KEYFRAME SCORES (%zu total) =====", candidates.size());
    for (size_t i = 0; i < keyframes_.size(); i++) {
        if (!keyframes_[i].cloud || keyframes_[i].cloud->empty()) continue;
        // 找到对应候选
        float sc = 0, fft = 0, csim = 0;
        int shift = 0;
        for (auto& cnd : candidates) {
            if (cnd.keyframe_id == static_cast<int>(i)) {
                sc = cnd.sc_align_score; fft = cnd.fft_sim; csim = cnd.combined_sim; shift = cnd.sc_best_shift;
                break;
            }
        }
        RCLCPP_INFO(this->get_logger(), "  kf_%d pos=(%7.2f,%7.2f) yaw=%6.1f° sc=%.4f fft=%.4f combined=%.4f shift=%d",
                     static_cast<int>(i),
                     keyframes_[i].pose(0,3), keyframes_[i].pose(1,3),
                     std::atan2(keyframes_[i].pose(1,0), keyframes_[i].pose(0,0)) * 180.0f / M_PI,
                     sc, fft, csim, shift);
    }

    // 按 combined_sim 降序排序
    std::sort(candidates.begin(), candidates.end(),
        [](const Candidate& a, const Candidate& b) {
            return a.combined_sim > b.combined_sim;
        });

    if (candidates.size() > static_cast<size_t>(top_k_)) {
        candidates.resize(top_k_);
    }

    RCLCPP_INFO(this->get_logger(), "===== TOP-%d CANDIDATES =====", static_cast<int>(candidates.size()));
    for (size_t i = 0; i < candidates.size(); i++) {
        const auto& kf = keyframes_[candidates[i].keyframe_id];
        RCLCPP_INFO(this->get_logger(), "  [%zu] kf=%d pos=(%7.2f,%7.2f) sc_score=%.4f combined=%.4f yaw=%.1f°",
                     i, candidates[i].keyframe_id,
                     kf.pose(0,3), kf.pose(1,3),
                     candidates[i].sc_align_score, candidates[i].combined_sim,
                     candidates[i].fine_yaw_deg);
    }

    // ── 逐候选验证 ──
    // 按 combined_sim 降序找第一个 fitness 低于阈值的候选
    bool found = false;
    Eigen::Matrix4f best_pose = Eigen::Matrix4f::Identity();
    int best_kf_id = -1;
    float best_fitness = 1e6f;

    for (auto& c : candidates) {
        int kf_id = c.keyframe_id;
        if (kf_id < 0 || kf_id >= static_cast<int>(keyframes_.size())) continue;

        const auto& kf = keyframes_[kf_id];
        if (!kf.cloud || kf.cloud->empty()) continue;

        RCLCPP_INFO(this->get_logger(), "--- Verifying kf=%d pos=(%.2f,%.2f) sim=%.4f ---",
                     kf_id, kf.pose(0,3), kf.pose(1,3), c.combined_sim);

        // 1. 使用候选的 yaw（由 yaw_method 决定：SC 或 PolarRing）
        RCLCPP_INFO(this->get_logger(), "  yaw: method=%s yaw=%.1f° sc_shift=%d/60 sc_score=%.4f",
                     yaw_method_.c_str(), c.fine_yaw_deg,
                     c.sc_best_shift, c.sc_align_score);

        // 2. 使用 fine_yaw 作为初始角度
        float yaw_fine_rad = c.fine_yaw_deg * M_PI / 180.0f;

        // 3. 平移初值：直接用关键帧位姿，让 ICP 修正
        Eigen::Matrix4f rot_mat = Eigen::Matrix4f::Identity();
        rot_mat.block<3,3>(0,0) = Eigen::AngleAxisf(yaw_fine_rad,
            Eigen::Vector3f::UnitZ()).toRotationMatrix();

        float dx = kf.pose(0,3);
        float dy = kf.pose(1,3);
        RCLCPP_INFO(this->get_logger(), "  Init pos: using kf_pos=(%.2f,%.2f) directly", dx, dy);

        // 4. 多初值 ICP 搜索（绕关键帧 ±3m，首位候选做 5×5 网格）
        Eigen::Matrix4f best_local_pose = Eigen::Matrix4f::Identity();
        float best_local_fitness = 1e6f;

        bool is_first = (&c == &candidates[0]);
        const int grid_half = is_first ? 2 : 0;
        const float grid_step = 1.5f;

        for (int gi = -grid_half; gi <= grid_half; gi++) {
            for (int gj = -grid_half; gj <= grid_half; gj++) {
                float gx = dx + gi * grid_step;
                float gy = dy + gj * grid_step;

                Eigen::Matrix4f init_guess = Eigen::Matrix4f::Identity();
                init_guess.block<3,3>(0,0) = rot_mat.block<3,3>(0,0);
                init_guess(0,3) = gx;
                init_guess(1,3) = gy;

                Eigen::Matrix4f icp_pose;
                float fitness = icpRefine(source, kf.cloud, icp_pose, init_guess);
                if (fitness < best_local_fitness) {
                    best_local_fitness = fitness;
                    best_local_pose = icp_pose;
                }
            }
        }

        c.icp_fitness = best_local_fitness;
        c.icp_pose = best_local_pose;

        RCLCPP_INFO(this->get_logger(), "  ICP: %s best_fitness=%.4f pos=(%.2f,%.2f)",
                     is_first ? "grid5x5" : "single", best_local_fitness,
                     best_local_pose(0,3), best_local_pose(1,3));

        if (best_local_fitness < icp_fitness_threshold_) {
            // 首次命中：combined_sim 最高且 fitness 合格的候选即当选
            if (!found) {
                found = true;
                best_fitness = best_local_fitness;
                best_pose = best_local_pose;
                best_kf_id = kf_id;
                RCLCPP_WARN(this->get_logger(), "  >>> ACCEPTED (first with fitness < %.2f)", icp_fitness_threshold_);
                break;
            }
        }
    }

    // ── 阈值判决 ──
    if (!found) {
        RCLCPP_WARN(this->get_logger(), "Reloc FAILED: no candidate met fitness < %.2f, retry next frame",
                    icp_fitness_threshold_);
        return;
    }

    float best_yaw_deg = std::atan2(best_pose(1,0), best_pose(0,0)) * 180.0f / M_PI;
    RCLCPP_WARN(this->get_logger(),
                "Reloc SUCCESS: kf=%d fitness=%.4f yaw=%.1f° pos=(%.2f,%.2f,%.2f)",
                best_kf_id, best_fitness, best_yaw_deg,
                best_pose(0,3), best_pose(1,3), best_pose(2,3));
    publishPose(best_pose, 1.0f / (best_fitness + 1e-6f));

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
    if (!sleeping_ && pointcloud_sub_) {
        RCLCPP_DEBUG(this->get_logger(), "Already awake, skip re-subscribe");
        return;
    }
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

}  // namespace dual_descriptor_relocalization
