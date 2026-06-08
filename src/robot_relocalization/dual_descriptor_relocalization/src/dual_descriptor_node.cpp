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
    rclcpp::TimerBase::SharedPtr init_tf_timer;
    init_tf_timer = this->create_wall_timer(
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
                "keyframes=%zu polar_bins=%d sc=[%dx%d] sc_radius=%.1f trigger_service=%s",
                keyframes_.size(), polar_bins_, sc_num_ring_, sc_num_sector_, sc_max_radius_,
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
    top_k_ = this->declare_parameter("top_k", 10);
    polar_cc_search_bins_ = this->declare_parameter("polar_cc_search_bins", 18);
    sc_weight_ = this->declare_parameter("sc_weight", 1.0f);
    fft_weight_ = this->declare_parameter("fft_weight", 1.0f);

    // 服务参数
    trigger_service_name_ = this->declare_parameter("trigger_service_name", "/relocalization_trigger");

    // ICP 参数
    icp_resolutions_ = this->declare_parameter("icp_resolutions", std::vector<double>{1.0, 0.5, 0.2});
    icp_iterations_ = this->declare_parameter("icp_iterations", 5);
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

    namespace fs = std::filesystem;
    std::string v2_path = keyframe_db_path_ + "/keyframes_v2.bin";
    std::string clouds_dir = keyframe_db_path_ + "/clouds";

    if (!fs::exists(v2_path)) {
        RCLCPP_ERROR(this->get_logger(), "keyframes_v2.bin not found: %s", v2_path.c_str());
        throw std::runtime_error("keyframes_v2.bin not found: " + v2_path);
    }

    std::ifstream f(v2_path, std::ios::binary);
    if (!f.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Cannot open %s", v2_path.c_str());
        throw std::runtime_error("Cannot open: " + v2_path);
    }

    uint32_t magic = 0, count = 0;
    int32_t file_ring_bins = 0, file_sc_ring = 0, file_sc_sector = 0;
    float file_sc_max_radius = 0.0f;
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

    RCLCPP_INFO(this->get_logger(), "V2: %u keyframes, ring=%d sc=[%dx%d] sc_radius=%.1f",
                count, file_ring_bins, file_sc_ring, file_sc_sector, file_sc_max_radius);

    if (file_ring_bins != polar_bins_) {
        RCLCPP_WARN(this->get_logger(), "ring_bins mismatch: file=%d config=%d",
                     file_ring_bins, polar_bins_);
        polar_bins_ = file_ring_bins;
    }
    sc_num_ring_ = file_sc_ring;
    sc_num_sector_ = file_sc_sector;
    sc_max_radius_ = file_sc_max_radius;

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

        // Polar FFT 幅度 (离线)
        int fft_size = polar_bins_ / 2;
        kf.polar_fft_mag.resize(fft_size);
        f.read(reinterpret_cast<char*>(kf.polar_fft_mag.data()),
               fft_size * sizeof(float));

        // Polar 原始 ring (离线, 互相关用)
        kf.polar_raw.resize(polar_bins_);
        f.read(reinterpret_cast<char*>(kf.polar_raw.data()),
               polar_bins_ * sizeof(float));

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
        float theta_deg = (theta * 180.0f / M_PI + 180.0f);  // [0, 360)

        int ring_idx = std::min(sc_num_ring_ - 1, static_cast<int>(rho / sc_max_radius_ * sc_num_ring_));
        int sector_idx = std::min(sc_num_sector_ - 1, static_cast<int>(theta_deg / 360.0f * sc_num_sector_));

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
//  PolarRing 互相关求精 yaw
// ══════════════════════════════════════════════════════════════

int DualDescriptorNode::polarCrossCorrYaw(const std::vector<float>& src,
                                           const std::vector<float>& tgt,
                                           int coarse_bin, int window_bins,
                                           float& best_corr) const {
    best_corr = -1e9f;
    int best_s = 0;
    int start = coarse_bin - window_bins;
    int end   = coarse_bin + window_bins;
    for (int s = start; s <= end; s++) {
        float corr = 0.0f;
        int shift = (s % polar_bins_ + polar_bins_) % polar_bins_;
        for (int j = 0; j < polar_bins_; j++) {
            corr += src[j] * tgt[(j + shift) % polar_bins_];
        }
        if (corr > best_corr) { best_corr = corr; best_s = s; }
    }
    return ((best_s % polar_bins_) + polar_bins_) % polar_bins_;
}

// ══════════════════════════════════════════════════════════════
//  BEV 质心 (平移估计)
// ══════════════════════════════════════════════════════════════

Eigen::Vector2f DualDescriptorNode::bevCentroid(const CloudT::Ptr& cloud,
                                                  float zmin, float zmax) const {
    Eigen::Vector2f sum(0, 0);
    int cnt = 0;
    for (const auto& pt : cloud->points) {
        if (pt.z >= zmin && pt.z <= zmax) {
            sum += Eigen::Vector2f(pt.x, pt.y);
            cnt++;
        }
    }
    return cnt > 0 ? sum / cnt : Eigen::Vector2f(0, 0);
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
        c.combined_sim = sc_weight_ * sc_score + fft_weight_ * fft_sim;
        c.sc_best_shift = best_shift;
        c.sc_align_score = sc_score;
        candidates.push_back(c);
    }

    if (candidates.empty()) {
        RCLCPP_WARN(this->get_logger(), "No valid keyframes");
        return;
    }

    // 按 combined_sim 降序排序
    std::sort(candidates.begin(), candidates.end(),
        [](const Candidate& a, const Candidate& b) {
            return a.combined_sim > b.combined_sim;
        });

    if (candidates.size() > static_cast<size_t>(top_k_)) {
        candidates.resize(top_k_);
    }

    RCLCPP_DEBUG(this->get_logger(), "=== Relocalization start: %zu keyframes, top-%d candidates ===",
                 keyframes_.size(), static_cast<int>(candidates.size()));
    for (size_t i = 0; i < candidates.size(); i++) {
        RCLCPP_DEBUG(this->get_logger(), "  [%zu] kf=%d combined_sim=%.4f sc_score=%.4f sc_shift=%d",
                     i, candidates[i].keyframe_id, candidates[i].combined_sim,
                     candidates[i].sc_align_score, candidates[i].sc_best_shift);
    }

    // ── 逐候选验证 ──
    float best_fitness = 1e6f;
    Eigen::Matrix4f best_pose = Eigen::Matrix4f::Identity();
    int best_kf_id = -1;

    int bins_per_sector = polar_bins_ / sc_num_sector_;  // 360/60 = 6

    for (auto& c : candidates) {
        int kf_id = c.keyframe_id;
        if (kf_id < 0 || kf_id >= static_cast<int>(keyframes_.size())) continue;

        const auto& kf = keyframes_[kf_id];
        if (!kf.cloud || kf.cloud->empty()) continue;

        RCLCPP_DEBUG(this->get_logger(), "--- Candidate kf=%d sim=%.4f ---", kf_id, c.combined_sim);

        // 1. SC 列对齐 → 粗 yaw (复用扫描时已算好的 best_shift)
        float yaw_coarse_deg = c.sc_best_shift * (360.0f / sc_num_sector_);
        RCLCPP_DEBUG(this->get_logger(), "  SC align: shift=%d/60 yaw=%.1f° score=%.4f",
                     c.sc_best_shift, yaw_coarse_deg, c.sc_align_score);

        // 2. PolarRing 互相关 → 精 yaw (在 SC 给的窗口内搜)
        int coarse_bin = c.sc_best_shift * bins_per_sector;
        float polar_corr;
        int best_shift = polarCrossCorrYaw(src_polar.max_rho, kf.polar_raw,
                                            coarse_bin, polar_cc_search_bins_, polar_corr);
        float yaw_fine_rad = best_shift * (2.0f * M_PI / polar_bins_);
        float yaw_fine_deg = best_shift * (360.0f / polar_bins_);
        RCLCPP_DEBUG(this->get_logger(), "  Polar CC: coarse_bin=%d best=%d yaw=%.1f° corr=%.4f",
                     coarse_bin, best_shift, yaw_fine_deg, polar_corr);

        // 3. BEV 质心 → 平移
        Eigen::Matrix4f rot_mat = Eigen::Matrix4f::Identity();
        rot_mat.block<3,3>(0,0) = Eigen::AngleAxisf(yaw_fine_rad,
            Eigen::Vector3f::UnitZ()).toRotationMatrix();

        CloudT::Ptr src_rotated(new CloudT);
        pcl::transformPointCloud(*source, *src_rotated, rot_mat);

        Eigen::Vector2f src_cent = bevCentroid(src_rotated, polar_bev_z_min_, polar_bev_z_max_);
        Eigen::Vector2f tgt_cent = bevCentroid(kf.cloud, polar_bev_z_min_, polar_bev_z_max_);
        float dx = tgt_cent.x() - src_cent.x();
        float dy = tgt_cent.y() - src_cent.y();
        RCLCPP_DEBUG(this->get_logger(), "  BEV centroid: dx=%.2f dy=%.2f src=(%.1f,%.1f) tgt=(%.1f,%.1f)",
                     dx, dy, src_cent.x(), src_cent.y(), tgt_cent.x(), tgt_cent.y());

        // 4. 构造 ICP 初值
        Eigen::Matrix4f init_guess = Eigen::Matrix4f::Identity();
        init_guess.block<3,3>(0,0) = rot_mat.block<3,3>(0,0);
        init_guess(0,3) = dx;
        init_guess(1,3) = dy;
        RCLCPP_DEBUG(this->get_logger(), "  Init guess: yaw=%.1f° pos=(%.2f,%.2f)",
                     yaw_fine_deg, dx, dy);

        // 5. ICP 精配准
        Eigen::Matrix4f icp_pose;
        float fitness = icpRefine(source, kf.cloud, icp_pose, init_guess);
        c.icp_fitness = fitness;

        RCLCPP_DEBUG(this->get_logger(), "  ICP: fitness=%.4f",
                     fitness);

        if (fitness < best_fitness) {
            best_fitness = fitness;
            best_pose = icp_pose;
            best_kf_id = kf_id;
        }
    }

    // ── 阈值判决 ──
    if (best_fitness > icp_fitness_threshold_) {
        RCLCPP_WARN(this->get_logger(), "Reloc FAILED: best_fitness=%.4f > thresh=%.4f, retry next frame",
                    best_fitness, icp_fitness_threshold_);
        return;
    }

    float best_yaw_deg = std::atan2(best_pose(1,0), best_pose(0,0)) * 180.0f / M_PI;
    RCLCPP_INFO(this->get_logger(),
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
