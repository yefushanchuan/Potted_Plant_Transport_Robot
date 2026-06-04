#ifndef DUAL_DESCRIPTOR_NODE_HPP_
#define DUAL_DESCRIPTOR_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/io/pcd_io.h>

#include <Eigen/Dense>
#include <vector>
#include <string>
#include <memory>
#include <map>

namespace dual_descriptor_relocalization {

using PointT = pcl::PointXYZI;
using CloudT = pcl::PointCloud<PointT>;

// ══════════════════════════════════════════════════════════════
//  数据结构
// ══════════════════════════════════════════════════════════════

// 极坐标环特征
struct PolarRing {
    int num_bins = 360;
    float bin_angle = 2.0f * M_PI / 360.0f;
    std::vector<float> max_rho;  // 每个 bin 的最大距离
};

// SC 矩阵特征
struct SCMatrix {
    int num_ring = 20;
    int num_sector = 60;
    float max_radius = 80.0f;
    Eigen::MatrixXf data;  // num_ring x num_sector，每个 bin 的最大高度
};

// 关键帧数据
struct Keyframe {
    int id;
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    CloudT::Ptr cloud;

    // 双描述子
    PolarRing polar_ring;
    SCMatrix sc_matrix;

    // SC 的 Ring Key（用于 KD-Tree）
    Eigen::VectorXf ring_key;
};

// 候选结果
struct Candidate {
    int keyframe_id = -1;
    float polar_sim = 0.0f;   // 极坐标环余弦相似度
    float sc_sim = 0.0f;      // SC 矩阵余弦相似度
    int votes = 0;            // 投票数
    float icp_fitness = 1e6f; // ICP fitness
    Eigen::Matrix4f icp_pose = Eigen::Matrix4f::Identity();
};

// ══════════════════════════════════════════════════════════════
//  节点类
// ══════════════════════════════════════════════════════════════

class DualDescriptorNode : public rclcpp::Node {
public:
    explicit DualDescriptorNode(const rclcpp::NodeOptions& options);
    ~DualDescriptorNode();

private:
    // ── 初始化 ──
    void loadParameters();
    void loadKeyframeDatabase();
    void buildKDTrees();

    // ── 描述子提取 ──
    PolarRing extractPolarRing(const CloudT::Ptr& cloud) const;
    SCMatrix extractSCMatrix(const CloudT::Ptr& cloud) const;
    Eigen::VectorXf extractRingKey(const SCMatrix& sc) const;

    // ── 搜索 ──
    std::vector<int> searchPolarCandidates(const PolarRing& query, int top_k) const;
    std::vector<int> searchSCCandidates(const Eigen::VectorXf& query, int top_k) const;

    // ── 投票机制 ──
    std::vector<Candidate> voteCandidates(
        const std::vector<int>& polar_candidates,
        const std::vector<int>& sc_candidates) const;

    // ── ICP 精配准 ──
    float icpRefine(const CloudT::Ptr& source, const CloudT::Ptr& target,
                    Eigen::Matrix4f& result) const;

    // ── 主流程 ──
    void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void publishPose(const Eigen::Matrix4f& pose, float confidence);

    // ── 休眠/唤醒 ──
    void enterSleep();
    void wakeUp();
    void triggerCallback(const std_srvs::srv::Trigger::Request::SharedPtr request,
                         std_srvs::srv::Trigger::Response::Response::SharedPtr response);

    // ── 工具函数 ──
    float cosineSimilarity(const std::vector<float>& a, const std::vector<float>& b) const;
    float cosineSimilarity(const Eigen::VectorXf& a, const Eigen::VectorXf& b) const;
    CloudT::Ptr cropCloud(const CloudT::Ptr& cloud, const Eigen::Vector3f& center, float radius) const;

    // ── 参数 ──
    std::string keyframe_db_path_;
    std::string pointcloud_topic_;
    std::string pose_topic_;
    std::string map_frame_id_;
    std::string base_frame_id_;
    std::string laser_frame_id_;

    // 极坐标环参数
    int polar_bins_;
    float polar_bev_z_min_;
    float polar_bev_z_max_;

    // SC 矩阵参数
    int sc_num_ring_;
    int sc_num_sector_;
    float sc_max_radius_;
    float sc_bev_z_min_;

    // 搜索参数
    int num_candidates_;

    // 服务参数
    std::string trigger_service_name_;

    // ICP 参数
    std::vector<float> icp_resolutions_;
    int icp_iterations_;
    float icp_max_corr_dist_;
    float icp_fitness_threshold_;

    // ── 数据 ──
    std::vector<Keyframe> keyframes_;
    pcl::KdTreeFLANN<PointT> polar_kdtree_;  // 极坐标环的 KD-Tree（用点云坐标存储）
    pcl::KdTreeFLANN<PointT> sc_kdtree_;     // SC Ring Key 的 KD-Tree

    // 极坐标环的特征矩阵（用于 KD-Tree 搜索）
    CloudT::Ptr polar_features_cloud_;
    CloudT::Ptr sc_features_cloud_;

    // ── TF ──
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    Eigen::Matrix4f base_to_sensor_T_ = Eigen::Matrix4f::Identity();
    bool has_sensor_tf_ = false;

    // ── 发布/订阅/服务 ──
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr trigger_service_;

    // ── 状态 ──
    bool initialized_ = false;
    bool sleeping_ = true;  // 启动时默认休眠
};

}  // namespace dual_descriptor_relocalization

#endif  // DUAL_DESCRIPTOR_NODE_HPP_
