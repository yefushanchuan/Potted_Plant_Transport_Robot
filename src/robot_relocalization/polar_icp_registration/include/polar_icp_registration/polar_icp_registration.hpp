#ifndef POLAR_ICP_REGISTRATION_HPP
#define POLAR_ICP_REGISTRATION_HPP

// std
#include <filesystem>
#include <vector>

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
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/octree/octree_search.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/features/normal_3d.h>

namespace polar_icp {

using PointXYZI    = pcl::PointXYZI;
using PointXYZIN   = pcl::PointXYZINormal;
using CloudXYZI    = pcl::PointCloud<PointXYZI>;
using CloudXYZIN   = pcl::PointCloud<PointXYZIN>;

// ── 极坐标环特征 ──────────────────────────────────────
struct PolarRing {
    std::vector<float> max_rho;   // 每个角度 bin 的最大 ρ
    int num_bins;                 // bin 总数（如 360）
    float bin_angle;              // 每 bin 角度（弧度）
};

// ── 八叉树层级 ────────────────────────────────────────
struct OctreeLevel {
    pcl::octree::OctreePointCloudSearch<PointXYZI>::Ptr octree;
    CloudXYZI::Ptr cloud;
    float resolution;
};

// ── 关键帧 ────────────────────────────────────────────
struct KeyframeData {
    Eigen::Matrix4d pose;              // 世界坐标系下的位姿
    PolarRing ring;                    // 预计算的环特征
    CloudXYZI::Ptr cloud;              // 局部点云
    pcl::octree::OctreePointCloudSearch<PointXYZI>::Ptr octree;  // 八叉树（用于精配准）
};

// ── 节点类 ────────────────────────────────────────────
class PolarIcpNode : public rclcpp::Node {
public:
    PolarIcpNode(const rclcpp::NodeOptions &options);
    ~PolarIcpNode();

private:
    // ── ROS 回调 ──
    void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void getStaticTf();

    // ── 关键帧管理 ──
    void loadKeyframeDatabase(const std::string &db_dir);
    int findBestKeyframe(const PolarRing &src_ring) const;

    // ── 粗配准：极坐标环 ──
    PolarRing extractPolarRing(const CloudXYZI::Ptr &cloud) const;
    float computeYawOffset(const PolarRing &src, const PolarRing &tgt) const;
    Eigen::Vector2f computeTranslation(const CloudXYZI::Ptr &src,
                                       const CloudXYZI::Ptr &tgt,
                                       float yaw_rad) const;

    // ── 精配准：多分辨率 ICP ──
    Eigen::Matrix4d multiResICP(const CloudXYZI::Ptr &source,
                                const Eigen::Matrix4d &init_guess,
                                const KeyframeData &kf);

    // ── 主流程 ──
    Eigen::Matrix4d coarseToFineAlign(CloudXYZI::Ptr source,
                                      const Eigen::Matrix4d &init_guess);

    // ── 工具函数 ──
    static CloudXYZIN::Ptr addNormals(const CloudXYZI::Ptr &cloud, int k = 15);

    // ── ROS 接口 ──
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr reloc_pose_pub_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // ── 点云缓存 ──
    CloudXYZI::Ptr cloud_in_;

    // ── 地图数据 ──
    CloudXYZI::Ptr raw_map_;              // 原始加载的点云（用于发布地图和兼容非关键帧模式）
    CloudXYZIN::Ptr refine_map_;          // 精配准用（带法线，非关键帧模式）
    PolarRing map_ring_;                  // 预计算的地图环特征（非关键帧模式）
    std::vector<OctreeLevel> octree_levels_;  // 非关键帧模式

    // ── 关键帧数据 ──
    std::vector<KeyframeData> keyframes_;
    bool use_keyframes_ = false;

    // ── ICP 对象 ──
    pcl::IterativeClosestPointWithNormals<PointXYZIN, PointXYZIN> icp_refine_;

    // ── 体素滤波器 ──
    pcl::VoxelGrid<PointXYZI> voxel_refine_filter_;

    // ── 参数 ──
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

    // 粗配准参数
    int ring_bins_;
    float bev_z_min_;
    float bev_z_max_;
    float bev_crop_radius_;

    // 精配准参数
    double thresh_;
    double drift_threshold_;
    int refine_iter_;
    std::vector<double> octree_resolutions_;

    // 关键帧数据库路径
    std::string keyframe_db_path_;

    // TF
    geometry_msgs::msg::TransformStamped map_to_odom_tf_;
    geometry_msgs::msg::Pose initial_pose_msg_;
    Eigen::Matrix4f base_to_sensor_T_;

    // 定时器
    rclcpp::TimerBase::SharedPtr init_tf_timer_;
    rclcpp::TimerBase::SharedPtr tf_timer_;
};

}  // namespace polar_icp

#endif  // POLAR_ICP_REGISTRATION_HPP
