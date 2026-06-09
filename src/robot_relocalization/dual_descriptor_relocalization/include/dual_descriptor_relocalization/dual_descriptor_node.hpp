#ifndef DUAL_DESCRIPTOR_NODE_HPP_
#define DUAL_DESCRIPTOR_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Dense>
#include <vector>
#include <string>
#include <memory>

namespace dual_descriptor_relocalization {

using PointT = pcl::PointXYZI;
using CloudT = pcl::PointCloud<PointT>;

struct PolarRing {
    int num_bins = 360;
    float bin_angle = 2.0f * M_PI / 360.0f;
    std::vector<float> max_rho;
};

struct SCMatrix {
    int num_ring = 20;
    int num_sector = 60;
    float max_radius = 80.0f;
    Eigen::MatrixXf matrix;
};

struct Keyframe {
    int id;
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    CloudT::Ptr cloud;
    std::vector<float> polar_fft_mag;   // 180维, FFT 幅度 → 余弦扫描
    std::vector<float> sc_matrix;       // 120×20 = 2400维, SC (20×120 row-major) → 列对齐
};

struct Candidate {
    int keyframe_id = -1;
    float combined_sim = 0.0f;
    float fft_sim = 0.0f;        // FFT 余弦相似度
    int sc_best_shift = 0;       // SC 列对齐最佳移位数
    float sc_align_score = 0.0f; // SC 列对齐分数 (归一化到 [0,1])
    float icp_fitness = 1e6f;
    Eigen::Matrix4f icp_pose = Eigen::Matrix4f::Identity();
};

class DualDescriptorNode : public rclcpp::Node {
public:
    explicit DualDescriptorNode(const rclcpp::NodeOptions& options);
    ~DualDescriptorNode();

private:
    void loadParameters();
    void loadKeyframeDatabase();

    // 描述子提取
    PolarRing extractPolarRing(const CloudT::Ptr& cloud) const;
    SCMatrix extractSCMatrix(const CloudT::Ptr& cloud) const;
    void extractFFTMagnitude(const std::vector<float>& ring,
                             std::vector<float>& mag) const;

    // yaw 估计 (SC 列对齐)
    int scColumnAlign(const Eigen::MatrixXf& src,
                      const std::vector<float>& tgt_mat,
                      float& best_dist) const;

    // ICP 精配准
    float icpRefine(const CloudT::Ptr& source, const CloudT::Ptr& target,
                    Eigen::Matrix4f& result,
                    const Eigen::Matrix4f& init_guess = Eigen::Matrix4f::Identity()) const;

    // 主流程
    void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void publishPose(const Eigen::Matrix4f& pose, float confidence);

    // 休眠/唤醒
    void enterSleep();
    void wakeUp();
    void triggerCallback(const std_srvs::srv::Trigger::Request::SharedPtr request,
                         std_srvs::srv::Trigger::Response::SharedPtr response);

    // 工具函数
    float cosineSimilarity(const std::vector<float>& a, const std::vector<float>& b) const;

    // 参数
    std::string keyframe_db_path_;
    std::string pointcloud_topic_;
    std::string pose_topic_;
    std::string map_frame_id_;
    std::string base_frame_id_;
    std::string laser_frame_id_;

    int polar_bins_;
    float polar_bev_z_min_;
    float polar_bev_z_max_;

    int sc_num_ring_;
    int sc_num_sector_;
    float sc_max_radius_;
    float sc_bev_z_min_;

    int top_k_;
    float sc_weight_;
    float fft_weight_;

    std::string trigger_service_name_;

    std::vector<double> icp_resolutions_;
    int icp_iterations_;
    float icp_fitness_threshold_;

    // 数据
    std::vector<Keyframe> keyframes_;

    // TF
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr init_tf_timer_;
    Eigen::Matrix4f base_to_sensor_T_ = Eigen::Matrix4f::Identity();
    bool has_sensor_tf_ = false;

    // 发布/订阅/服务
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr trigger_service_;

    // 状态
    bool initialized_ = false;
    bool sleeping_ = true;
};

}  // namespace dual_descriptor_relocalization

#endif  // DUAL_DESCRIPTOR_NODE_HPP_
