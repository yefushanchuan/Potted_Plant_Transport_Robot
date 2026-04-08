#include "p2pl_icp.h"
#include "eskf.h"
#include "cloud_process_livox.h" // Livox 点云处理类
#include "cloud_process_rs.h"    // RS16 点云处理类
#include "utility/utility.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

// TF2 ROS 接口（广播器和监听器）
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
// 几何消息与 TF2 的转换工具（注意 .h → .hpp）
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// 用于性能统计
#include <chrono>
#include <deque>
#include <algorithm>
#include <builtin_interfaces/msg/time.hpp>

// Sophus 用于去畸变
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>

// 用于独立处理线程和信号处理
#include <csignal>
#include <thread>
#include <condition_variable>
#include <atomic>

using namespace std::chrono_literals;

// 全局退出标志,用于信号处理
std::atomic<bool> g_b_exit(false);

// 信号处理函数
void SigHandle(int sig)
{
    g_b_exit.store(true);
    RCLCPP_WARN(rclcpp::get_logger("lidar_location_3d"), "捕获信号 %d, 准备退出...", sig);
    rclcpp::shutdown();
}

bool check_imu_data(Eigen::VectorXd imu_data)
{
    bool is_ok = true;
    if (std::abs(imu_data(0)) > 10000.0)
        is_ok = false;
    if (std::abs(imu_data(1)) > 10000.0)
        is_ok = false;
    if (std::abs(imu_data(2)) > 10000.0)
        is_ok = false;
    if (std::abs(imu_data(3)) > 10000.0)
        is_ok = false;
    if (std::abs(imu_data(4)) > 10000.0)
        is_ok = false;
    if (std::abs(imu_data(5)) > 10000.0)
        is_ok = false;
    return is_ok;
}

class location
{
public:
    rclcpp::Node::SharedPtr nh_                                                        // ROS2 节点对象
    std::string map_file_path;                                                         // 地图路径
    pcl::PointCloud<PointType>::Ptr map_cloud;                                         // 地图点云
    pose_type act_pose;                                                                // 当前位姿
    pose_type init_pose_;                                                              // 初始位姿
    pose_type imu2base_pose;                                                           // imu到baselink相对位姿

    // ROS2 中定义消息
    sensor_msgs::msg::PointCloud2 pubmap_msg; // 作为消息播放的地图

    // 发布者
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_map_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr processed_cloud_pub_; // 处理后点云发布器
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    // rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    //  订阅者
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_pose_sub_;

    // tf 广播器和监听器
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // 记录上一次成功计算的 map->odom，避免 TF 暂时不可用时将树“劈成两棵”（map->base_footprint 会破坏 Nav2 期望的 map->odom->base 链）
    tf2::Transform last_map_to_odom_tf_;
    bool has_last_map_to_odom_tf_ = false;

    pcl::VoxelGrid<PointType> map_voxelgrid_filter_; // 地图点云体素滤波器

    std::unique_ptr<scan2map3d> localizier3d;
    std::unique_ptr<ErrorStateKalmanFilter> eskf; // 误差状态卡尔曼滤波

    std::atomic<bool> has_new_global_pose_{false};
    pose_type global_pose_buffer_;
    std::mutex global_pose_mutex_;
    
    // IMU数据结构（与lio_node保持一致）
    struct IMUData
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Eigen::Vector3d acc;
        Eigen::Vector3d gyro;
        double time;
        IMUData() = default;
        IMUData(const Eigen::Vector3d &a, const Eigen::Vector3d &g, const double &t)
            : acc(a), gyro(g), time(t) {}
    };

    // 同步数据包结构（与lio_node保持一致）
    struct SyncPackage
    {
        std::vector<IMUData> imus;
        pcl::PointCloud<PointType>::Ptr cloud;
        double cloud_start_time = 0.0;
        double cloud_end_time = 0.0;
    };

    // 状态数据结构（与lio_node保持一致）
    struct StateData
    {
        bool lidar_pushed = false;
        std::mutex imu_mutex;
        std::mutex lidar_mutex;
        double last_lidar_time = -1.0;
        double last_imu_time = -1.0;
        std::deque<IMUData> imu_buffer;
        std::deque<std::pair<double, pcl::PointCloud<PointType>::Ptr>> lidar_buffer;
    };

    StateData m_state_data;
    SyncPackage m_package;

    // 调试计数器
    int imu_cb_count_ = 0;
    int lidar_cb_count_ = 0;

    // 时间同步诊断开关
    bool enable_sync_debug_ = true;

    // ========== 点云预处理对象 ==========
    std::unique_ptr<cloud_process_livox> livox_processor_; // Livox点云处理对象
    std::unique_ptr<cloud_process> rs16_processor_;        // RS16点云处理对象
    std::string lidar_type_;                               // 雷达类型: "livox" 或 "rs16"

    int count_maplost_ = 0;
    bool use_reloc_ = false; // 是否使用重定位？

    float reloc_threshold_ = 0.0; // 重定位的匹配度阈值

    double voxel_leaf_size_ = 0.0, imu_scale_ = 0.0;
    double last_cloud_time_ = 0.0; // 上次点云时间戳

    Eigen::Matrix3f imu2link_rot_; // imu2link的旋转矩阵

    std::string map_frame_, base_frame_, odom_frame_;
    bool enable_map_odom_tf_ = true;  // 是否发布 map→odom TF（用于Nav2导航）

    // ===== 发布前位姿重估（仅用于发布链路，不回写算法内部 act_pose） =====
    bool pose_refine_enable_ = true;
    bool pose_refine_force_accept_reloc_ = true;
    bool pose_refine_force_accept_next_ = false;
    bool pose_refine_inited_ = false;

    double pose_refine_gate_xy_m_ = 0.25;
    double pose_refine_gate_yaw_rad_ = 0.35;
    double pose_refine_k_xy_move_ = 0.25;
    double pose_refine_k_yaw_move_ = 0.20;
    double pose_refine_k_xy_static_ = 0.05;
    double pose_refine_k_yaw_static_ = 0.04;
    double pose_refine_static_v_th_ = 0.03;
    double pose_refine_static_w_th_ = 0.05;
    double pose_refine_tf_lookup_timeout_sec_ = 0.0;

    tf2::Transform pose_refine_map_to_base_prev_;
    tf2::Transform pose_refine_odom_to_base_prev_;
    rclcpp::Time pose_refine_last_stamp_{0, 0, RCL_SYSTEM_TIME};

    // ============ 性能统计相关变量 ============
    struct PerformanceStats
    {
        // 处理耗时统计
        double imu_predict_time_ms = 0.0;        // IMU预测耗时
        double cloud_registration_time_ms = 0.0; // 点云配准耗时
        double eskf_correct_time_ms = 0.0;       // ESKF校正耗时
        double pose_publish_time_ms = 0.0;       // 位姿发布耗时
        double total_process_time_ms = 0.0;      // 总处理时间

        // 时间同步信息
        bool sync_success = false;     // 同步是否成功
        double lidar_start_time = 0.0; // 雷达起始时间
        double lidar_end_time = 0.0;   // 雷达结束时间
        double imu_first_time = 0.0;   // 同步后的IMU首帧时间
        double imu_last_time = 0.0;    // 同步后的IMU末帧时间
        int imu_count = 0;             // 同步的IMU数量

        // 调试信息：缓冲区状态
        size_t imu_buffer_before_sync = 0;  // 同步前IMU缓冲区大小
        double imu_buffer_first_time = 0.0; // 缓冲区中第一个IMU时间
        double imu_buffer_last_time = 0.0;  // 缓冲区中最后一个IMU时间
    };

    std::deque<PerformanceStats> perf_history_; // 性能历史记录（用于计算平均值）
    const size_t perf_history_size_ = 50;       // 保留最近50帧的数据

    rclcpp::Time last_pose_publish_time_; // 上次位姿发布时间
    int pose_publish_count_ = 0;          // 位姿发布计数
    rclcpp::Time stats_start_time_;       // 统计开始时间

    rclcpp::TimerBase::SharedPtr stats_timer_; // 定时器，用于定期输出统计信息

    // ============ 独立处理线程相关变量 ============
    std::thread process_thread_;                    // 独立处理线程
    std::mutex process_mutex_;                      // 处理线程同步互斥锁
    std::condition_variable process_cv_;            // 条件变量,用于唤醒处理线程
    // 注意: 使用syncPackage()作为唤醒条件,不需要额外的标志位

    // ============ 去畸变相关变量 ============
    bool enable_undistort_ = false; // 是否启用去畸变功能（暂时禁用）
    int undistort_count_ = 0;       // 去畸变帧计数

    location(rclcpp::Node::SharedPtr nh) : nh_(nh)
    {
        YAML::Node config;
        nh_ = nh;
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(nh_); // 初始化 tf 广播器
        // 初始化 TF Buffer 和 Listener（用于监听 odom→base_footprint）
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(nh_->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        RCLCPP_INFO(nh->get_logger(), "\n--- Loaded location Parameters ---");
        std::string config_file_name;
        nh->declare_parameter<std::string>("config_file", "none");
        nh_->get_parameter("config_file", config_file_name);
        RCLCPP_INFO(nh_->get_logger(), "Loc: config file: %s", config_file_name.c_str());

        // 读取yaml数据
        try
        {
            config = YAML::LoadFile(config_file_name);
        }
        catch (YAML::BadFile &e)
        {
            RCLCPP_ERROR(nh_->get_logger(), "Loc :  read config file error!");
            rclcpp::shutdown(); 
            throw std::runtime_error("Config file not found");
        }

        // 是否使用重定位
        use_reloc_ = config["use_reloc"].as<bool>();
        RCLCPP_INFO(nh_->get_logger(), "use_reloc %d", use_reloc_);

        imu_scale_ = config["imu_scale"].as<double>();
        RCLCPP_INFO(nh_->get_logger(), "imu_scale %f", imu_scale_);

        reloc_threshold_ = config["reloc_threshold"].as<float>();
        RCLCPP_INFO(nh_->get_logger(), "reloc_threshold %f", reloc_threshold_);

        base_frame_ = config["base_frame"].as<std::string>();
        map_frame_ = config["map_frame"].as<std::string>();
        // 读取 odom_frame 配置，默认为 "odom"
        odom_frame_ = config["odom_frame"] ? config["odom_frame"].as<std::string>() : "odom";
        // 是否启用 map→odom TF 发布（用于 Nav2 导航）
        enable_map_odom_tf_ = config["enable_map_odom_tf"] ? config["enable_map_odom_tf"].as<bool>() : true;
        RCLCPP_INFO(nh_->get_logger(), "base_frame_: %s, map_frame_: %s, odom_frame_: %s, enable_map_odom_tf: %d", 
                    base_frame_.c_str(), map_frame_.c_str(), odom_frame_.c_str(), enable_map_odom_tf_);

        // 发布前位姿重估参数（用于抑制定位侧 map->odom 抖动）
        pose_refine_enable_ = config["pose_refine_enable"] ? config["pose_refine_enable"].as<bool>() : true;
        pose_refine_force_accept_reloc_ =
            config["pose_refine_force_accept_reloc"] ? config["pose_refine_force_accept_reloc"].as<bool>() : true;
        pose_refine_gate_xy_m_ = config["pose_refine_gate_xy_m"] ? config["pose_refine_gate_xy_m"].as<double>() : 0.25;
        pose_refine_gate_yaw_rad_ =
            config["pose_refine_gate_yaw_rad"] ? config["pose_refine_gate_yaw_rad"].as<double>() : 0.35;
        pose_refine_k_xy_move_ = config["pose_refine_k_xy_move"] ? config["pose_refine_k_xy_move"].as<double>() : 0.25;
        pose_refine_k_yaw_move_ =
            config["pose_refine_k_yaw_move"] ? config["pose_refine_k_yaw_move"].as<double>() : 0.20;
        pose_refine_k_xy_static_ =
            config["pose_refine_k_xy_static"] ? config["pose_refine_k_xy_static"].as<double>() : 0.05;
        pose_refine_k_yaw_static_ =
            config["pose_refine_k_yaw_static"] ? config["pose_refine_k_yaw_static"].as<double>() : 0.04;
        pose_refine_static_v_th_ =
            config["pose_refine_static_v_th"] ? config["pose_refine_static_v_th"].as<double>() : 0.03;
        pose_refine_static_w_th_ =
            config["pose_refine_static_w_th"] ? config["pose_refine_static_w_th"].as<double>() : 0.05;
        pose_refine_tf_lookup_timeout_sec_ =
            config["pose_refine_tf_lookup_timeout_sec"] ? config["pose_refine_tf_lookup_timeout_sec"].as<double>() : 0.08;

        // 约束增益范围，避免异常配置导致发散
        pose_refine_k_xy_move_ = std::max(0.0, std::min(1.0, pose_refine_k_xy_move_));
        pose_refine_k_yaw_move_ = std::max(0.0, std::min(1.0, pose_refine_k_yaw_move_));
        pose_refine_k_xy_static_ = std::max(0.0, std::min(1.0, pose_refine_k_xy_static_));
        pose_refine_k_yaw_static_ = std::max(0.0, std::min(1.0, pose_refine_k_yaw_static_));

        if (pose_refine_tf_lookup_timeout_sec_ <= 0.0) {
            pose_refine_tf_lookup_timeout_sec_ = 0.08;
        }
        pose_refine_last_stamp_ = rclcpp::Time(0, 0, nh_->get_clock()->get_clock_type());

        RCLCPP_INFO(
            nh_->get_logger(),
            "[PoseRefine] enable=%d force_accept_reloc=%d gate_xy=%.3f gate_yaw=%.3f "
            "k_move(xy=%.2f,yaw=%.2f) k_static(xy=%.2f,yaw=%.2f)",
            pose_refine_enable_, pose_refine_force_accept_reloc_, pose_refine_gate_xy_m_, pose_refine_gate_yaw_rad_,
            pose_refine_k_xy_move_, pose_refine_k_yaw_move_, pose_refine_k_xy_static_, pose_refine_k_yaw_static_);

        // ========== 根据雷达类型初始化对应的点云处理对象 ==========
        lidar_type_ = config["lidar_type"].as<std::string>(); // "livox" 或 "rs16"
        double filter_leaf_size = config["filter_leaf_size"].as<double>();

        // 从 YAML 中读取雷达和 IMU 的坐标系名称（若无则设为默认值）
        std::string lidar_frame_ = config["lidar_frame"] ? config["lidar_frame"].as<std::string>() : "livox_frame";
        std::string imu_frame_   = config["imu_frame"]   ? config["imu_frame"].as<std::string>()   : "imu_link";

        RCLCPP_INFO(nh_->get_logger(), "雷达类型: %s", lidar_type_.c_str());
        RCLCPP_INFO(nh_->get_logger(), "等待获取静态 TF 树: Source[%s] -> Target[%s] & Source[%s] -> Target[%s]", 
                    lidar_frame_.c_str(), base_frame_.c_str(), imu_frame_.c_str(), base_frame_.c_str());

        geometry_msgs::msg::TransformStamped lidar2base_msg;
        geometry_msgs::msg::TransformStamped imu2base_msg;

        // 1. 阻塞等待：Lidar -> Base 的 TF
        while (rclcpp::ok()) {
            try {
                // lookupTransform 参数顺序：(target_frame, source_frame)
                // 这里 target 是 base_frame_，source 是 lidar_frame_
                // 计算结果正好是将点从 lidar 转换到 base，即 lidar2base
                lidar2base_msg = tf_buffer_->lookupTransform(base_frame_, lidar_frame_, tf2::TimePointZero);
                break;
            } catch (const tf2::TransformException &ex) {
                RCLCPP_WARN_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 2000, 
                    "等待获取 TF (Source[%s] -> Target[%s]) 失败: %s", 
                    lidar_frame_.c_str(), base_frame_.c_str(), ex.what());
                rclcpp::sleep_for(std::chrono::milliseconds(500));
            }
        }

        // 2. 阻塞等待：IMU -> Base 的 TF
        while (rclcpp::ok()) {
            try {
                // 同理，target = base, source = imu
                imu2base_msg = tf_buffer_->lookupTransform(base_frame_, imu_frame_, tf2::TimePointZero);
                break;
            } catch (const tf2::TransformException &ex) {
                RCLCPP_WARN_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 2000, 
                    "等待获取 TF (Source[%s] -> Target[%s]) 失败: %s", 
                    imu_frame_.c_str(), base_frame_.c_str(), ex.what());
                rclcpp::sleep_for(std::chrono::milliseconds(500));
            }
        }

        // ========== 解析 Lidar 到 Base 的外参 ==========
        // 这里提取的 translation 就是你在 yaml 里的 lidar2base_x, y, z
        Eigen::Vector3f lidar2base_trans(
            lidar2base_msg.transform.translation.x,
            lidar2base_msg.transform.translation.y,
            lidar2base_msg.transform.translation.z
        );
        
        // 解析四元数到 Roll, Pitch, Yaw
        tf2::Quaternion q_lidar(
            lidar2base_msg.transform.rotation.x,
            lidar2base_msg.transform.rotation.y,
            lidar2base_msg.transform.rotation.z,
            lidar2base_msg.transform.rotation.w
        );
        double l_roll, l_pitch, l_yaw;
        tf2::Matrix3x3(q_lidar).getRPY(l_roll, l_pitch, l_yaw);
        
        Eigen::Vector3f lidar2base_rpy(l_roll, l_pitch, l_yaw);
        Eigen::Matrix3f lidar2base_rot = RPY2Mat(lidar2base_rpy); // 完美复用你原有的 RPY 转矩阵函数

        RCLCPP_INFO(nh_->get_logger(), ">> 雷达外参(Lidar2Base) TF获取成功: trans=(%.3f, %.3f, %.3f), rpy=(%.3f, %.3f, %.3f)",
                    lidar2base_trans(0), lidar2base_trans(1), lidar2base_trans(2),
                    l_roll, l_pitch, l_yaw);

        // ========== 解析 IMU 到 Base 的外参 ==========
        // 这里提取的 translation 就是你在 yaml 里的 imu2base_x, y, z
        imu2base_pose.pos(0) = imu2base_msg.transform.translation.x;
        imu2base_pose.pos(1) = imu2base_msg.transform.translation.y;
        imu2base_pose.pos(2) = imu2base_msg.transform.translation.z;

        tf2::Quaternion q_imu(
            imu2base_msg.transform.rotation.x,
            imu2base_msg.transform.rotation.y,
            imu2base_msg.transform.rotation.z,
            imu2base_msg.transform.rotation.w
        );
        double i_roll, i_pitch, i_yaw;
        tf2::Matrix3x3(q_imu).getRPY(i_roll, i_pitch, i_yaw);
        
        // 赋值给你原有的 imu2base_pose 结构体
        imu2base_pose.orient(0) = i_roll;
        imu2base_pose.orient(1) = i_pitch;
        imu2base_pose.orient(2) = i_yaw;

        update_q(imu2base_pose);
        imu2link_rot_ = RPY2Mat(imu2base_pose.orient); // 生成 IMU 到 Baselink 的旋转矩阵

        RCLCPP_INFO(nh_->get_logger(), ">> IMU外参(IMU2Base) TF获取成功: trans=(%.3f, %.3f, %.3f), rpy=(%.3f, %.3f, %.3f)", 
            imu2base_pose.pos(0), imu2base_pose.pos(1), imu2base_pose.pos(2), 
            i_roll, i_pitch, i_yaw);

        if (lidar_type_ == "livox")
        {
            // 初始化 Livox 处理器
            livox_processor_.reset(new cloud_process_livox(filter_leaf_size));
            livox_processor_->N_SCANS_ = config["N_SCANS"].as<int>();
            livox_processor_->min_range_ = config["min_range"].as<double>();
            livox_processor_->max_range_ = config["max_range"].as<double>();
            livox_processor_->min_angle_ = config["limit_angle_min"].as<double>() * Deg2Rad;
            livox_processor_->max_angle_ = config["limit_angle_max"].as<double>() * Deg2Rad;
            livox_processor_->base2link_rot = lidar2base_rot;
            livox_processor_->base2link_trans = lidar2base_trans;

            RCLCPP_INFO(nh_->get_logger(), "Livox 点云处理器初始化完成: leaf_size=%.2f, range=[%.1f, %.1f], scans=%d",
                        filter_leaf_size, livox_processor_->min_range_, livox_processor_->max_range_,
                        livox_processor_->N_SCANS_);
        }
        else if (lidar_type_ == "rs16")
        {
            // 初始化 RS16 处理器
            rs16_processor_.reset(new cloud_process(filter_leaf_size));
            rs16_processor_->n_scans_ = config["N_SCANS"].as<uint16_t>();
            rs16_processor_->horizon_n_scans_ = config["Horizon_SCAN"].as<uint16_t>();
            rs16_processor_->min_range_ = config["min_range"].as<double>();
            rs16_processor_->max_range_ = config["max_range"].as<double>();
            rs16_processor_->min_angle_ = config["limit_angle_min"].as<double>() * Deg2Rad;
            rs16_processor_->max_angle_ = config["limit_angle_max"].as<double>() * Deg2Rad;
            rs16_processor_->lidar_low_angle_ = config["lidar_low_angle"].as<float>();
            rs16_processor_->lidar_angle_res_x_ = config["lidar_angle_res_x"].as<float>();
            rs16_processor_->lidar_angle_res_y_ = config["lidar_angle_res_y"].as<float>();
            rs16_processor_->base2link_rot = lidar2base_rot;
            rs16_processor_->base2link_trans = lidar2base_trans;

            RCLCPP_INFO(nh_->get_logger(), "RS16 点云处理器初始化完成: leaf_size=%.2f, range=[%.1f, %.1f], scans=%d",
                        filter_leaf_size, rs16_processor_->min_range_, rs16_processor_->max_range_,
                        rs16_processor_->n_scans_);
        }
        else
        {
            RCLCPP_ERROR(nh_->get_logger(), "未知的雷达类型: %s,仅支持 'livox' 或 'rs16'", lidar_type_.c_str());
            return;
        }

        RCLCPP_INFO(nh_->get_logger(), "Base to Link Rotation Matrix:\n [%f, %f, %f;\n %f, %f, %f;\n %f, %f, %f]",
                    imu2link_rot_(0, 0), imu2link_rot_(0, 1), imu2link_rot_(0, 2),
                    imu2link_rot_(1, 0), imu2link_rot_(1, 1), imu2link_rot_(1, 2),
                    imu2link_rot_(2, 0), imu2link_rot_(2, 1), imu2link_rot_(2, 2));

        // 声明并获取初始位置参数
        nh_->declare_parameter("init_x", 0.0);
        nh_->declare_parameter("init_y", 0.0);
        nh_->declare_parameter("init_z", 0.0);
        nh_->declare_parameter("init_roll", 0.0);
        nh_->declare_parameter("init_pitch", 0.0);
        nh_->declare_parameter("init_yaw", 0.0);

        init_pose_.pos(0) = config["init_x"].as<float>();
        init_pose_.pos(1) = config["init_y"].as<float>();
        init_pose_.pos(2) = config["init_z"].as<float>();
        init_pose_.orient(0) = config["init_roll"].as<float>();
        init_pose_.orient(1) = config["init_pitch"].as<float>();
        init_pose_.orient(2) = config["init_yaw"].as<float>();

        RCLCPP_INFO(nh_->get_logger(), "Loc: init_pose_: %f, %f, %f, %f, %f, %f", init_pose_.pos(0), init_pose_.pos(1), init_pose_.pos(2), init_pose_.orient(0), init_pose_.orient(1), init_pose_.orient(2));

        update_q(init_pose_);
        act_pose = init_pose_;
        act_pose.vel << 0.0, 0.0, 0.0;

        // 初始化订阅，发布器（直接订阅原始Livox话题）
        std::string lidar_topic = config["lidar_topic"].as<std::string>();
        // 强制队列长度为 1，只保留最新的一帧点云
        cloud_sub_ = nh_->create_subscription<sensor_msgs::msg::PointCloud2>(lidar_topic, rclcpp::QoS(rclcpp::KeepLast(1)), std::bind(&location::robosense_cb, this, std::placeholders::_1));
        
        std::string cmd_topic = config["cmd_topic"].as<std::string>();
        cmd_sub_ = nh_->create_subscription<std_msgs::msg::String>(cmd_topic, 10, std::bind(&location::command_cb, this, std::placeholders::_1));

        // 单线程executor模式,IMU队列=200: 200Hz频率,队列200可缓存1秒数据,应对处理线程短时繁忙
        std::string imu_topic = config["imu_topic"].as<std::string>();
        
        rclcpp::QoS imu_qos(200); // 自定义 QoS: 队列长度200，并显式指定 BEST_EFFORT 以匹配底层硬件驱动
        imu_qos.best_effort();
        
        imu_sub_ = nh_->create_subscription<sensor_msgs::msg::Imu>(imu_topic, imu_qos, std::bind(&location::imu_cb, this, std::placeholders::_1));

        std::string init_pose_topic = config["init_pose_topic"].as<std::string>();
        init_pose_sub_ = nh_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(init_pose_topic, 1, std::bind(&location::initpose_cb, this, std::placeholders::_1)); // 手动重定位订阅

        std::string map_pub_topic_ = config["map_pub_topic"].as<std::string>();
        pc_map_pub_ = nh_->create_publisher<sensor_msgs::msg::PointCloud2>(map_pub_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable()); // 发布全局pcd点云地图话题数据
        RCLCPP_INFO(nh_->get_logger(), "map_pub_topic_: %s", map_pub_topic_.c_str());

        std::string ros_map_filename;
        nh_->declare_parameter<std::string>("map_filename", ""); // 声明参数，默认值为空
        nh_->get_parameter("map_filename", ros_map_filename);

        // 如果 Launch 文件或终端传了 map_filename，则覆盖 YAML 配置
        if (!ros_map_filename.empty()) {
            map_file_path = ros_map_filename;
            RCLCPP_INFO(nh_->get_logger(), "Loc: [Launch Overwrite] map_file_path: %s", map_file_path.c_str());
        } 
        else {
            // 回退到读取 YAML 文件配置
            if (config["map_filename"]) {
                map_file_path = config["map_filename"].as<std::string>();
                RCLCPP_INFO(nh_->get_logger(), "Loc: [YAML Config] map_file_path: %s", map_file_path.c_str());
            } else {
                RCLCPP_ERROR(nh_->get_logger(), "Loc: No map_filename provided in Launch args or YAML config!");
            }
        }
        map_cloud.reset(new pcl::PointCloud<PointType>);

        // 直接加载为PointType类型，避免类型转换循环
        if (pcl::io::loadPCDFile(map_file_path.c_str(), *map_cloud) == -1)
        {
            RCLCPP_ERROR(nh_->get_logger(), "Loc: load map failed %s", map_file_path.c_str());
            return;
        }

        RCLCPP_INFO(nh_->get_logger(), "Loc: map_cloud->size(): %zu", map_cloud->size());

        // 获取地图体素滤波 map_voxelgrid_filter_的尺寸
        voxel_leaf_size_ = config["map_filtr_leaf_size"].as<double>();

        map_voxelgrid_filter_.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
        pcl::PointCloud<PointType>::Ptr map_cloud_filter(new pcl::PointCloud<PointType>);
        map_cloud_filter->clear();

        map_voxelgrid_filter_.setInputCloud(map_cloud);

        map_voxelgrid_filter_.filter(*map_cloud_filter);
        RCLCPP_INFO(nh_->get_logger(), "Loc: map_cloud_filter->points.size(): %zu", map_cloud_filter->points.size());

        map_cloud_filter->width = map_cloud_filter->points.size();

        map_cloud_filter->height = 1;

        // 发布地图
        pcl::toROSMsg(*map_cloud_filter, pubmap_msg);
        if (pubmap_msg.width != 0)
        {
            RCLCPP_INFO(nh_->get_logger(), "Loc: map has been published");
            pubmap_msg.header.frame_id = map_frame_;
            pubmap_msg.header.stamp = nh_->get_clock()->now(); // 设置时间戳
            RCLCPP_INFO(nh_->get_logger(), "Publishing map with timestamp: %.9f", getSec(pubmap_msg.header));
            pc_map_pub_->publish(pubmap_msg);
        }
        else
        {
            RCLCPP_ERROR(nh_->get_logger(), " Loc :  can not publish map");
            return;
        }

        localizier3d.reset(new scan2map3d(map_cloud_filter, config_file_name)); // 载入滤波地图
        localizier3d->setNormalMode();

        // 初始化eskf
        std::string eskf_cfg_path;
        nh_->declare_parameter<std::string>("eskf_cfg_file", "none");
        nh_->get_parameter<std::string>("eskf_cfg_file", eskf_cfg_path);
        RCLCPP_INFO(nh_->get_logger(), "eskf_cfg_path: %s", eskf_cfg_path.c_str());
        eskf.reset(new ErrorStateKalmanFilter(eskf_cfg_path));

        std::string pose_pub_topic_ = config["pose_pub_topic"].as<std::string>();
        pose_pub_ = nh_->create_publisher<geometry_msgs::msg::PoseStamped>(pose_pub_topic_, 10); // 发布定位位姿话题数据
        RCLCPP_INFO(nh_->get_logger(), "pose_pub_topic_: %s", pose_pub_topic_.c_str());

        // 初始化处理后点云发布器
        std::string processed_cloud_topic = config["pub_cloud_topic"].as<std::string>();
        processed_cloud_pub_ = nh_->create_publisher<sensor_msgs::msg::PointCloud2>(processed_cloud_topic, 10);
        RCLCPP_INFO(nh_->get_logger(), "processed_cloud_topic: %s", processed_cloud_topic.c_str());

        // 初始化性能统计
        last_pose_publish_time_ = nh_->get_clock()->now();
        stats_start_time_ = nh_->get_clock()->now();

        // ========== 初始化去畸变配置 ==========
        // 从配置文件读取是否启用去畸变
        enable_undistort_ = config["enable_undistort"] ? config["enable_undistort"].as<bool>() : true;
        
        if (enable_undistort_) {
            RCLCPP_INFO(nh_->get_logger(), "[去畸变] 功能已启用 (已优化为 base_link 系下直接运动补偿)");
        } else {
            RCLCPP_WARN(nh_->get_logger(), "[去畸变] 配置文件已禁用去畸变功能");
        }

        // 创建定时器，每5秒输出一次性能统计
        stats_timer_ = nh_->create_wall_timer(
            std::chrono::seconds(5),
            std::bind(&location::print_performance_stats, this));

        // 启动独立处理线程(专注于同步+定位,点云处理在回调中完成)
        process_thread_ = std::thread(&location::processThreadFunc, this);
    }
    
    // 析构函数:正确关闭处理线程
    ~location()
    {
        RCLCPP_INFO(nh_->get_logger(), "[析构] 开始清理资源...");
        
        // 通知处理线程退出
        g_b_exit.store(true);
        process_cv_.notify_all();
        
        // 等待线程结束
        if (process_thread_.joinable()) {
            process_thread_.join();
            RCLCPP_INFO(nh_->get_logger(), "[析构] 处理线程已关闭");
        }
        
        RCLCPP_INFO(nh_->get_logger(), "[析构] 资源清理完成");
    }

    // 性能统计输出函数
    void print_performance_stats()
    {
        if (perf_history_.empty())
            return;

        // 计算平均值
        double avg_imu_predict = 0.0;
        double avg_registration = 0.0;
        double avg_eskf_correct = 0.0;
        double avg_pose_publish = 0.0;
        double avg_total = 0.0;
        double avg_imu_count = 0.0;

        for (const auto &stat : perf_history_)
        {
            avg_imu_predict += stat.imu_predict_time_ms;
            avg_registration += stat.cloud_registration_time_ms;
            avg_eskf_correct += stat.eskf_correct_time_ms;
            avg_pose_publish += stat.pose_publish_time_ms;
            avg_total += stat.total_process_time_ms;
            avg_imu_count += stat.imu_count;
        }

        size_t count = perf_history_.size();
        avg_imu_predict /= count;
        avg_registration /= count;
        avg_eskf_correct /= count;
        avg_pose_publish /= count;
        avg_total /= count;
        avg_imu_count /= count;

        // 计算位姿发布频率
        double elapsed_time = (nh_->get_clock()->now() - stats_start_time_).seconds();
        double pose_pub_freq = (elapsed_time > 0.0) ? (pose_publish_count_ / elapsed_time) : 0.0;

        // 获取最后一帧的数据
        const auto &last_stat = perf_history_.back();

        RCLCPP_INFO(nh_->get_logger(),
                    "\n========== 定位性能统计 (最近%zu帧) ==========\n"
                    "定位频率: %.2f Hz (总共 %d 次)\n"
                    "\n--- 1. 各模块平均耗时 ---\n"
                    "  IMU预积分:     %.2f ms (平均 %.1f 个)\n"
                    "  点云配准:       %.2f ms\n"
                    "  ESKF校正:      %.2f ms\n"
                    "  位姿发布:       %.2f ms\n"
                    "  总处理时间:     %.2f ms\n"
                    "\n--- 2. 最新一帧时间同步详情 ---\n"
                    "  同步状态:       %s\n"
                    "  雷达起始:       %.6f s\n"
                    "  雷达结束:       %.6f s (扫描时长 %.1f ms)\n"
                    "  同步后IMU:     %d 个 (%.6f ~ %.6f s)\n"
                    "  IMU与雷达:     首帧差 %+.1f ms | 末帧差 %+.1f ms\n"
                    "\n--- 同步前缓冲区状态 ---\n"
                    "  IMU缓冲区:     %zu 个 (%.6f ~ %.6f s)\n"
                    "  时间覆盖:       %.1f ms\n"
                    "\n--- 最新一帧耗时 ---\n"
                    "  IMU预积分:     %.2f ms\n"
                    "  点云配准:       %.2f ms\n"
                    "  ESKF校正:      %.2f ms\n"
                    "  位姿发布:       %.2f ms\n"
                    "  总处理:         %.2f ms\n"
                    "==========================================",
                    count, pose_pub_freq, pose_publish_count_,
                    avg_imu_predict, avg_imu_count, avg_registration, avg_eskf_correct, avg_pose_publish, avg_total,
                    last_stat.sync_success ? "成功✓" : "失败✗",
                    last_stat.lidar_start_time,
                    last_stat.lidar_end_time,
                    (last_stat.lidar_end_time - last_stat.lidar_start_time) * 1000.0,
                    last_stat.imu_count,
                    last_stat.imu_first_time,
                    last_stat.imu_last_time,
                    (last_stat.imu_first_time - last_stat.lidar_start_time) * 1000.0,
                    (last_stat.imu_last_time - last_stat.lidar_end_time) * 1000.0,
                    last_stat.imu_buffer_before_sync,
                    last_stat.imu_buffer_first_time,
                    last_stat.imu_buffer_last_time,
                    (last_stat.imu_buffer_last_time - last_stat.imu_buffer_first_time) * 1000.0,
                    last_stat.imu_predict_time_ms, last_stat.cloud_registration_time_ms,
                    last_stat.eskf_correct_time_ms, last_stat.pose_publish_time_ms, last_stat.total_process_time_ms);
    }

    // 发布前位姿重估：只影响发布链路，不回写 act_pose，避免干扰下一帧配准/ESKF
    bool refinePoseForPublish(const pose_type &raw_pose, const double &time, pose_type &refined_pose)
    {
        refined_pose = raw_pose;
        if (!pose_refine_enable_) {
            return false;
        }

        const rclcpp::Time stamp = getTime(time);
        geometry_msgs::msg::TransformStamped odom_to_base_msg;
        try {
            const auto timeout = rclcpp::Duration::from_seconds(pose_refine_tf_lookup_timeout_sec_);
            odom_to_base_msg = tf_buffer_->lookupTransform(odom_frame_, base_frame_, stamp, timeout);
        } catch (tf2::TransformException &ex) {
            // 回退到最新TF，尽量保证可发布
            try {
                odom_to_base_msg = tf_buffer_->lookupTransform(odom_frame_, base_frame_, tf2::TimePointZero);
            } catch (tf2::TransformException &ex_latest) {
                RCLCPP_WARN_THROTTLE(
                    nh_->get_logger(), *nh_->get_clock(), 2000,
                    "[PoseRefine] lookupTransform失败: %s | latest也失败: %s",
                    ex.what(), ex_latest.what());
                return false;
            }
        }

        tf2::Transform odom_to_base_tf;
        tf2::fromMsg(odom_to_base_msg.transform, odom_to_base_tf);

        tf2::Quaternion q_meas;
        q_meas.setRPY(raw_pose.orient(0), raw_pose.orient(1), raw_pose.orient(2));
        tf2::Transform map_to_base_meas;
        map_to_base_meas.setOrigin(tf2::Vector3(raw_pose.pos(0), raw_pose.pos(1), raw_pose.pos(2)));
        map_to_base_meas.setRotation(q_meas);

        if (!pose_refine_inited_) {
            pose_refine_map_to_base_prev_ = map_to_base_meas;
            pose_refine_odom_to_base_prev_ = odom_to_base_tf;
            pose_refine_last_stamp_ = stamp;
            pose_refine_inited_ = true;
            return true;
        }

        const double dt = (stamp - pose_refine_last_stamp_).seconds();
        if (dt <= 1e-4) {
            return true;
        }

        const tf2::Transform odom_delta = pose_refine_odom_to_base_prev_.inverse() * odom_to_base_tf;
        const tf2::Transform map_to_base_pred = pose_refine_map_to_base_prev_ * odom_delta;

        const double linear_speed = std::hypot(odom_delta.getOrigin().x(), odom_delta.getOrigin().y()) / dt;
        const double yaw_delta = normalize_angle(tf2::getYaw(odom_delta.getRotation()));
        const double yaw_rate = std::fabs(yaw_delta / dt);

        const double pred_x = map_to_base_pred.getOrigin().x();
        const double pred_y = map_to_base_pred.getOrigin().y();
        const double pred_yaw = tf2::getYaw(map_to_base_pred.getRotation());
        const double meas_x = map_to_base_meas.getOrigin().x();
        const double meas_y = map_to_base_meas.getOrigin().y();
        const double meas_yaw = tf2::getYaw(map_to_base_meas.getRotation());

        const double err_x = meas_x - pred_x;
        const double err_y = meas_y - pred_y;
        const double err_xy = std::hypot(err_x, err_y);
        const double err_yaw = normalize_angle(meas_yaw - pred_yaw);

        const bool is_static = linear_speed < pose_refine_static_v_th_ && yaw_rate < pose_refine_static_w_th_;
        const double k_xy = is_static ? pose_refine_k_xy_static_ : pose_refine_k_xy_move_;
        const double k_yaw = is_static ? pose_refine_k_yaw_static_ : pose_refine_k_yaw_move_;

        tf2::Transform map_to_base_refined = map_to_base_pred;
        const bool force_accept = pose_refine_force_accept_next_;
        pose_refine_force_accept_next_ = false;

        if (force_accept) {
            map_to_base_refined = map_to_base_meas;
            RCLCPP_WARN(
                nh_->get_logger(),
                "[PoseRefine] 重定位后强制接受测量位姿: err_xy=%.3f m, err_yaw=%.3f rad",
                err_xy, err_yaw);
        } else if (err_xy > pose_refine_gate_xy_m_ || std::fabs(err_yaw) > pose_refine_gate_yaw_rad_) {
            RCLCPP_WARN_THROTTLE(
                nh_->get_logger(), *nh_->get_clock(), 1000,
                "[PoseRefine] 位姿门控生效: err_xy=%.3f m, err_yaw=%.3f rad (gate=%.3f, %.3f)",
                err_xy, err_yaw, pose_refine_gate_xy_m_, pose_refine_gate_yaw_rad_);
        } else {
            const double out_x = pred_x + k_xy * err_x;
            const double out_y = pred_y + k_xy * err_y;
            const double out_yaw = normalize_angle(pred_yaw + k_yaw * err_yaw);
            tf2::Quaternion q_refined;
            q_refined.setRPY(raw_pose.orient(0), raw_pose.orient(1), out_yaw);
            map_to_base_refined.setOrigin(tf2::Vector3(out_x, out_y, raw_pose.pos(2)));
            map_to_base_refined.setRotation(q_refined);
        }

        // 回填发布位姿（保持 z/roll/pitch 使用原始解算结果，重点抑制 x/y/yaw 抖动）
        refined_pose.pos(0) = map_to_base_refined.getOrigin().x();
        refined_pose.pos(1) = map_to_base_refined.getOrigin().y();
        refined_pose.pos(2) = raw_pose.pos(2);
        refined_pose.orient(0) = raw_pose.orient(0);
        refined_pose.orient(1) = raw_pose.orient(1);
        refined_pose.orient(2) = normalize_angle(tf2::getYaw(map_to_base_refined.getRotation()));
        update_q(refined_pose);

        pose_refine_map_to_base_prev_ = map_to_base_refined;
        pose_refine_odom_to_base_prev_ = odom_to_base_tf;
        pose_refine_last_stamp_ = stamp;
        return true;
    }

    // 发布点云数据到指定话题（参考lio_node.cpp的publishCloud函数）
    void publishCloud(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub,
                      pcl::PointCloud<PointType>::Ptr cloud,
                      const std::string &frame_id,
                      const double &time)
    {
        // 如果没有订阅者则不发布，节省计算资源
        if (pub->get_subscription_count() <= 0)
            return;

        // 将PCL点云转换为ROS消息格式
        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(*cloud, cloud_msg);

        // 设置消息头信息
        cloud_msg.header.frame_id = frame_id;   // 坐标系名称
        cloud_msg.header.stamp = getTime(time); // 时间戳

        // 发布点云消息
        pub->publish(cloud_msg);
    }

    // 发布位姿信息（参考lio_node.cpp的发布模式）
    void publishPose(const pose_type &pose, const std::string &frame_id, const double &time)
    {
        // 如果没有订阅者则不发布
        if (pose_pub_->get_subscription_count() <= 0)
            return;

        // 创建位姿消息
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.frame_id = frame_id;
        pose_msg.header.stamp = getTime(time);

        // 设置位置信息
        pose_msg.pose.position.x = pose.pos(0);
        pose_msg.pose.position.y = pose.pos(1);
        pose_msg.pose.position.z = pose.pos(2);

        // 设置姿态信息（从欧拉角转换为四元数）
        tf2::Quaternion q;
        q.setRPY(pose.orient(0), pose.orient(1), pose.orient(2));
        pose_msg.pose.orientation.x = q.x();
        pose_msg.pose.orientation.y = q.y();
        pose_msg.pose.orientation.z = q.z();
        pose_msg.pose.orientation.w = q.w();

        // 发布位姿消息
        pose_pub_->publish(pose_msg);
    }

    // 广播TF变换信息（支持 map→odom 模式用于 Nav2 导航）
    void broadCastTF(const pose_type &pose, const std::string &frame_id, const std::string &child_frame, const double &time)
    {
        // 构建 map → base_footprint 的变换
        tf2::Transform map_to_base_tf;
        map_to_base_tf.setOrigin(tf2::Vector3(pose.pos(0), pose.pos(1), pose.pos(2)));
        tf2::Quaternion q;
        q.setRPY(pose.orient(0), pose.orient(1), pose.orient(2));
        map_to_base_tf.setRotation(q);

        rclcpp::Time stamp = getTime(time);

        if (!enable_map_odom_tf_) {
            // 禁用 map→odom 模式：直接发布 map → base_footprint
            geometry_msgs::msg::TransformStamped transformStamped;
            transformStamped.header.frame_id = frame_id;
            transformStamped.child_frame_id = child_frame;
            transformStamped.header.stamp = stamp;
            transformStamped.transform.translation.x = pose.pos(0);
            transformStamped.transform.translation.y = pose.pos(1);
            transformStamped.transform.translation.z = pose.pos(2);
            transformStamped.transform.rotation.x = q.x();
            transformStamped.transform.rotation.y = q.y();
            transformStamped.transform.rotation.z = q.z();
            transformStamped.transform.rotation.w = q.w();
            tf_broadcaster_->sendTransform(transformStamped);
        } else {
            // 启用 map→odom 模式：计算并发布 map → odom
            // 公式: map→odom = (map→base_footprint) * (odom→base_footprint)^(-1)
            try {
                // 监听 odom → base_footprint 变换
                geometry_msgs::msg::TransformStamped odom_to_base_msg = 
                    tf_buffer_->lookupTransform(odom_frame_, base_frame_, tf2::TimePointZero);
                
                tf2::Transform odom_to_base_tf;
                tf2::fromMsg(odom_to_base_msg.transform, odom_to_base_tf);
                
                // 计算 map → odom = (map→base) * (odom→base)^(-1)
                tf2::Transform map_to_odom_tf = map_to_base_tf * odom_to_base_tf.inverse();
                last_map_to_odom_tf_ = map_to_odom_tf;
                has_last_map_to_odom_tf_ = true;
                
                // 发布 map → odom
                geometry_msgs::msg::TransformStamped map_to_odom_stamped;
                map_to_odom_stamped.header.stamp = stamp;
                map_to_odom_stamped.header.frame_id = map_frame_;
                map_to_odom_stamped.child_frame_id = odom_frame_;
                map_to_odom_stamped.transform = tf2::toMsg(map_to_odom_tf);
                tf_broadcaster_->sendTransform(map_to_odom_stamped);
                
            } catch (tf2::TransformException &ex) {
                static int warn_count = 0;
                if (warn_count++ % 50 == 0) {  // 每 50 次只警告一次
                    RCLCPP_WARN(nh_->get_logger(), 
                        "[TF] 无法获取 %s→%s 变换: %s",
                        odom_frame_.c_str(), base_frame_.c_str(), ex.what());
                }

                // 不要发布 map->base_footprint（会导致 TF 出现两棵树，Nav2/rviz 更难用）。
                // 若之前成功发布过 map->odom，则继续重发上一帧结果以保持 TF 连续性。
                if (has_last_map_to_odom_tf_) {
                    geometry_msgs::msg::TransformStamped map_to_odom_stamped;
                    map_to_odom_stamped.header.stamp = stamp;
                    map_to_odom_stamped.header.frame_id = map_frame_;
                    map_to_odom_stamped.child_frame_id = odom_frame_;
                    map_to_odom_stamped.transform = tf2::toMsg(last_map_to_odom_tf_);
                    tf_broadcaster_->sendTransform(map_to_odom_stamped);
                }
            }
        }
    }

    // IMU回调函数（完全对齐lio_node.cpp的逻辑）
    void imu_cb(const sensor_msgs::msg::Imu::SharedPtr imu_msg)
    {
        // 【锁前】记录回调开始时间
        double sys_time_before_lock = nh_->get_clock()->now().seconds();
        double msg_timestamp = getSec(imu_msg->header);
        double delay_before_lock = sys_time_before_lock - msg_timestamp;
        
        // 时间戳诊断打印（每200帧打印一次）
        static int imu_diag_count = 0;
        bool should_print = (++imu_diag_count % 200 == 0);
        
        if (should_print)
        {
            RCLCPP_INFO(nh_->get_logger(), 
                       "[IMU-锁前] 消息时间=%.6f | 系统时间=%.6f | 延迟=%.3fs(%.0fms)",
                       msg_timestamp, sys_time_before_lock, delay_before_lock, delay_before_lock * 1000.0);
        }
        
        // 加锁保护IMU缓冲区
        std::lock_guard<std::mutex> lock(m_state_data.imu_mutex);
        
        // // 【锁后】记录获取锁后的时间
        // double sys_time_after_lock = nh_->get_clock()->now().seconds();
        // double lock_wait_time = sys_time_after_lock - sys_time_before_lock;
        // double delay_after_lock = sys_time_after_lock - msg_timestamp;
        
        // if (should_print)
        // {
        //     RCLCPP_INFO(nh_->get_logger(), 
        //                "[IMU-锁后] 系统时间=%.6f | 等锁耗时=%.1fms | 总延迟=%.3fs(%.0fms)",
        //                sys_time_after_lock, lock_wait_time * 1000.0, 
        //                delay_after_lock, delay_after_lock * 1000.0);
        // }
        
        double timestamp = msg_timestamp; // 使用消息时间戳

        // 检测时间戳乱序（与lio_node一致）
        if (timestamp < m_state_data.last_imu_time)
        {
            RCLCPP_WARN(nh_->get_logger(), "IMU Message is out of order");
            std::deque<IMUData>().swap(m_state_data.imu_buffer);
        }

        // 第1步：读取原始IMU数据（与lio_node一致使用Vector3d）
        Eigen::Vector3d acc(imu_msg->linear_acceleration.x,
                            imu_msg->linear_acceleration.y,
                            imu_msg->linear_acceleration.z);
        Eigen::Vector3d gyro(imu_msg->angular_velocity.x,
                             imu_msg->angular_velocity.y,
                             imu_msg->angular_velocity.z);

        // 第2步：坐标系旋转到base_link（与lio_node一致）
        acc = imu2link_rot_.cast<double>() * acc;
        gyro = imu2link_rot_.cast<double>() * gyro;

        // 第3步：先处理单位和特殊雷达方向（统一转换到 m/s^2）
        if (acc.z() > 5.0) {
            // 输入已经是 m/s^2，不需要操作
        } else if (acc.z() < 0) {
            // 特殊处理：速腾airy 96线雷达
            acc = Eigen::Vector3d(-acc.y(), -acc.x(), -acc.z());
            gyro = Eigen::Vector3d(-gyro.y(), -gyro.x(), -gyro.z());
            if (std::abs(acc.z()) < 5.0) acc = acc * imu_scale_; // 如果是g，转m/s^2
        } else {
            // -5 < z < 5，说明输入单位是 g，需要转成 m/s^2
            acc = acc * imu_scale_;
        }

        // 第4步：现在 acc 的单位已经是 m/s^2，进行向心加速度补偿
        Eigen::Vector3d p_imu = imu2base_pose.pos.cast<double>();
        Eigen::Vector3d centripetal_acc = gyro.cross(gyro.cross(p_imu));
        Eigen::Vector3d final_acc = acc - centripetal_acc; // 量纲统一，相减合法

        // 添加到缓冲区（与lio_node一致）
        m_state_data.imu_buffer.emplace_back(final_acc, gyro, timestamp);

        // 更新最新的IMU时间戳
        m_state_data.last_imu_time = timestamp;
        
        // // 【结束时】记录回调结束时间
        // double sys_time_end = nh_->get_clock()->now().seconds();
        // double process_time = sys_time_end - sys_time_after_lock;
        // double total_delay = sys_time_end - msg_timestamp;
        
        // if (should_print)
        // {
        //     RCLCPP_INFO(nh_->get_logger(), 
        //                "[IMU-结束] 系统时间=%.6f | 处理耗时=%.1fms | 总延迟=%.3fs(%.0fms) | 缓冲区=%zu个",
        //                sys_time_end, process_time * 1000.0, 
        //                total_delay, total_delay * 1000.0,
        //                m_state_data.imu_buffer.size());
        // }
    }

    // 激光点云回调 - 最优版:直接处理点云,然后唤醒定位线程
    void robosense_cb(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
    {
        // 获取时间戳
        double msg_timestamp = getSec(cloud_msg->header);
        double sys_timestamp = nh_->get_clock()->now().seconds();
        double time_diff = sys_timestamp - msg_timestamp;
        
        // 时间戳诊断打印（每一帧都打印，LiDAR频率低）
        RCLCPP_INFO(nh_->get_logger(), 
                   "[LiDAR回调] 消息时间=%.6f | 系统时间=%.6f | 差值=%.3fs(%.0fms)",
                   msg_timestamp, sys_timestamp, time_diff, time_diff * 1000.0);
        
        // === 步骤1: 点云处理(在回调线程中,不阻塞IMU因为是单线程executor) ===
        auto process_start = std::chrono::high_resolution_clock::now();
        
        pcl::PointCloud<PointType>::Ptr processed_cloud;
        if (lidar_type_ == "livox") {
            processed_cloud = livox_processor_->livox_cloud_handler(cloud_msg);
        } else if (lidar_type_ == "rs16") {
            processed_cloud = rs16_processor_->rs16_handler(cloud_msg);
        } else {
            RCLCPP_ERROR(nh_->get_logger(), "[LiDAR回调] 未知雷达类型: %s", lidar_type_.c_str());
            return;
        }
        if (processed_cloud == nullptr || processed_cloud->points.empty()) {
            return;
        }
        auto process_end = std::chrono::high_resolution_clock::now();
        double process_time_ms = std::chrono::duration<double, std::milli>(
            process_end - process_start).count();
        
        // === 步骤2: 加入同步队列(锁粒度小) ===
        {
            std::lock_guard<std::mutex> lock(m_state_data.lidar_mutex);
            
            // 检测时间戳乱序
            if (msg_timestamp < m_state_data.last_lidar_time) {
                RCLCPP_WARN(nh_->get_logger(), "[LiDAR回调] 消息乱序,清空缓冲");
                std::deque<std::pair<double, pcl::PointCloud<PointType>::Ptr>>().swap(
                    m_state_data.lidar_buffer);
                    m_state_data.lidar_pushed = false; // 乱序时也必须重置标志位
            }
            
            m_state_data.lidar_buffer.emplace_back(msg_timestamp, processed_cloud);
            m_state_data.last_lidar_time = msg_timestamp;
            
            // 内部防积压机制: 如果积压超过2帧，强制丢弃历史帧，保留最新一帧，重置同步标志
            if (m_state_data.lidar_buffer.size() > 2) {
                // 1. 备份最新的一帧
                auto latest_cloud = m_state_data.lidar_buffer.back(); 
                // 2. 清空所有积压历史帧
                m_state_data.lidar_buffer.clear();                    
                // 3. 把最新的一帧放回去
                m_state_data.lidar_buffer.push_back(latest_cloud);    
                
                // 4. 重置同步标志，强迫 syncPackage 放弃老帧，重新打包这帧最新数据！
                m_state_data.lidar_pushed = false;
            }

            // 记录处理时间用于诊断
            RCLCPP_DEBUG(nh_->get_logger(), "[LiDAR回调] 点云处理耗时=%.1fms", process_time_ms);
        }
        
        // === 步骤3: 唤醒处理线程尝试同步 ===
        // 注意: 处理线程会在wait_for中调用syncPackage()判断是否真的能同步
        process_cv_.notify_one();
    }

    // 同步IMU和LiDAR数据包（完全对齐lio_node.cpp的逻辑）
    bool syncPackage()
    {
        // 保护队列不被底层回调线程（如 clear操作）同时修改，彻底杜绝 -11 崩溃！
        std::scoped_lock lock(m_state_data.imu_mutex, m_state_data.lidar_mutex);

        // 如果IMU缓冲区或LiDAR缓冲区为空，无法同步
        if (m_state_data.imu_buffer.empty() || m_state_data.lidar_buffer.empty())
            return false;

        // 如果还未处理当前帧雷达数据
        if (!m_state_data.lidar_pushed)
        {
            // 取出当前帧雷达点云（现在有锁保护，绝对安全！）
            m_package.cloud = m_state_data.lidar_buffer.front().second;

            // 按照点的曲率（curvature）排序
            std::sort(m_package.cloud->points.begin(), m_package.cloud->points.end(),
                      [](PointType &p1, PointType &p2)
                      { return p1.curvature < p2.curvature; });

            // 设置当前点云开始时间
            m_package.cloud_start_time = m_state_data.lidar_buffer.front().first;

            // 设置当前点云的结束时间
            if (m_package.cloud->points.empty()) {
                m_state_data.lidar_buffer.pop_front();
                m_state_data.lidar_pushed = false;
                return false; 
            }
            m_package.cloud_end_time = m_package.cloud_start_time + m_package.cloud->points.back().curvature / 1000.0;

            // 标记lidar已经处理
            m_state_data.lidar_pushed = true;
        }

        // 如果最新的IMU时间还没有覆盖到当前点云的结束时间，则等待更多的IMU数据
        if (m_state_data.last_imu_time < m_package.cloud_end_time)
            return false;

        // 清空上一帧打包的IMU数据
        std::vector<IMUData>().swap(m_package.imus);

        // 将所有时间 < 点云结束时间的IMU数据打包进m_package.imus
        while (!m_state_data.imu_buffer.empty() &&
               m_state_data.imu_buffer.front().time < m_package.cloud_end_time)
        {
            m_package.imus.emplace_back(m_state_data.imu_buffer.front());
            m_state_data.imu_buffer.pop_front(); 
        }

        // 移除已处理的雷达数据帧
        m_state_data.lidar_buffer.pop_front();
        m_state_data.lidar_pushed = false;

        // 数据包同步成功
        return true;
    }

    // 初始位 姿反馈
    void initpose_cb(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose_msg)
    {
        RCLCPP_INFO(nh_->get_logger(), "获取到全局重定位初始位姿!");
        std::lock_guard<std::mutex> lock(global_pose_mutex_);
        
        global_pose_buffer_.pos(0) = pose_msg->pose.pose.position.x;
        global_pose_buffer_.pos(1) = pose_msg->pose.pose.position.y;
        global_pose_buffer_.pos(2) = 0.0; // 强制Z=0

        double siny_cosp = 2.0 * pose_msg->pose.pose.orientation.w * pose_msg->pose.pose.orientation.z;
        double cosy_cosp = 1.0 - 2.0 * pose_msg->pose.pose.orientation.z * pose_msg->pose.pose.orientation.z;
        
        global_pose_buffer_.orient(0) = 0.0; // 强制Roll=0
        global_pose_buffer_.orient(1) = 0.0; // 强制Pitch=0
        global_pose_buffer_.orient(2) = atan2(siny_cosp, cosy_cosp);
        global_pose_buffer_.vel << 0.0, 0.0, 0.0;

        has_new_global_pose_.store(true);
    }

    // 状态命 令反馈
    void command_cb(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(nh_->get_logger(), "cmd: %s", msg->data.c_str());
        if (msg->data == "shutdown")
        {
            rclcpp::shutdown();
        }
        else if (msg->data == "test_reloc")
        {
            RCLCPP_INFO(nh_->get_logger(), "test reloc");
            
            std::lock_guard<std::mutex> lock(global_pose_mutex_);
            global_pose_buffer_.pos(0) = act_pose.pos(0) + 5.0;
            global_pose_buffer_.pos(1) = 4.0;
            global_pose_buffer_.pos(2) = 0.0;
            global_pose_buffer_.orient = act_pose.orient;  // 保持朝向
            global_pose_buffer_.vel << 0.0, 0.0, 0.0;
            
            has_new_global_pose_.store(true);  // 触发统一的重定位流程
        }
    }


    void imu_predict(const std::vector<IMUData> &imus, PerformanceStats &perf_stat)
    {
        if (imus.empty())
        {
            perf_stat.imu_predict_time_ms = 0.0;
            return;
        }

        auto imu_predict_start = std::chrono::high_resolution_clock::now();

        for (const auto &imu : imus)
        {
            Eigen::VectorXd imu_data(6);
            imu_data << imu.acc, imu.gyro;

            if (check_imu_data(imu_data))
            {
                eskf->predict(imu_data, imu.time);
                eskf->getPose(act_pose);
            }
        }

        auto imu_predict_end = std::chrono::high_resolution_clock::now();
        perf_stat.imu_predict_time_ms = std::chrono::duration<double, std::milli>(imu_predict_end - imu_predict_start).count();
    }

    // ============ 去畸变相关函数 ============

    // IMU插值函数（参考gyr_int.cpp）
    IMUData interpolateImu(const IMUData &imu1, const IMUData &imu2, double t)
    {
        if (std::abs(imu2.time - imu1.time) < 1e-9)
        {
            return imu1;
        }

        double ratio = (t - imu1.time) / (imu2.time - imu1.time);
        double w1 = 1.0 - ratio;
        double w2 = ratio;

        IMUData imu_inter;
        imu_inter.acc = w1 * imu1.acc + w2 * imu2.acc;
        imu_inter.gyro = w1 * imu1.gyro + w2 * imu2.gyro;
        imu_inter.time = t;

        return imu_inter;
    }

    // 角速度积分函数（参考gyr_int.cpp的Integrate函数）
    Sophus::SO3d integrateRotation(double start_time, double end_time,
                                   const std::vector<IMUData> &v_imu)
    {
        Sophus::SO3d result_rot; // 初始化为单位旋转

        if (v_imu.empty() || start_time >= end_time)
        {
            return result_rot;
        }

        // 找到第一个时间大于等于start_time的IMU数据
        size_t start_idx = 0;
        for (size_t i = 0; i < v_imu.size(); ++i)
        {
            if (v_imu[i].time >= start_time)
            {
                start_idx = (i > 0) ? i - 1 : 0;
                break;
            }
        }

        // 找到最后一个时间小于等于end_time的IMU数据
        size_t end_idx = v_imu.size() - 1;
        for (size_t i = v_imu.size(); i > 0; --i)
        {
            if (v_imu[i - 1].time <= end_time)
            {
                end_idx = i - 1;
                break;
            }
        }

        if (start_idx >= v_imu.size() || end_idx >= v_imu.size() || start_idx > end_idx)
        {
            return result_rot;
        }

        // 在start_time处插值
        IMUData current_imu;
        double current_time = start_time;

        if (start_idx == 0)
        {
            current_imu = v_imu[0];
        }
        else
        {
            current_imu = interpolateImu(v_imu[start_idx - 1], v_imu[start_idx], start_time);
        }

        // 开始积分
        for (size_t i = start_idx; i <= end_idx; ++i)
        {
            double next_time = v_imu[i].time;

            if (next_time > end_time)
            {
                // 在end_time处插值并处理最后一个区间
                double prev_time = (i > 0) ? v_imu[i - 1].time : start_time;
                IMUData end_imu = interpolateImu(v_imu[i - 1], v_imu[i], end_time);

                // 积分最后一个区间
                double dt = end_time - current_time;
                Eigen::Vector3d delta_angle = dt * 0.5 * (current_imu.gyro + end_imu.gyro);
                Sophus::SO3d delta_r = Sophus::SO3d::exp(delta_angle);
                result_rot = result_rot * delta_r;

                break;
            }
            else
            {
                // 积分当前区间
                double dt = next_time - current_time;
                Eigen::Vector3d delta_angle = dt * 0.5 * (current_imu.gyro + v_imu[i].gyro);
                Sophus::SO3d delta_r = Sophus::SO3d::exp(delta_angle);
                result_rot = result_rot * delta_r;

                // 更新当前状态
                current_time = next_time;
                current_imu = v_imu[i];
            }
        }

        // 处理end_time在所有IMU数据之后的情况
        if (end_time > v_imu.back().time)
        {
            double dt = end_time - current_time;
            Eigen::Vector3d delta_angle = dt * current_imu.gyro;
            Sophus::SO3d delta_r = Sophus::SO3d::exp(delta_angle);
            result_rot = result_rot * delta_r;
        }

        return result_rot;
    }

    // 点云去畸变函数（参考data_process.cpp的UndistortPcl）
    void undistortPointCloud(pcl::PointCloud<PointType>::Ptr &cloud,
                             double start_time, double end_time,
                             const std::vector<IMUData> &imus)
    {
        if (!enable_undistort_ || imus.empty() || cloud->empty())
        {
            return;
        }

        auto undistort_start = std::chrono::high_resolution_clock::now();

        // 积分IMU数据，计算从扫描开始到结束的旋转
        Sophus::SO3d rot_base_be = integrateRotation(start_time, end_time, imus);

        // 因为点云也已经在 base_link 下了，根本不需要任何外参变换！直接使用！
        Eigen::Vector3d rso3_be = rot_base_be.log();
        Eigen::Vector3d tbe = Eigen::Vector3d::Zero(); // 平移忽略不计
        double dt_be = end_time - start_time;

        if (dt_be <= 0)
        {
            RCLCPP_ERROR(nh_->get_logger(), "[去畸变] 无效的时间跨度: start=%.6f, end=%.6f",
                         start_time, end_time);
            return;
        }

        // 对每个点进行去畸变
        for (auto &pt : cloud->points)
        {
            // curvature字段存储相对时间（毫秒）
            double dt_bi = pt.curvature / 1000.0; // 转换为秒

            if (dt_bi < 0 || dt_bi > dt_be)
            {
                // 时间戳异常，使用均匀分布
                continue;
            }

            double ratio_bi = dt_bi / dt_be;
            double ratio_ie = 1.0 - ratio_bi;

            // 计算从当前点时刻到扫描结束的旋转和平移
            Eigen::Vector3d rso3_ie = ratio_ie * rso3_be;
            Sophus::SO3d Rie = Sophus::SO3d::exp(rso3_ie);
            Eigen::Vector3d tie = ratio_ie * tbe;

            // 运动补偿：将点从当前时刻变换到扫描结束时刻
            Eigen::Vector3d v_pt_i(pt.x, pt.y, pt.z);
            Eigen::Vector3d v_pt_comp_e = Rie.inverse() * (v_pt_i - tie);

            // 更新点坐标
            pt.x = v_pt_comp_e.x();
            pt.y = v_pt_comp_e.y();
            pt.z = v_pt_comp_e.z();
        }

        auto undistort_end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(undistort_end - undistort_start);

        // 每一帧都打印去畸变信息
        RCLCPP_INFO(nh_->get_logger(),
                    "[去畸变] 点云数=%zu | IMU数=%zu | 耗时=%.2fms | 旋转角度=(%.2f°, %.2f°, %.2f°) | 时间跨度=%.1fms",
                    cloud->size(), imus.size(), duration.count() / 1000.0,
                    rot_base_be.angleX() * 180.0 / M_PI,
                    rot_base_be.angleY() * 180.0 / M_PI,
                    rot_base_be.angleZ() * 180.0 / M_PI,
                    (end_time - start_time) * 1000.0);
    }

    // ============ 独立处理线程函数 ============
        // ============ 独立处理线程函数 ============
    void processThreadFunc()
    {
        RCLCPP_INFO(nh_->get_logger(), "[处理线程] 启动,使用同步触发模式");
        
        int process_count = 0;
        
        while (rclcpp::ok() && !g_b_exit.load())
        {
            // === 等待同步成功 ===
            {
                std::unique_lock<std::mutex> lock(process_mutex_);
                
                if (!process_cv_.wait_for(lock, std::chrono::milliseconds(10),
                    [this]() { 
                        if (g_b_exit.load()) return true;
                        return syncPackage();
                    }))
                {
                    continue;
                }
                
                if (g_b_exit.load()) {
                    RCLCPP_INFO(nh_->get_logger(), "[处理线程] 收到退出信号");
                    break;
                }
            }
            
            // ========== 初始位姿注入（ICP 或 RViz）==========
            if (has_new_global_pose_.load()) {
                std::lock_guard<std::mutex> lock(global_pose_mutex_);
                
                act_pose = global_pose_buffer_;
                
                eskf->setMean(act_pose.pos.cast<double>(), 
                             act_pose.orient.cast<double>(), 
                             act_pose.vel.cast<double>());
                eskf->reset();
                
                has_new_global_pose_.store(false);
                use_reloc_ = false; 
                localizier3d->setNormalMode();
                
                RCLCPP_WARN(nh_->get_logger(), 
                    "[重定位] 接收初始位姿: (%.2f, %.2f, %.2f°), 等待下一帧...", 
                    act_pose.pos(0), act_pose.pos(1), 
                    act_pose.orient(2) * 57.3);

                // 清空数据，等待下一帧点云开始正常追踪
                std::vector<IMUData>().swap(m_package.imus);
                m_package.cloud.reset();
                continue;  // ✅ 关键修复
            }

            // ========== 正常定位流程 ==========
            PerformanceStats perf_stat;
            auto frame_start = std::chrono::high_resolution_clock::now();
            
            pcl::PointCloud<PointType>::Ptr cloud_in = m_package.cloud;
            bool should_publish = false;
            
            // 重定位模式
            if (use_reloc_) {
                auto reg_start = std::chrono::high_resolution_clock::now();
                act_pose = localizier3d->location(act_pose, cloud_in);
                auto reg_end = std::chrono::high_resolution_clock::now();
                perf_stat.cloud_registration_time_ms = 
                    std::chrono::duration<double, std::milli>(reg_end - reg_start).count();
                
                // 重定位成功判断（ fitness 高 = 匹配好）
                if (localizier3d->fitness_ > (reloc_threshold_ * 1.2)) {
                    RCLCPP_WARN(nh_->get_logger(), 
                        "[重定位] 成功: fitness=%.3f, pos=(%.2f, %.2f)", 
                        localizier3d->fitness_, act_pose.pos(0), act_pose.pos(1));
                    
                    use_reloc_ = false;
                    localizier3d->setNormalMode();
                    should_publish = true;
                    
                    if (pose_refine_force_accept_reloc_) {
                        pose_refine_force_accept_next_ = true;
                    }
                    
                    // 重置 ESKF
                    auto eskf_start = std::chrono::high_resolution_clock::now();
                    eskf->setMean(act_pose.pos.cast<double>(), 
                                 act_pose.orient.cast<double>(), 
                                 act_pose.vel.cast<double>());
                    eskf->reset();
                    auto eskf_end = std::chrono::high_resolution_clock::now();
                    perf_stat.eskf_correct_time_ms = 
                        std::chrono::duration<double, std::milli>(eskf_end - eskf_start).count();
                } else {
                    RCLCPP_WARN_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 2000,
                        "[重定位] 进行中: fitness=%.3f (阈值=%.3f)",
                        localizier3d->fitness_, reloc_threshold_ * 1.2);
                }
            } 
            // 正常追踪模式
            else {
                // 去畸变
                if (!m_package.imus.empty()) {
                    undistortPointCloud(cloud_in, m_package.cloud_start_time,
                                       m_package.cloud_end_time, m_package.imus);
                }
                
                // IMU预测
                if (!m_package.imus.empty()) {
                    imu_predict(m_package.imus, perf_stat);
                }
                
                // 点云配准
                auto reg_start = std::chrono::high_resolution_clock::now();
                act_pose = localizier3d->location(act_pose, cloud_in);
                auto reg_end = std::chrono::high_resolution_clock::now();
                perf_stat.cloud_registration_time_ms = 
                    std::chrono::duration<double, std::milli>(reg_end - reg_start).count();

                // 强制 2D 约束
                act_pose.pos(2) = 0.0;
                act_pose.orient(0) = 0.0;
                act_pose.orient(1) = 0.0;

                // ESKF校正
                if (!m_package.imus.empty()) {
                    auto eskf_start = std::chrono::high_resolution_clock::now();
                    eskf->correct(act_pose);
                    auto eskf_end = std::chrono::high_resolution_clock::now();
                    perf_stat.eskf_correct_time_ms = 
                        std::chrono::duration<double, std::milli>(eskf_end - eskf_start).count();
                }
                
                // 检查是否丢失
                if (localizier3d->fitness_ < reloc_threshold_) {
                    RCLCPP_WARN(nh_->get_logger(), 
                        "[定位] Fitness过低 (%.3f < %.3f)，切换重定位",
                        localizier3d->fitness_, reloc_threshold_);
                    use_reloc_ = true;
                    localizier3d->setRelocMode();
                    should_publish = false;
                } else {
                    should_publish = true;
                }
            }
            
            // 统计和发布
            auto frame_end = std::chrono::high_resolution_clock::now();
            perf_stat.total_process_time_ms = 
                std::chrono::duration<double, std::milli>(frame_end - frame_start).count();
            
            if (should_publish) {
                if (std::isnan(act_pose.pos(0)) || std::isnan(act_pose.pos(1))) {
                    RCLCPP_ERROR(nh_->get_logger(), "[处理线程] NaN位姿,跳过发布");
                } else {
                    auto pub_start = std::chrono::high_resolution_clock::now();
                    
                    pose_publish_count_++;
                    pose_type act_pose_pub = act_pose;
                    refinePoseForPublish(act_pose, m_package.cloud_end_time, act_pose_pub);

                    publishPose(act_pose_pub, map_frame_, m_package.cloud_end_time);
                    broadCastTF(act_pose_pub, map_frame_, base_frame_, m_package.cloud_end_time);
                    publishCloud(processed_cloud_pub_, cloud_in, base_frame_, 
                                m_package.cloud_end_time);
                    
                    auto pub_end = std::chrono::high_resolution_clock::now();
                    perf_stat.pose_publish_time_ms = 
                        std::chrono::duration<double, std::milli>(pub_end - pub_start).count();
                }
            }
            
            // 更新统计
            perf_history_.push_back(perf_stat);
            if (perf_history_.size() > perf_history_size_) {
                perf_history_.pop_front();
            }
            
            // 清理
            std::vector<IMUData>().swap(m_package.imus);
            m_package.cloud.reset();
            
            ++process_count;
        }
        
        RCLCPP_INFO(nh_->get_logger(), "[处理线程] 退出");
    }
};
int main(int argc, char **argv)
{
    // 初始化ROS2客户端库
    rclcpp::init(argc, argv);
    
    // 注册信号处理函数(Ctrl+C优雅退出)
    signal(SIGINT, SigHandle);
    
    RCLCPP_INFO(rclcpp::get_logger("lidar_location_3d"), 
               "\033[1;32m---->\033[0m Lidar Location 3D Started (多线程+数据驱动模式)");
    
    // 创建节点
    auto nh = rclcpp::Node::make_shared("lidar_location_3d");
    
    // 创建location对象(自动初始化订阅器、发布器和处理线程)
    location loc(nh);

    // 线程数=2: 一个处理IMU(高频200Hz),一个处理LiDAR(低频10Hz)，使用多线程executor,允许IMU和LiDAR回调并发执行
    rclcpp::executors::MultiThreadedExecutor executor(
        rclcpp::ExecutorOptions(), 
        2  // 2个工作线程足够(IMU+LiDAR)
    );
    executor.add_node(nh);
    executor.spin();


    // 使用单线程executor(回调顺序执行,无锁竞争)
    // rclcpp::spin(nh);
    
    RCLCPP_INFO(rclcpp::get_logger("lidar_location_3d"), "正在关闭...");
    
    // 关闭ROS2节点
    rclcpp::shutdown();
    
    RCLCPP_INFO(rclcpp::get_logger("lidar_location_3d"), "已安全退出");
    return 0;
}
