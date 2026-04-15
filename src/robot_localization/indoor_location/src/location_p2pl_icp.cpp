#include "p2pl_icp.h"
#include "eskf.h"
#include "cloud_process_livox.h"  // Livox 点云处理类
#include "cloud_process_rs.h"     // RS16 点云处理类
#include "utility/utility.h"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

// TF2 ROS 接口（广播器和监听器）
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

// 几何消息与 TF2 的转换工具（注意 .h → .hpp）
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <std_msgs/msg/int8.hpp>

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

// 全局退出标志，用于信号处理
std::atomic<bool> g_b_exit(false);

// 信号处理函数
void SigHandle(int sig) {
    g_b_exit.store(true);
    RCLCPP_WARN(rclcpp::get_logger("lidar_location_3d"), "捕获信号 %d, 准备退出...", sig);
}

bool check_imu_data(Eigen::VectorXd imu_data) {
    bool is_ok = true;
    if (std::abs(imu_data(0)) > 10000.0) is_ok = false;
    if (std::abs(imu_data(1)) > 10000.0) is_ok = false;
    if (std::abs(imu_data(2)) > 10000.0) is_ok = false;
    if (std::abs(imu_data(3)) > 10000.0) is_ok = false;
    if (std::abs(imu_data(4)) > 10000.0) is_ok = false;
    if (std::abs(imu_data(5)) > 10000.0) is_ok = false;
    return is_ok;
}

class location {
public:
    // ROS2 节点对象
    rclcpp::Node::SharedPtr nh_ = std::make_shared<rclcpp::Node>("lidar_location_3d");
    
    // 地图相关
    std::string                     map_file_path;
    pcl::PointCloud<PointType>::Ptr map_cloud;
    
    // 位姿
    pose_type act_pose;
    pose_type init_pose_;
    pose_type imu2base_pose;

    std::mutex reloc_pose_mutex_;       // 保护外部位姿的互斥锁
    bool       has_new_global_pose_ = false;  // 是否收到新位姿的标志位
    pose_type  target_global_pose_;     // 缓存的新位姿

    // ROS2 中定义消息
    sensor_msgs::msg::PointCloud2 pubmap_msg;  // 作为消息播放的地图

    // 发布者
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr      pc_map_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr      processed_cloud_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr    pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr call_node_a_pub_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr loc_status_pub_;

    // 订阅者
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr                    cloud_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr                            imu_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr                            cmd_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr    init_pose_sub_;

    // tf 广播器和监听器
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::Buffer>               tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener>    tf_listener_;

    // 记录上一次成功计算的 map->odom，避免 TF 暂时不可用时将树"劈成两棵"
    tf2::Transform last_map_to_odom_tf_;
    bool           has_last_map_to_odom_tf_ = false;

    // 地图点云体素滤波器
    pcl::VoxelGrid<PointType> map_voxelgrid_filter_;

    std::unique_ptr<scan2map3d>              localizier3d;
    std::unique_ptr<ErrorStateKalmanFilter>  eskf;

    // IMU数据结构（与lio_node保持一致）
    struct IMUData {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Eigen::Vector3d acc;
        Eigen::Vector3d gyro;
        double          time;
        
        IMUData() = default;
        IMUData(const Eigen::Vector3d &a, const Eigen::Vector3d &g, const double &t)
            : acc(a), gyro(g), time(t) {}
    };

    // 同步数据包结构（与lio_node保持一致）
    struct SyncPackage {
        std::vector<IMUData>            imus;
        pcl::PointCloud<PointType>::Ptr cloud;
        double                          cloud_start_time = 0.0;
        double                          cloud_end_time   = 0.0;
    };

    // 状态数据结构（与lio_node保持一致）
    struct StateData {
        bool    lidar_pushed   = false;
        double  last_lidar_time = -1.0;
        double  last_imu_time   = -1.0;
        
        std::mutex  imu_mutex;
        std::mutex  lidar_mutex;
        
        std::deque<IMUData> imu_buffer;
        std::deque<std::pair<double, pcl::PointCloud<PointType>::Ptr>> lidar_buffer;
    };

    StateData   m_state_data;
    SyncPackage m_package;

    // ========== 点云预处理对象 ==========
    std::unique_ptr<cloud_process_livox> livox_processor_;
    std::unique_ptr<cloud_process>       rs16_processor_;
    std::string                          lidar_type_;

    std::string lidar_frame_;
    std::string imu_frame_;

    std::atomic<bool>          has_extrinsics_{false};
    rclcpp::TimerBase::SharedPtr extrinsics_timer_;

    bool   use_reloc_          = false;
    float  reloc_threshold_    = 0.0;
    double voxel_leaf_size_    = 0.0;
    double imu_scale_          = 0.0;

    Eigen::Matrix3f imu2link_rot_;

    std::string map_frame_;
    std::string base_frame_;
    std::string odom_frame_;
    bool        enable_map_odom_tf_ = true;

    // ===== 发布前位姿重估（仅用于发布链路，不回写算法内部 act_pose） =====
    bool   pose_refine_enable_              = true;
    bool   pose_refine_force_accept_reloc_  = true;
    bool   pose_refine_force_accept_next_   = false;
    bool   pose_refine_inited_              = false;

    double pose_refine_gate_xy_m_           = 0.25;
    double pose_refine_gate_yaw_rad_        = 0.35;
    double pose_refine_k_xy_move_           = 0.25;
    double pose_refine_k_yaw_move_          = 0.20;
    double pose_refine_k_xy_static_         = 0.05;
    double pose_refine_k_yaw_static_        = 0.04;
    double pose_refine_static_v_th_         = 0.03;
    double pose_refine_static_w_th_         = 0.05;
    double pose_refine_tf_lookup_timeout_sec_ = 0.08;

    tf2::Transform pose_refine_map_to_base_prev_;
    tf2::Transform pose_refine_odom_to_base_prev_;
    rclcpp::Time   pose_refine_last_stamp_{0, 0, RCL_SYSTEM_TIME};

    // ============ 性能统计相关变量 ============
    struct PerformanceStats {
        double imu_predict_time_ms        = 0.0;
        double cloud_registration_time_ms = 0.0;
        double eskf_correct_time_ms       = 0.0;
        double pose_publish_time_ms       = 0.0;
        double total_process_time_ms      = 0.0;

        bool   sync_success       = false;
        double lidar_start_time   = 0.0;
        double lidar_end_time     = 0.0;
        double imu_first_time     = 0.0;
        double imu_last_time      = 0.0;
        int    imu_count          = 0;

        size_t imu_buffer_before_sync  = 0;
        double imu_buffer_first_time   = 0.0;
        double imu_buffer_last_time    = 0.0;
    };

    std::deque<PerformanceStats> perf_history_;
    const size_t                 perf_history_size_ = 50;

    rclcpp::Time last_pose_publish_time_;
    int          pose_publish_count_ = 0;
    rclcpp::Time stats_start_time_;

    rclcpp::TimerBase::SharedPtr stats_timer_;

    // ============ 独立处理线程相关变量 ============
    std::thread              process_thread_;
    std::mutex               process_mutex_;
    std::condition_variable  process_cv_;

    // ============ 去畸变相关变量 ============
    bool enable_undistort_ = false;
    int  undistort_count_  = 0;

    location(rclcpp::Node::SharedPtr nh) : nh_(nh) {
        YAML::Node config;
        nh_ = nh;
        
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(nh_);
        tf_buffer_      = std::make_shared<tf2_ros::Buffer>(nh_->get_clock());
        tf_listener_    = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        RCLCPP_INFO(nh->get_logger(), "\n--- Loaded location Parameters ---");
        
        std::string config_file_name;
        nh->declare_parameter<std::string>("config_file", "none");
        nh_->get_parameter("config_file", config_file_name);
        RCLCPP_INFO(nh_->get_logger(), "Loc: config file: %s", config_file_name.c_str());

        // 读取yaml数据
        try {
            config = YAML::LoadFile(config_file_name);
        } catch (YAML::BadFile &e) {
            RCLCPP_ERROR(nh_->get_logger(), "Loc :  read config file error!");
            return;
        }

        // 是否使用重定位
        use_reloc_ = config["use_reloc"].as<bool>();
        RCLCPP_INFO(nh_->get_logger(), "use_reloc %d", use_reloc_);

        imu_scale_ = config["imu_scale"].as<double>();
        RCLCPP_INFO(nh_->get_logger(), "imu_scale %f", imu_scale_);

        reloc_threshold_ = config["reloc_threshold"].as<float>();
        RCLCPP_INFO(nh_->get_logger(), "reloc_threshold %f", reloc_threshold_);

        base_frame_ = config["base_frame"].as<std::string>();
        map_frame_  = config["map_frame"].as<std::string>();
        odom_frame_ = config["odom_frame"] ? config["odom_frame"].as<std::string>() : "odom";
        
        enable_map_odom_tf_ = config["enable_map_odom_tf"] ? 
                              config["enable_map_odom_tf"].as<bool>() : true;
        
        RCLCPP_INFO(nh_->get_logger(), 
                    "base_frame_: %s, map_frame_: %s, odom_frame_: %s, enable_map_odom_tf: %d", 
                    base_frame_.c_str(), map_frame_.c_str(), odom_frame_.c_str(), 
                    enable_map_odom_tf_);

        // 发布前位姿重估参数
        pose_refine_enable_ = config["pose_refine_enable"] ? 
                              config["pose_refine_enable"].as<bool>() : true;
        pose_refine_force_accept_reloc_ = config["pose_refine_force_accept_reloc"] ? 
                                          config["pose_refine_force_accept_reloc"].as<bool>() : true;
        pose_refine_gate_xy_m_ = config["pose_refine_gate_xy_m"] ? 
                                 config["pose_refine_gate_xy_m"].as<double>() : 0.25;
        pose_refine_gate_yaw_rad_ = config["pose_refine_gate_yaw_rad"] ? 
                                    config["pose_refine_gate_yaw_rad"].as<double>() : 0.35;
        pose_refine_k_xy_move_ = config["pose_refine_k_xy_move"] ? 
                                 config["pose_refine_k_xy_move"].as<double>() : 0.25;
        pose_refine_k_yaw_move_ = config["pose_refine_k_yaw_move"] ? 
                                  config["pose_refine_k_yaw_move"].as<double>() : 0.20;
        pose_refine_k_xy_static_ = config["pose_refine_k_xy_static"] ? 
                                   config["pose_refine_k_xy_static"].as<double>() : 0.05;
        pose_refine_k_yaw_static_ = config["pose_refine_k_yaw_static"] ? 
                                    config["pose_refine_k_yaw_static"].as<double>() : 0.04;
        pose_refine_static_v_th_ = config["pose_refine_static_v_th"] ? 
                                   config["pose_refine_static_v_th"].as<double>() : 0.03;
        pose_refine_static_w_th_ = config["pose_refine_static_w_th"] ? 
                                   config["pose_refine_static_w_th"].as<double>() : 0.05;
        pose_refine_tf_lookup_timeout_sec_ = config["pose_refine_tf_lookup_timeout_sec"] ? 
                                             config["pose_refine_tf_lookup_timeout_sec"].as<double>() : 0.08;

        // 约束增益范围
        pose_refine_k_xy_move_   = std::max(0.0, std::min(1.0, pose_refine_k_xy_move_));
        pose_refine_k_yaw_move_  = std::max(0.0, std::min(1.0, pose_refine_k_yaw_move_));
        pose_refine_k_xy_static_ = std::max(0.0, std::min(1.0, pose_refine_k_xy_static_));
        pose_refine_k_yaw_static_ = std::max(0.0, std::min(1.0, pose_refine_k_yaw_static_));

        if (pose_refine_tf_lookup_timeout_sec_ <= 0.0) {
            pose_refine_tf_lookup_timeout_sec_ = 0.08;
        }
        pose_refine_last_stamp_ = rclcpp::Time(0, 0, nh_->get_clock()->get_clock_type());

        RCLCPP_INFO(nh_->get_logger(),
                    "[PoseRefine] enable=%d force_accept_reloc=%d gate_xy=%.3f gate_yaw=%.3f "
                    "k_move(xy=%.2f,yaw=%.2f) k_static(xy=%.2f,yaw=%.2f)",
                    pose_refine_enable_, pose_refine_force_accept_reloc_, 
                    pose_refine_gate_xy_m_, pose_refine_gate_yaw_rad_,
                    pose_refine_k_xy_move_, pose_refine_k_yaw_move_, 
                    pose_refine_k_xy_static_, pose_refine_k_yaw_static_);

        // ========== 新的 TF 外参逻辑 ==========
        lidar_frame_ = config["lidar_frame"] ? config["lidar_frame"].as<std::string>() : "livox_frame";
        imu_frame_   = config["imu_frame"]   ? config["imu_frame"].as<std::string>()   : "imu_link";
        
        Eigen::Matrix3f initial_rot   = Eigen::Matrix3f::Identity();
        Eigen::Vector3f initial_trans = Eigen::Vector3f::Zero();
        
        imu2base_pose.pos.setZero();
        imu2base_pose.orient.setZero();
        imu2link_rot_ = initial_rot;

        // ========== 根据雷达类型初始化对应的点云处理对象 ==========
        lidar_type_       = config["lidar_type"].as<std::string>();
        double filter_leaf_size = config["filter_leaf_size"].as<double>();

        if (lidar_type_ == "livox") {
            livox_processor_.reset(new cloud_process_livox(filter_leaf_size));
            livox_processor_->N_SCANS_     = config["N_SCANS"].as<int>();
            livox_processor_->min_range_   = config["min_range"].as<double>();
            livox_processor_->max_range_   = config["max_range"].as<double>();
            livox_processor_->min_angle_   = config["limit_angle_min"].as<double>() * Deg2Rad;
            livox_processor_->max_angle_   = config["limit_angle_max"].as<double>() * Deg2Rad;
            livox_processor_->base2link_rot   = initial_rot;
            livox_processor_->base2link_trans = initial_trans;

            RCLCPP_INFO(nh_->get_logger(), 
                        "Livox 点云处理器初始化完成: leaf_size=%.2f, range=[%.1f, %.1f], scans=%d",
                        filter_leaf_size, livox_processor_->min_range_, 
                        livox_processor_->max_range_, livox_processor_->N_SCANS_);
        } else if (lidar_type_ == "rs16") {
            rs16_processor_.reset(new cloud_process(filter_leaf_size));
            rs16_processor_->n_scans_         = config["N_SCANS"].as<uint16_t>();
            rs16_processor_->horizon_n_scans_ = config["Horizon_SCAN"].as<uint16_t>();
            rs16_processor_->min_range_       = config["min_range"].as<double>();
            rs16_processor_->max_range_       = config["max_range"].as<double>();
            rs16_processor_->min_angle_       = config["limit_angle_min"].as<double>() * Deg2Rad;
            rs16_processor_->max_angle_       = config["limit_angle_max"].as<double>() * Deg2Rad;
            rs16_processor_->lidar_low_angle_ = config["lidar_low_angle"].as<float>();
            rs16_processor_->lidar_angle_res_x_ = config["lidar_angle_res_x"].as<float>();
            rs16_processor_->lidar_angle_res_y_ = config["lidar_angle_res_y"].as<float>();
            rs16_processor_->base2link_rot   = initial_rot;
            rs16_processor_->base2link_trans = initial_trans;

            RCLCPP_INFO(nh_->get_logger(), 
                        "RS16 点云处理器初始化完成: leaf_size=%.2f, range=[%.1f, %.1f], scans=%d",
                        filter_leaf_size, rs16_processor_->min_range_, 
                        rs16_processor_->max_range_, rs16_processor_->n_scans_);
        } else {
            RCLCPP_ERROR(nh_->get_logger(), "未知的雷达类型: %s,仅支持 'livox' 或 'rs16'", 
                        lidar_type_.c_str());
            return;
        }

        // 外参获取定时器
        extrinsics_timer_ = nh_->create_wall_timer(std::chrono::milliseconds(500), [this]() {
            try {
                // 1. 获取 Lidar 到 Base 的静态 TF
                auto lidar_tf = tf_buffer_->lookupTransform(base_frame_, lidar_frame_, tf2::TimePointZero);
                
                Eigen::Vector3f lidar_trans(lidar_tf.transform.translation.x, 
                                            lidar_tf.transform.translation.y, 
                                            lidar_tf.transform.translation.z);
                Eigen::Quaternionf lidar_q(lidar_tf.transform.rotation.w, 
                                           lidar_tf.transform.rotation.x, 
                                           lidar_tf.transform.rotation.y, 
                                           lidar_tf.transform.rotation.z);
                Eigen::Matrix3f lidar_rot = lidar_q.toRotationMatrix();

                // 2. 获取 IMU 到 Base 的静态 TF
                auto imu_tf = tf_buffer_->lookupTransform(base_frame_, imu_frame_, tf2::TimePointZero);
                
                Eigen::Vector3f imu_trans(imu_tf.transform.translation.x, 
                                          imu_tf.transform.translation.y, 
                                          imu_tf.transform.translation.z);
                Eigen::Quaternionf imu_q(imu_tf.transform.rotation.w, 
                                         imu_tf.transform.rotation.x, 
                                         imu_tf.transform.rotation.y, 
                                         imu_tf.transform.rotation.z);
                Eigen::Matrix3f imu_rot = imu_q.toRotationMatrix();

                // 3. 将真实外参覆写给处理器
                if (lidar_type_ == "livox") {
                    livox_processor_->base2link_trans = lidar_trans;
                    livox_processor_->base2link_rot   = lidar_rot;
                } else if (lidar_type_ == "rs16") {
                    rs16_processor_->base2link_trans = lidar_trans;
                    rs16_processor_->base2link_rot   = lidar_rot;
                }

                imu2base_pose.pos = imu_trans;
                Eigen::Vector3f imu_rpy = imu_rot.eulerAngles(2, 1, 0);
                imu2base_pose.orient << imu_rpy[2], imu_rpy[1], imu_rpy[0];
                update_q(imu2base_pose);
                imu2link_rot_ = imu_rot;

                RCLCPP_INFO(nh_->get_logger(), "🎉 成功从 TF 树获取传感器外参！定位系统正式激活。");
                RCLCPP_INFO(nh_->get_logger(), "Lidar2Base: [%.3f, %.3f, %.3f]", 
                            lidar_trans(0), lidar_trans(1), lidar_trans(2));
                RCLCPP_INFO(nh_->get_logger(), "Imu2Base:   [%.3f, %.3f, %.3f]", 
                            imu_trans(0), imu_trans(1), imu_trans(2));

                has_extrinsics_ = true;
                extrinsics_timer_->cancel();
            } catch (const tf2::TransformException &ex) {
                RCLCPP_WARN_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 2000, 
                    "等待 URDF 静态 TF (%s & %s) -> %s 准备就绪...", 
                    lidar_frame_.c_str(), imu_frame_.c_str(), base_frame_.c_str());
            }
        });

        // 声明并获取初始位置参数
        nh_->declare_parameter("init_x", 0.0);
        nh_->declare_parameter("init_y", 0.0);
        nh_->declare_parameter("init_z", 0.0);
        nh_->declare_parameter("init_roll", 0.0);
        nh_->declare_parameter("init_pitch", 0.0);
        nh_->declare_parameter("init_yaw", 0.0);

        init_pose_.pos(0)    = config["init_x"].as<float>();
        init_pose_.pos(1)    = config["init_y"].as<float>();
        init_pose_.pos(2)    = config["init_z"].as<float>();
        init_pose_.orient(0) = config["init_roll"].as<float>();
        init_pose_.orient(1) = config["init_pitch"].as<float>();
        init_pose_.orient(2) = config["init_yaw"].as<float>();

        RCLCPP_INFO(nh_->get_logger(), "Loc: init_pose_: %f, %f, %f, %f, %f, %f", 
                    init_pose_.pos(0), init_pose_.pos(1), init_pose_.pos(2), 
                    init_pose_.orient(0), init_pose_.orient(1), init_pose_.orient(2));

        update_q(init_pose_);
        act_pose = init_pose_;
        act_pose.vel << 0.0, 0.0, 0.0;

        // 初始化订阅，发布器
        std::string lidar_topic = config["lidar_topic"].as<std::string>();

        cloud_sub_ = nh_->create_subscription<sensor_msgs::msg::PointCloud2>(
            lidar_topic, rclcpp::QoS(rclcpp::KeepLast(1)),
            std::bind(&location::robosense_cb, this, std::placeholders::_1));
        
        std::string cmd_topic = config["cmd_topic"].as<std::string>();
        cmd_sub_ = nh_->create_subscription<std_msgs::msg::String>(
            cmd_topic, 10, std::bind(&location::command_cb, this, std::placeholders::_1));

        std::string imu_topic = config["imu_topic"].as<std::string>();
        
        rclcpp::QoS imu_qos(200);
        imu_qos.best_effort();
        
        imu_sub_ = nh_->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic, imu_qos, std::bind(&location::imu_cb, this, std::placeholders::_1));

        std::string loc_status_topic = config["loc_status_topic"] ? 
                                            config["loc_status_topic"].as<std::string>() : "/localization_status";
        loc_status_pub_ = nh_->create_publisher<std_msgs::msg::Int8>(loc_status_topic, rclcpp::QoS(rclcpp::KeepLast(1));

        std::string call_node_a_topic = config["call_pub_topic"] ? 
                                            config["call_pub_topic"].as<std::string>() : "/initialpose";
        call_node_a_pub_ = nh_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(call_node_a_topic, 1);
        RCLCPP_INFO(nh_->get_logger(), "初始化求救发布器"：%s", call_pub_topic.c_str());

        std::string init_pose_topic = config["init_pose_topic"] ? 
                                      config["init_pose_topic"].as<std::string>() : "/icp_pose";
        init_pose_sub_ = nh_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            init_pose_topic, 1, std::bind(&location::initpose_cb, this, std::placeholders::_1));
        RCLCPP_INFO(nh_->get_logger(), "监听外部高精度位姿话题: %s", init_pose_topic.c_str());

        std::string map_pub_topic_ = config["map_pub_topic"].as<std::string>();
        pc_map_pub_ = nh_->create_publisher<sensor_msgs::msg::PointCloud2>(
            map_pub_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
        RCLCPP_INFO(nh_->get_logger(), "map_pub_topic_: %s", map_pub_topic_.c_str());

        // 地图文件路径处理
        std::string ros_map_filename;
        nh_->declare_parameter<std::string>("map_filename", "");
        nh_->get_parameter("map_filename", ros_map_filename);

        if (!ros_map_filename.empty()) {
            map_file_path = ros_map_filename;
            RCLCPP_INFO(nh_->get_logger(), "Loc: [Launch Overwrite] map_file_path: %s", 
                        map_file_path.c_str());
        } else {
            if (config["map_filename"]) {
                map_file_path = config["map_filename"].as<std::string>();
                RCLCPP_INFO(nh_->get_logger(), "Loc: [YAML Config] map_file_path: %s", 
                            map_file_path.c_str());
            } else {
                RCLCPP_ERROR(nh_->get_logger(), 
                             "Loc: No map_filename provided in Launch args or YAML config!");
            }
        }
        
        map_cloud.reset(new pcl::PointCloud<PointType>);

        if (pcl::io::loadPCDFile(map_file_path.c_str(), *map_cloud) == -1) {
            RCLCPP_ERROR(nh_->get_logger(), "Loc: load map failed %s", map_file_path.c_str());
            return;
        }

        RCLCPP_INFO(nh_->get_logger(), "Loc: map_cloud->size(): %zu", map_cloud->size());

        voxel_leaf_size_ = config["map_filtr_leaf_size"].as<double>();

        map_voxelgrid_filter_.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
        pcl::PointCloud<PointType>::Ptr map_cloud_filter(new pcl::PointCloud<PointType>);
        map_cloud_filter->clear();

        map_voxelgrid_filter_.setInputCloud(map_cloud);
        map_voxelgrid_filter_.filter(*map_cloud_filter);
        
        RCLCPP_INFO(nh_->get_logger(), "Loc: map_cloud_filter->points.size(): %zu", 
                    map_cloud_filter->points.size());

        map_cloud_filter->width  = map_cloud_filter->points.size();
        map_cloud_filter->height = 1;

        // 发布地图
        pcl::toROSMsg(*map_cloud_filter, pubmap_msg);
        if (pubmap_msg.width != 0) {
            RCLCPP_INFO(nh_->get_logger(), "Loc: map has been published");
            pubmap_msg.header.frame_id = map_frame_;
            pubmap_msg.header.stamp    = nh_->get_clock()->now();
            RCLCPP_INFO(nh_->get_logger(), "Publishing map with timestamp: %.9f", 
                        getSec(pubmap_msg.header));
            rclcpp::sleep_for(std::chrono::seconds(2));
            pc_map_pub_->publish(pubmap_msg);
        } else {
            RCLCPP_ERROR(nh_->get_logger(), " Loc :  can not publish map");
            return;
        }

        localizier3d.reset(new scan2map3d(map_cloud_filter, config_file_name));
        localizier3d->setNormalMode();

        // 初始化eskf
        std::string eskf_cfg_path;
        nh_->declare_parameter<std::string>("eskf_cfg_file", "none");
        nh_->get_parameter<std::string>("eskf_cfg_file", eskf_cfg_path);
        RCLCPP_INFO(nh_->get_logger(), "eskf_cfg_path: %s", eskf_cfg_path.c_str());
        eskf.reset(new ErrorStateKalmanFilter(eskf_cfg_path));

        std::string pose_pub_topic_ = config["pose_pub_topic"].as<std::string>();
        pose_pub_ = nh_->create_publisher<geometry_msgs::msg::PoseStamped>(pose_pub_topic_, 10);
        RCLCPP_INFO(nh_->get_logger(), "pose_pub_topic_: %s", pose_pub_topic_.c_str());

        std::string processed_cloud_topic = config["pub_cloud_topic"].as<std::string>();
        processed_cloud_pub_ = nh_->create_publisher<sensor_msgs::msg::PointCloud2>(
            processed_cloud_topic, 10);
        RCLCPP_INFO(nh_->get_logger(), "processed_cloud_topic: %s", processed_cloud_topic.c_str());

        // 初始化性能统计
        last_pose_publish_time_ = nh_->get_clock()->now();
        stats_start_time_       = nh_->get_clock()->now();

        // 初始化去畸变配置
        enable_undistort_ = config["enable_undistort"] ? 
                            config["enable_undistort"].as<bool>() : true;
        
        if (enable_undistort_) {
            RCLCPP_INFO(nh_->get_logger(), 
                        "[去畸变] 功能已启用 (已优化为 base_link 系下直接运动补偿)");
        } else {
            RCLCPP_WARN(nh_->get_logger(), "[去畸变] 配置文件已禁用去畸变功能");
        }

        // 创建定时器，每5秒输出一次性能统计
        stats_timer_ = nh_->create_wall_timer(
            std::chrono::seconds(5),
            std::bind(&location::print_performance_stats, this));

        // 启动独立处理线程
        process_thread_ = std::thread(&location::processThreadFunc, this);
    }
    
    ~location() {
        RCLCPP_INFO(nh_->get_logger(), "[析构] 开始清理资源...");
        
        g_b_exit.store(true);
        process_cv_.notify_all();
        
        if (process_thread_.joinable()) {
            process_thread_.join();
            RCLCPP_INFO(nh_->get_logger(), "[析构] 处理线程已关闭");
        }
        
        RCLCPP_INFO(nh_->get_logger(), "[析构] 资源清理完成");
    }

    // 性能统计输出函数
    void print_performance_stats() {
        if (perf_history_.empty()) return;

        double avg_imu_predict  = 0.0;
        double avg_registration = 0.0;
        double avg_eskf_correct = 0.0;
        double avg_pose_publish = 0.0;
        double avg_total        = 0.0;
        double avg_imu_count    = 0.0;

        for (const auto &stat : perf_history_) {
            avg_imu_predict  += stat.imu_predict_time_ms;
            avg_registration += stat.cloud_registration_time_ms;
            avg_eskf_correct += stat.eskf_correct_time_ms;
            avg_pose_publish += stat.pose_publish_time_ms;
            avg_total        += stat.total_process_time_ms;
            avg_imu_count    += stat.imu_count;
        }

        size_t count = perf_history_.size();
        avg_imu_predict  /= count;
        avg_registration /= count;
        avg_eskf_correct /= count;
        avg_pose_publish /= count;
        avg_total        /= count;
        avg_imu_count    /= count;

        double elapsed_time   = (nh_->get_clock()->now() - stats_start_time_).seconds();
        double pose_pub_freq  = (elapsed_time > 0.0) ? (pose_publish_count_ / elapsed_time) : 0.0;

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
                    avg_imu_predict, avg_imu_count, avg_registration, 
                    avg_eskf_correct, avg_pose_publish, avg_total,
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
                    last_stat.eskf_correct_time_ms, last_stat.pose_publish_time_ms, 
                    last_stat.total_process_time_ms);
    }

    // 发布前位姿重估
    bool refinePoseForPublish(const pose_type &raw_pose, const double &time, 
                              pose_type &refined_pose) {
        refined_pose = raw_pose;
        if (!pose_refine_enable_) return false;

        const rclcpp::Time stamp = getTime(time);
        geometry_msgs::msg::TransformStamped odom_to_base_msg;
        
        try {
            const auto timeout = rclcpp::Duration::from_seconds(pose_refine_tf_lookup_timeout_sec_);
            odom_to_base_msg = tf_buffer_->lookupTransform(odom_frame_, base_frame_, stamp, timeout);
        } catch (tf2::TransformException &ex) {
            try {
                odom_to_base_msg = tf_buffer_->lookupTransform(odom_frame_, base_frame_, 
                                                                tf2::TimePointZero);
            } catch (tf2::TransformException &ex_latest) {
                RCLCPP_WARN_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 2000,
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
            pose_refine_map_to_base_prev_  = map_to_base_meas;
            pose_refine_odom_to_base_prev_ = odom_to_base_tf;
            pose_refine_last_stamp_        = stamp;
            pose_refine_inited_            = true;
            return true;
        }

        const double dt = (stamp - pose_refine_last_stamp_).seconds();
        if (dt <= 1e-4) return true;

        const tf2::Transform odom_delta = pose_refine_odom_to_base_prev_.inverse() * odom_to_base_tf;
        const tf2::Transform map_to_base_pred = pose_refine_map_to_base_prev_ * odom_delta;

        const double linear_speed = std::hypot(odom_delta.getOrigin().x(), 
                                               odom_delta.getOrigin().y()) / dt;
        const double yaw_delta    = normalize_angle(tf2::getYaw(odom_delta.getRotation()));
        const double yaw_rate     = std::fabs(yaw_delta / dt);

        const double pred_x   = map_to_base_pred.getOrigin().x();
        const double pred_y   = map_to_base_pred.getOrigin().y();
        const double pred_yaw = tf2::getYaw(map_to_base_pred.getRotation());
        const double meas_x   = map_to_base_meas.getOrigin().x();
        const double meas_y   = map_to_base_meas.getOrigin().y();
        const double meas_yaw = tf2::getYaw(map_to_base_meas.getRotation());

        const double err_x   = meas_x - pred_x;
        const double err_y   = meas_y - pred_y;
        const double err_xy  = std::hypot(err_x, err_y);
        const double err_yaw = normalize_angle(meas_yaw - pred_yaw);

        const bool is_static = linear_speed < pose_refine_static_v_th_ && 
                               yaw_rate < pose_refine_static_w_th_;
        const double k_xy  = is_static ? pose_refine_k_xy_static_ : pose_refine_k_xy_move_;
        const double k_yaw = is_static ? pose_refine_k_yaw_static_ : pose_refine_k_yaw_move_;

        tf2::Transform map_to_base_refined = map_to_base_pred;
        const bool force_accept = pose_refine_force_accept_next_;
        pose_refine_force_accept_next_ = false;

        if (force_accept) {
            map_to_base_refined = map_to_base_meas;
            RCLCPP_WARN(nh_->get_logger(),
                        "[PoseRefine] 重定位后强制接受测量位姿: err_xy=%.3f m, err_yaw=%.3f rad",
                        err_xy, err_yaw);
        } else if (err_xy > pose_refine_gate_xy_m_ || 
                   std::fabs(err_yaw) > pose_refine_gate_yaw_rad_) {
            RCLCPP_WARN_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 10000,
                "[PoseRefine] 位姿门控生效: err_xy=%.3f m, err_yaw=%.3f rad (gate=%.3f, %.3f)",
                err_xy, err_yaw, pose_refine_gate_xy_m_, pose_refine_gate_yaw_rad_);
        } else {
            const double out_x   = pred_x + k_xy * err_x;
            const double out_y   = pred_y + k_xy * err_y;
            const double out_yaw = normalize_angle(pred_yaw + k_yaw * err_yaw);
            
            tf2::Quaternion q_refined;
            q_refined.setRPY(raw_pose.orient(0), raw_pose.orient(1), out_yaw);
            
            map_to_base_refined.setOrigin(tf2::Vector3(out_x, out_y, raw_pose.pos(2)));
            map_to_base_refined.setRotation(q_refined);
        }

        refined_pose.pos(0)    = map_to_base_refined.getOrigin().x();
        refined_pose.pos(1)    = map_to_base_refined.getOrigin().y();
        refined_pose.pos(2)    = raw_pose.pos(2);
        refined_pose.orient(0) = raw_pose.orient(0);
        refined_pose.orient(1) = raw_pose.orient(1);
        refined_pose.orient(2) = normalize_angle(tf2::getYaw(map_to_base_refined.getRotation()));
        update_q(refined_pose);

        pose_refine_map_to_base_prev_  = map_to_base_refined;
        pose_refine_odom_to_base_prev_ = odom_to_base_tf;
        pose_refine_last_stamp_        = stamp;
        
        return true;
    }

    // 发布点云数据到指定话题
    void publishCloud(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub,
                      pcl::PointCloud<PointType>::Ptr cloud,
                      const std::string &frame_id,
                      const double &time) {
        if (pub->get_subscription_count() <= 0) return;

        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(*cloud, cloud_msg);

        cloud_msg.header.frame_id = frame_id;
        cloud_msg.header.stamp    = getTime(time);

        pub->publish(cloud_msg);
    }

    // 发布位姿信息
    void publishPose(const pose_type &pose, const std::string &frame_id, 
                     const double &time) {
        if (pose_pub_->get_subscription_count() <= 0) return;

        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.frame_id = frame_id;
        pose_msg.header.stamp    = getTime(time);

        pose_msg.pose.position.x = pose.pos(0);
        pose_msg.pose.position.y = pose.pos(1);
        pose_msg.pose.position.z = pose.pos(2);

        tf2::Quaternion q;
        q.setRPY(pose.orient(0), pose.orient(1), pose.orient(2));
        pose_msg.pose.orientation.x = q.x();
        pose_msg.pose.orientation.y = q.y();
        pose_msg.pose.orientation.z = q.z();
        pose_msg.pose.orientation.w = q.w();

        pose_pub_->publish(pose_msg);
    }

    // 广播TF变换信息
    void broadCastTF(const pose_type &pose, const std::string &frame_id, 
                     const std::string &child_frame, const double &time) {
        tf2::Transform map_to_base_tf;
        map_to_base_tf.setOrigin(tf2::Vector3(pose.pos(0), pose.pos(1), pose.pos(2)));
        
        tf2::Quaternion q;
        q.setRPY(pose.orient(0), pose.orient(1), pose.orient(2));
        map_to_base_tf.setRotation(q);

        rclcpp::Time stamp = getTime(time);

        if (!enable_map_odom_tf_) {
            geometry_msgs::msg::TransformStamped transformStamped;
            transformStamped.header.frame_id    = frame_id;
            transformStamped.child_frame_id     = child_frame;
            transformStamped.header.stamp       = stamp;
            transformStamped.transform.translation.x = pose.pos(0);
            transformStamped.transform.translation.y = pose.pos(1);
            transformStamped.transform.translation.z = pose.pos(2);
            transformStamped.transform.rotation.x    = q.x();
            transformStamped.transform.rotation.y    = q.y();
            transformStamped.transform.rotation.z    = q.z();
            transformStamped.transform.rotation.w    = q.w();
            
            tf_broadcaster_->sendTransform(transformStamped);
        } else {
            try {
                geometry_msgs::msg::TransformStamped odom_to_base_msg = 
                    tf_buffer_->lookupTransform(odom_frame_, base_frame_, stamp, 
                                                rclcpp::Duration::from_seconds(0.1));
                
                tf2::Transform odom_to_base_tf;
                tf2::fromMsg(odom_to_base_msg.transform, odom_to_base_tf);
                
                tf2::Transform map_to_odom_tf = map_to_base_tf * odom_to_base_tf.inverse();
                last_map_to_odom_tf_ = map_to_odom_tf;
                has_last_map_to_odom_tf_ = true;
                
                geometry_msgs::msg::TransformStamped map_to_odom_stamped;
                map_to_odom_stamped.header.stamp       = stamp;
                map_to_odom_stamped.header.frame_id    = map_frame_;
                map_to_odom_stamped.child_frame_id     = odom_frame_;
                map_to_odom_stamped.transform          = tf2::toMsg(map_to_odom_tf);
                
                tf_broadcaster_->sendTransform(map_to_odom_stamped);
                
            } catch (tf2::TransformException &ex) {
                static int warn_count = 0;
                if (warn_count++ % 50 == 0) {
                    RCLCPP_WARN(nh_->get_logger(), 
                                "[TF] 无法获取 %s→%s 变换: %s",
                                odom_frame_.c_str(), base_frame_.c_str(), ex.what());
                }

                if (has_last_map_to_odom_tf_) {
                    geometry_msgs::msg::TransformStamped map_to_odom_stamped;
                    map_to_odom_stamped.header.stamp       = stamp;
                    map_to_odom_stamped.header.frame_id    = map_frame_;
                    map_to_odom_stamped.child_frame_id     = odom_frame_;
                    map_to_odom_stamped.transform          = tf2::toMsg(last_map_to_odom_tf_);
                    
                    tf_broadcaster_->sendTransform(map_to_odom_stamped);
                }
            }
        }
    }

    // IMU回调函数
    void imu_cb(const sensor_msgs::msg::Imu::SharedPtr imu_msg) {
        if (!has_extrinsics_) return;

        double sys_time_before_lock = nh_->get_clock()->now().seconds();
        double msg_timestamp        = getSec(imu_msg->header);
        double delay_before_lock    = sys_time_before_lock - msg_timestamp;
        
        static int imu_diag_count = 0;
        bool should_print = (++imu_diag_count % 200 == 0);
        
        if (should_print) {
            RCLCPP_INFO(nh_->get_logger(), 
                        "[IMU-锁前] 消息时间=%.6f | 系统时间=%.6f | 延迟=%.3fs(%.0fms)",
                        msg_timestamp, sys_time_before_lock, 
                        delay_before_lock, delay_before_lock * 1000.0);
        }
        
        std::lock_guard<std::mutex> lock(m_state_data.imu_mutex);
        
        double timestamp = msg_timestamp;

        if (timestamp < m_state_data.last_imu_time) {
            RCLCPP_WARN(nh_->get_logger(), "IMU Message is out of order");
            std::deque<IMUData>().swap(m_state_data.imu_buffer);
        }

        Eigen::Vector3d acc(imu_msg->linear_acceleration.x,
                            imu_msg->linear_acceleration.y,
                            imu_msg->linear_acceleration.z);
        Eigen::Vector3d gyro(imu_msg->angular_velocity.x,
                             imu_msg->angular_velocity.y,
                             imu_msg->angular_velocity.z);

        acc  = imu2link_rot_.cast<double>() * acc;
        gyro = imu2link_rot_.cast<double>() * gyro;

        Eigen::Vector3d p_imu = imu2base_pose.pos.cast<double>();
        Eigen::Vector3d centripetal_acc = gyro.cross(gyro.cross(p_imu));
        acc = acc - centripetal_acc; // 杆臂补偿
        
        if (acc.z() > 5.0) {
            acc = acc / imu_scale_;
        } else if (acc.z() < 0) {
            acc   = Eigen::Vector3d(-acc.y(), -acc.x(), -acc.z());
            gyro  = Eigen::Vector3d(-gyro.y(), -gyro.x(), -gyro.z());
        }

        Eigen::Vector3d final_acc = acc * imu_scale_;

        m_state_data.imu_buffer.emplace_back(final_acc, gyro, timestamp);
        m_state_data.last_imu_time = timestamp;
    }

    // 激光点云回调
    void robosense_cb(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg) {
        if (!has_extrinsics_) return;

        double msg_timestamp = getSec(cloud_msg->header);
        double sys_timestamp = nh_->get_clock()->now().seconds();
        double time_diff     = sys_timestamp - msg_timestamp;
        
        RCLCPP_INFO(nh_->get_logger(), 
                    "[LiDAR回调] 消息时间=%.6f | 系统时间=%.6f | 差值=%.3fs(%.0fms)",
                    msg_timestamp, sys_timestamp, time_diff, time_diff * 1000.0);
        
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
        
        auto process_end = std::chrono::high_resolution_clock::now();
        double process_time_ms = std::chrono::duration<double, std::milli>(
            process_end - process_start).count();
        
        {
            std::lock_guard<std::mutex> lock(m_state_data.lidar_mutex);
            
            if (msg_timestamp < m_state_data.last_lidar_time) {
                RCLCPP_WARN(nh_->get_logger(), "[LiDAR回调] 消息乱序,清空缓冲");
                std::deque<std::pair<double, pcl::PointCloud<PointType>::Ptr>>().swap(
                    m_state_data.lidar_buffer);
            }
            
            m_state_data.lidar_buffer.emplace_back(msg_timestamp, processed_cloud);
            m_state_data.last_lidar_time = msg_timestamp;
            
            RCLCPP_DEBUG(nh_->get_logger(), "[LiDAR回调] 点云处理耗时=%.1fms", process_time_ms);
        }
        
        process_cv_.notify_one();
    }

    // 同步IMU和LiDAR数据包
    bool syncPackage() {
        if (m_state_data.imu_buffer.empty() || m_state_data.lidar_buffer.empty())
            return false;

        if (!m_state_data.lidar_pushed) {
            m_package.cloud = m_state_data.lidar_buffer.front().second;

            std::sort(m_package.cloud->points.begin(), m_package.cloud->points.end(),
                      [](PointType &p1, PointType &p2) {
                          return p1.curvature < p2.curvature;
                      });

            m_package.cloud_start_time = m_state_data.lidar_buffer.front().first;
            m_package.cloud_end_time   = m_package.cloud_start_time +
                                         m_package.cloud->points.back().curvature / 1000.0;

            m_state_data.lidar_pushed = true;
        }

        if (m_state_data.last_imu_time < m_package.cloud_end_time)
            return false;

        std::vector<IMUData>().swap(m_package.imus);

        while (!m_state_data.imu_buffer.empty() &&
               m_state_data.imu_buffer.front().time < m_package.cloud_end_time) {
            m_package.imus.emplace_back(m_state_data.imu_buffer.front());
            m_state_data.imu_buffer.pop_front();
        }

        m_state_data.lidar_buffer.pop_front();
        m_state_data.lidar_pushed = false;

        return true;
    }

    // 接收 Node A 发来的高精度绝对位姿
    void initpose_cb(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose_msg) {
        // 1. 提取 Node A 算出来的坐标
        double node_a_x = pose_msg->pose.pose.position.x;
        double node_a_y = pose_msg->pose.pose.position.y;
        
        // 2. 计算 Node A 坐标与当前主节点坐标的直线偏差距离
        double dx = act_pose.pos(0) - node_a_x;
        double dy = act_pose.pos(1) - node_a_y;
        double diff_dist = std::sqrt(dx * dx + dy * dy);

        // 3. 智能拦截逻辑
        // 如果主节点现在没有在迷失（use_reloc_ == false），而且匹配得分大于阈值
        if (!use_reloc_ && localizier3d->fitness_ > reloc_threshold_) {
            if (diff_dist < 1.0) { // 阈值可以根据实际车速调，1.0米是个不错的经验值
                RCLCPP_INFO(nh_->get_logger(), 
                    "💡 Node A 算完了，但主节点已自行恢复且偏差很小(%.2fm)。为防止延时拉扯，忽略此次空投。", 
                    diff_dist);
                return; // 直接拦截，不执行覆盖！
            } else {
                RCLCPP_WARN(nh_->get_logger(), 
                    "⚠️ 危险！主节点自认为正常，但与 Node A 的绝对坐标相差达 %.2fm！疑似陷入局部误匹配，准备强行接管！", 
                    diff_dist);
            }
        } else {
            RCLCPP_INFO(nh_->get_logger(), 
                "🆘 主节点仍在迷失中，像抓住了救命稻草，正在接受 Node A 绝对位姿接管！");
        }

        // 4. 执行接管覆盖逻辑
        std::lock_guard<std::mutex> lock(reloc_pose_mutex_);
        
        target_global_pose_.pos(0) = pose_msg->pose.pose.position.x;
        target_global_pose_.pos(1) = pose_msg->pose.pose.position.y;
        target_global_pose_.pos(2) = pose_msg->pose.pose.position.z;

        double siny_cosp = 2.0 * (pose_msg->pose.pose.orientation.w * pose_msg->pose.pose.orientation.z + 
                                  pose_msg->pose.pose.orientation.x * pose_msg->pose.pose.orientation.y);
        double cosy_cosp = 1.0 - 2.0 * (pose_msg->pose.pose.orientation.y * pose_msg->pose.pose.orientation.y + 
                                        pose_msg->pose.pose.orientation.z * pose_msg->pose.pose.orientation.z);
        
        target_global_pose_.orient(0) = 0.0;
        target_global_pose_.orient(1) = 0.0;
        target_global_pose_.orient(2) = atan2(siny_cosp, cosy_cosp);

        target_global_pose_.vel << 0.0, 0.0, 0.0;
        
        has_new_global_pose_ = true;
    }

    // 状态命令反馈
    void command_cb(const std_msgs::msg::String::SharedPtr msg) {
        RCLCPP_INFO(nh_->get_logger(), "cmd: %s", msg->data.c_str());
        
        if (msg->data == "shutdown") {
            rclcpp::shutdown();
        } else if (msg->data == "test_reloc") {
            RCLCPP_INFO(nh_->get_logger(), "test reloc");
            act_pose.pos(0) = act_pose.pos(0) + 5.0;
            act_pose.pos(1) = 4.0;
        }
    }

    void imu_predict(const std::vector<IMUData> &imus, PerformanceStats &perf_stat) {
        if (imus.empty()) {
            perf_stat.imu_predict_time_ms = 0.0;
            return;
        }

        auto imu_predict_start = std::chrono::high_resolution_clock::now();

        for (const auto &imu : imus) {
            Eigen::VectorXd imu_data(6);
            imu_data << imu.acc, imu.gyro;

            if (check_imu_data(imu_data)) {
                eskf->predict(imu_data, imu.time);
                eskf->getPose(act_pose);
            }
        }

        auto imu_predict_end = std::chrono::high_resolution_clock::now();
        perf_stat.imu_predict_time_ms = std::chrono::duration<double, std::milli>(
            imu_predict_end - imu_predict_start).count();
    }

    // IMU插值函数
    IMUData interpolateImu(const IMUData &imu1, const IMUData &imu2, double t) {
        if (std::abs(imu2.time - imu1.time) < 1e-9) return imu1;

        double ratio = (t - imu1.time) / (imu2.time - imu1.time);
        double w1    = 1.0 - ratio;
        double w2    = ratio;

        IMUData imu_inter;
        imu_inter.acc  = w1 * imu1.acc + w2 * imu2.acc;
        imu_inter.gyro = w1 * imu1.gyro + w2 * imu2.gyro;
        imu_inter.time = t;

        return imu_inter;
    }

    // 角速度积分函数
    Sophus::SO3d integrateRotation(double start_time, double end_time,
                                   const std::vector<IMUData> &v_imu) {
        Sophus::SO3d result_rot;

        if (v_imu.empty() || start_time >= end_time) return result_rot;

        size_t start_idx = 0;
        for (size_t i = 0; i < v_imu.size(); ++i) {
            if (v_imu[i].time >= start_time) {
                start_idx = (i > 0) ? i - 1 : 0;
                break;
            }
        }

        size_t end_idx = v_imu.size() - 1;
        for (size_t i = v_imu.size(); i > 0; --i) {
            if (v_imu[i - 1].time <= end_time) {
                end_idx = i - 1;
                break;
            }
        }

        if (start_idx >= v_imu.size() || end_idx >= v_imu.size() || start_idx > end_idx)
            return result_rot;

        IMUData current_imu;
        double current_time = start_time;

        if (start_idx == 0) {
            current_imu = v_imu[0];
        } else {
            current_imu = interpolateImu(v_imu[start_idx - 1], v_imu[start_idx], start_time);
        }

        for (size_t i = start_idx; i <= end_idx; ++i) {
            double next_time = v_imu[i].time;

            if (next_time > end_time) {
                double prev_time = (i > 0) ? v_imu[i - 1].time : start_time;
                IMUData end_imu = interpolateImu(v_imu[i - 1], v_imu[i], end_time);

                double dt = end_time - current_time;
                Eigen::Vector3d delta_angle = dt * 0.5 * (current_imu.gyro + end_imu.gyro);
                Sophus::SO3d delta_r = Sophus::SO3d::exp(delta_angle);
                result_rot = result_rot * delta_r;

                break;
            } else {
                double dt = next_time - current_time;
                Eigen::Vector3d delta_angle = dt * 0.5 * (current_imu.gyro + v_imu[i].gyro);
                Sophus::SO3d delta_r = Sophus::SO3d::exp(delta_angle);
                result_rot = result_rot * delta_r;

                current_time = next_time;
                current_imu  = v_imu[i];
            }
        }

        if (end_time > v_imu.back().time) {
            double dt = end_time - current_time;
            Eigen::Vector3d delta_angle = dt * current_imu.gyro;
            Sophus::SO3d delta_r = Sophus::SO3d::exp(delta_angle);
            result_rot = result_rot * delta_r;
        }

        return result_rot;
    }

    // 点云去畸变函数
    void undistortPointCloud(pcl::PointCloud<PointType>::Ptr &cloud,
                             double start_time, double end_time,
                             const std::vector<IMUData> &imus) {
        if (!enable_undistort_ || imus.empty() || cloud->empty()) return;

        auto undistort_start = std::chrono::high_resolution_clock::now();

        Sophus::SO3d rot_base_be = integrateRotation(start_time, end_time, imus);

        Eigen::Vector3d rso3_be = rot_base_be.log();
        Eigen::Vector3d tbe     = Eigen::Vector3d::Zero();
        double dt_be            = end_time - start_time;

        if (dt_be <= 0) {
            RCLCPP_ERROR(nh_->get_logger(), "[去畸变] 无效的时间跨度: start=%.6f, end=%.6f",
                         start_time, end_time);
            return;
        }

        for (auto &pt : cloud->points) {
            double dt_bi = pt.curvature / 1000.0;

            if (dt_bi < 0 || dt_bi > dt_be) continue;

            double ratio_bi = dt_bi / dt_be;
            double ratio_ie = 1.0 - ratio_bi;

            Eigen::Vector3d rso3_ie = ratio_ie * rso3_be;
            Sophus::SO3d Rie = Sophus::SO3d::exp(rso3_ie);
            Eigen::Vector3d tie = ratio_ie * tbe;

            Eigen::Vector3d v_pt_i(pt.x, pt.y, pt.z);
            Eigen::Vector3d v_pt_comp_e = Rie.inverse() * (v_pt_i - tie);

            pt.x = v_pt_comp_e.x();
            pt.y = v_pt_comp_e.y();
            pt.z = v_pt_comp_e.z();
        }

        auto undistort_end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
            undistort_end - undistort_start);

        RCLCPP_INFO(nh_->get_logger(),
                    "[去畸变] 点云数=%zu | IMU数=%zu | 耗时=%.2fms | "
                    "旋转角度=(%.2f°, %.2f°, %.2f°) | 时间跨度=%.1fms",
                    cloud->size(), imus.size(), duration.count() / 1000.0,
                    rot_base_be.angleX() * 180.0 / M_PI,
                    rot_base_be.angleY() * 180.0 / M_PI,
                    rot_base_be.angleZ() * 180.0 / M_PI,
                    (end_time - start_time) * 1000.0);
    }

    // ============ 独立处理线程函数 ============
    void processThreadFunc() {
        RCLCPP_INFO(nh_->get_logger(), "[处理线程] 启动,使用同步触发模式(参考Livox)");
        
        int process_count = 0;

        while (rclcpp::ok() && !g_b_exit.load()) {
            if (!has_extrinsics_) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }
            
            {
                std::unique_lock<std::mutex> lock(process_mutex_);
                
                if (!process_cv_.wait_for(lock, std::chrono::milliseconds(10),
                    [this]() { 
                        if (g_b_exit.load()) return true;
                        return syncPackage();
                    })) {
                    continue;
                }
                
                if (g_b_exit.load()) {
                    RCLCPP_INFO(nh_->get_logger(), "[处理线程] 收到退出信号");
                    break;
                }
            }
            
            // 安全重置外部传来的全局位姿
            {
                std::lock_guard<std::mutex> lock(reloc_pose_mutex_);
                if (has_new_global_pose_) {
                    RCLCPP_WARN(nh_->get_logger(), "🔄 正在执行全局位姿强行覆盖...");
                    
                    act_pose = target_global_pose_;
                    use_reloc_ = false;
                    localizier3d->setNormalMode();

                    eskf->setMean(act_pose.pos.cast<double>(), 
                                  act_pose.orient.cast<double>(), 
                                  act_pose.vel.cast<double>());
                    eskf->reset();

                    {
                        std::lock_guard<std::mutex> imu_lock(m_state_data.imu_mutex);
                        m_state_data.imu_buffer.clear();
                    }
                    
                    pose_refine_force_accept_next_ = true; 
                    pose_refine_inited_ = false;

                    has_new_global_pose_ = false;
                    RCLCPP_INFO(nh_->get_logger(), "✅ 全局位姿覆盖完成，已恢复平滑跟踪状态！");
                }
            }

            PerformanceStats perf_stat;
            auto frame_start_time = std::chrono::high_resolution_clock::now();
            
            perf_stat.sync_success     = true;
            perf_stat.lidar_start_time = m_package.cloud_start_time;
            perf_stat.lidar_end_time   = m_package.cloud_end_time;
            perf_stat.imu_count        = static_cast<int>(m_package.imus.size());
            
            if (!m_package.imus.empty()) {
                perf_stat.imu_first_time = m_package.imus.front().time;
                perf_stat.imu_last_time  = m_package.imus.back().time;
            }
            
            {
                std::lock_guard<std::mutex> lock(m_state_data.imu_mutex);
                perf_stat.imu_buffer_before_sync = m_state_data.imu_buffer.size();
                if (!m_state_data.imu_buffer.empty()) {
                    perf_stat.imu_buffer_first_time = m_state_data.imu_buffer.front().time;
                    perf_stat.imu_buffer_last_time  = m_state_data.imu_buffer.back().time;
                }
            }
            
            pcl::PointCloud<PointType>::Ptr cloud_in = m_package.cloud;
            bool should_publish = false;
            
            if (use_reloc_) {
                auto reg_start = std::chrono::high_resolution_clock::now();
                act_pose = localizier3d->location(act_pose, cloud_in);
                auto reg_end = std::chrono::high_resolution_clock::now();
                perf_stat.cloud_registration_time_ms = 
                    std::chrono::duration<double, std::milli>(reg_end - reg_start).count();
                
                if (localizier3d->fitness_ > (reloc_threshold_ * 1.2)) {
                    RCLCPP_WARN(nh_->get_logger(), 
                                "[重定位] 成功: fitness=%.3f, pos=(%.2f, %.2f, %.2f)",
                                localizier3d->fitness_, 
                                act_pose.pos(0), act_pose.pos(1), act_pose.pos(2));
                    
                    localizier3d->setNormalMode();
                    use_reloc_ = false;
                    should_publish = true;
                    
                    if (pose_refine_force_accept_reloc_) {
                        pose_refine_force_accept_next_ = true;
                    }
                    
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
            } else {
                if (!m_package.imus.empty()) {
                    undistortPointCloud(cloud_in, m_package.cloud_start_time,
                                       m_package.cloud_end_time, m_package.imus);
                }
                
                if (!m_package.imus.empty()) {
                    imu_predict(m_package.imus, perf_stat);
                }
                
                auto reg_start = std::chrono::high_resolution_clock::now();
                act_pose = localizier3d->location(act_pose, cloud_in);
                auto reg_end = std::chrono::high_resolution_clock::now();
                perf_stat.cloud_registration_time_ms = 
                    std::chrono::duration<double, std::milli>(reg_end - reg_start).count();
                
                if (!m_package.imus.empty()) {
                    auto eskf_start = std::chrono::high_resolution_clock::now();
                    eskf->correct(act_pose);
                    auto eskf_end = std::chrono::high_resolution_clock::now();
                    perf_stat.eskf_correct_time_ms = 
                        std::chrono::duration<double, std::milli>(eskf_end - eskf_start).count();
                }
                
                if (localizier3d->fitness_ < reloc_threshold_) {
                    RCLCPP_WARN(nh_->get_logger(), 
                                "[定位] Fitness过低 (%.3f < %.3f)，切换内部重定位，并呼叫 Node A 协助！",
                                localizier3d->fitness_, reloc_threshold_);
                    use_reloc_ = true;
                    localizier3d->setRelocMode();
                    should_publish = false;
                    if (call_node_a_pub_->get_subscription_count() > 0) {
                        geometry_msgs::msg::PoseWithCovarianceStamped help_msg;
                        help_msg.header.stamp = nh_->get_clock()->now();
                        help_msg.header.frame_id = map_frame_;
                        
                        // 使用迷失前最后一刻的位姿
                        help_msg.pose.pose.position.x = act_pose.pos(0);
                        help_msg.pose.pose.position.y = act_pose.pos(1);
                        help_msg.pose.pose.position.z = act_pose.pos(2);
                        
                        // 欧拉角转四元数
                        tf2::Quaternion q;
                        q.setRPY(act_pose.orient(0), act_pose.orient(1), act_pose.orient(2));
                        help_msg.pose.pose.orientation.x = q.x();
                        help_msg.pose.pose.orientation.y = q.y();
                        help_msg.pose.pose.orientation.z = q.z();
                        help_msg.pose.pose.orientation.w = q.w();
                        
                        call_node_a_pub_->publish(help_msg);
                    }
                } else {
                    should_publish = true;
                }
            }
            
            auto frame_end_time = std::chrono::high_resolution_clock::now();
            perf_stat.total_process_time_ms = 
                std::chrono::duration<double, std::milli>(frame_end_time - frame_start_time).count();
            
            if (should_publish) {
                if (std::isnan(act_pose.pos(0)) || std::isnan(act_pose.pos(1)) || 
                    std::isnan(act_pose.pos(2))) {
                    RCLCPP_ERROR(nh_->get_logger(), "[处理线程] 检测到NaN位姿,跳过发布");
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
                    
                    if (pose_publish_count_ % 20 == 0) {
                        RCLCPP_INFO(nh_->get_logger(),
                                   "[位姿发布] 第%d帧 | 时间戳=%.6f | pos=(%.3f, %.3f, %.3f) | "
                                   "rpy=(%.3f°, %.3f°, %.3f°)",
                                   pose_publish_count_,
                                   m_package.cloud_end_time,
                                   act_pose_pub.pos(0), act_pose_pub.pos(1), act_pose_pub.pos(2),
                                   act_pose_pub.orient(0) * 57.3, 
                                   act_pose_pub.orient(1) * 57.3, 
                                   act_pose_pub.orient(2) * 57.3);
                    }
                }
            }
            
            perf_history_.push_back(perf_stat);
            if (perf_history_.size() > perf_history_size_) {
                perf_history_.pop_front();
            }
            
            std::vector<IMUData>().swap(m_package.imus);
            m_package.cloud.reset();
            
            ++process_count;
            ++undistort_count_;
            
            double current_system_time = nh_->get_clock()->now().seconds();
            double cloud_to_system_delay = current_system_time - m_package.cloud_end_time;
            
            std_msgs::msg::Int8 status_msg;
            if (use_reloc_) {
                status_msg.data = 1; // 1 表示迷失 / 正在重定位
            } else {
                status_msg.data = 0; // 0 表示健康 / 正常跟踪
            }
            loc_status_pub_->publish(status_msg);
            
            RCLCPP_INFO(nh_->get_logger(),
                       "[第%d帧] 同步云末=%.6f | 系统时间=%.6f | "
                       "数据延迟=%.3fs(%.0fms) | 算法=%.1fms | IMU=%d个 | 配准=%.1fms | "
                       "fitness=%.3f | pos=(%.2f,%.2f,%.2f) | yaw=%.1f°",
                       process_count,
                       m_package.cloud_end_time, current_system_time,
                       cloud_to_system_delay, cloud_to_system_delay * 1000.0,
                       perf_stat.total_process_time_ms, perf_stat.imu_count,
                       perf_stat.cloud_registration_time_ms,
                       localizier3d->fitness_,
                       act_pose.pos(0), act_pose.pos(1), act_pose.pos(2),
                       act_pose.orient(2) * 57.3);
        }
        
        RCLCPP_INFO(nh_->get_logger(), "[处理线程] 退出");
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    
    signal(SIGINT, SigHandle);
    
    RCLCPP_INFO(rclcpp::get_logger("lidar_location_3d"), 
                "\033[1;32m---->\033[0m Lidar Location 3D Started (多线程+数据驱动模式)");
    
    auto nh = rclcpp::Node::make_shared("lidar_location_3d");
    
    location loc(nh);

    rclcpp::executors::MultiThreadedExecutor executor(
        rclcpp::ExecutorOptions(), 
        2
    );
    executor.add_node(nh);
    executor.spin();

    RCLCPP_INFO(rclcpp::get_logger("lidar_location_3d"), "正在关闭...");
    
    rclcpp::shutdown();
    
    RCLCPP_INFO(rclcpp::get_logger("lidar_location_3d"), "已安全退出");
    return 0;
}
