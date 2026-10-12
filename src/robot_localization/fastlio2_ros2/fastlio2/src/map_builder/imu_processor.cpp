#include "imu_processor.h"
#include "timed_motion.h"
#include <rclcpp/rclcpp.hpp>
#include <sstream>

// 构造函数：初始化状态变量和噪声协方差矩阵 Q
IMUProcessor::IMUProcessor(Config &config, std::shared_ptr<IESKF> kf): m_config(config), m_kf(kf)
{
    // 初始化系统协方差矩阵 Q（用于IMU预积分中的过程噪声）
    m_Q.setIdentity();
    m_Q.block<3, 3>(0, 0) = M3D::Identity() * m_config.ng*1.5 ;         // gyro noise
    m_Q.block<3, 3>(3, 3) = M3D::Identity() * m_config.na ;             // acc noise
    m_Q.block<3, 3>(6, 6) = M3D::Identity() * m_config.nbg*2.0;         // gyro bias noise
    m_Q.block<3, 3>(9, 9) = M3D::Identity() * m_config.nba;             // acc bias noise
    // 可针对Z轴调低na：
    m_Q(5, 5) *= 5.0;   // 增加Z轴线加速度噪声，让其不信任z轴的噪声

    m_imu_cache.clear();    // 清空IMU缓存
}

bool IMUProcessor::initialize(SyncPackage &package)
{
    // 累积IMU数据用于初始估计
    for (const auto &imu : package.imus)
        if (imu.time <= package.cloud_end_time &&
            (m_imu_cache.empty() || imu.time > m_imu_cache.back().time))
            m_imu_cache.push_back(imu);

    // 未满初始化所需IMU数量，不初始化
    if (m_imu_cache.size() < static_cast<size_t>(m_config.imu_init_num))
        return false;

    V3D acc_mean = V3D::Zero();   // 加速度平均值
    V3D gyro_mean = V3D::Zero();  // 陀螺仪平均值

    // 计算平均加速度和陀螺仪偏置
    for (const auto &imu : m_imu_cache)
    {
        acc_mean += imu.acc;
        gyro_mean += imu.gyro;
    }
    acc_mean /= static_cast<double>(m_imu_cache.size());
    gyro_mean /= static_cast<double>(m_imu_cache.size());

    // 设置 IMU 到雷达的外参
    m_kf->x().r_il = m_config.r_il;
    m_kf->x().t_il = m_config.t_il;

    // 设置初始的陀螺仪 bias 为 gyro 平均值
    m_kf->x().bg = gyro_mean;

     //重力方向对齐
    if (m_config.gravity_align)
    {
        // 使用 Eigen 中 FromTwoVectors 将 -acc_mean 对齐到 z 轴负方向
        m_kf->x().r_wi = (Eigen::Quaterniond::FromTwoVectors((-acc_mean).normalized(), V3D(0.0, 0.0, -1.0)).matrix());
        m_kf->x().initGravityDir(V3D(0, 0, -1.0));
    }
    else{
        m_kf->x().initGravityDir(-acc_mean);  // 使用加速度平均值直接估计重力
    }
    // IMU初始化状态详细监控
    if (m_config.enable_debug_logs) {
        RCLCPP_INFO(rclcpp::get_logger("imu_processor"), 
                   "[IMU初始化] ==================== 初始化报告 ====================");
        RCLCPP_INFO(rclcpp::get_logger("imu_processor"), 
                   "[IMU初始化] 使用样本数: %zu帧, 要求最少: %d帧", 
                   m_imu_cache.size(), m_config.imu_init_num);
        RCLCPP_INFO(rclcpp::get_logger("imu_processor"), 
                   "[IMU初始化] 加速度均值: [%.4f,%.4f,%.4f] m/s², 模长: %.4f (期望值: ~9.81)", 
                   acc_mean.x(), acc_mean.y(), acc_mean.z(), acc_mean.norm());
        RCLCPP_INFO(rclcpp::get_logger("imu_processor"), 
                   "[IMU初始化] 陀螺仪零偏: [%.6f,%.6f,%.6f] rad/s", 
                   gyro_mean.x(), gyro_mean.y(), gyro_mean.z());
    }
    
    // 检查加速度计读数是否异常
    double gravity_error = std::abs(acc_mean.norm() - 9.81);
    if (gravity_error > 2.0) {
        RCLCPP_WARN(rclcpp::get_logger("imu_processor"), 
                   "[IMU初始化] 警告: 重力加速度误差 %.2f m/s² (>2.0)，请检查IMU标定！", 
                   gravity_error);
    }
    
    // 检查陀螺仪偏置是否过大
    if (gyro_mean.norm() > 0.1) {
        RCLCPP_WARN(rclcpp::get_logger("imu_processor"), 
                   "[IMU初始化] 警告: 陀螺仪零偏过大 %.4f rad/s (>0.1)，IMU可能需要重新标定！", 
                   gyro_mean.norm());
    }
    
    if (m_config.enable_debug_logs) {
        // 将 Eigen 矩阵转换为字符串
        std::stringstream ss;
        ss << Eigen::MatrixXd(m_kf->x().r_wi).format(Eigen::IOFormat(3, 0, " ", "\n", "[", "]"));
        RCLCPP_INFO(rclcpp::get_logger("imu_processor"), "[IMU初始化] 初始旋转矩阵 R_wi:\n%s", ss.str().c_str());
        RCLCPP_INFO(rclcpp::get_logger("imu_processor"), 
                   "[IMU初始化] 重力向量: [%.4f,%.4f,%.4f] m/s²", 
                   m_kf->x().g.x(), m_kf->x().g.y(), m_kf->x().g.z());
        RCLCPP_INFO(rclcpp::get_logger("imu_processor"), 
                   "[IMU初始化] 重力对齐: %s", 
                   m_config.gravity_align ? "已启用(对齐到-Z轴)" : "已禁用(使用测量值)");
    }
    RCLCPP_INFO(rclcpp::get_logger("imu_processor"), 
               "[IMU初始化] ================= 初始化完成 =================");
    // 设置初始状态协方差矩阵
    m_kf->P().setIdentity();
    m_kf->P().block<3, 3>(6, 6)   = M3D::Identity() * 0.00001; // 外参旋转不确定性
    m_kf->P().block<3, 3>(9, 9)   = M3D::Identity() * 0.00001; // 外参平移不确定性
    m_kf->P().block<3, 3>(15, 15) = M3D::Identity() * 0.0001;  // 陀螺仪偏置不确定性
    m_kf->P().block<3, 3>(18, 18) = M3D::Identity() * 0.0001;  // 加速度计偏置不确定性

    // 设置上一次使用的IMU和时间戳
    m_kf->x().time = package.cloud_end_time;
    m_kf->x().td = m_config.time_offset.initial_td;
    m_kf->time_options = m_config.time_offset;
    m_kf->P()(21, 21) = std::pow(m_config.time_offset.initial_td_std, 2);

    return true;
}

bool IMUProcessor::undistort(SyncPackage &package) {

    // Keep the anchor on the IMU clock, independent of the current td estimate.
    fast_time::Trajectory check;
    auto &x = m_kf->x();
    const double end = package.cloud_end_time;
    if (end < x.time || end - x.time > 1. ||
        !check.build(x, package.imus, std::min(x.time, package.cloud_start_time + x.td),
                     std::max(end, end + x.td), m_config.time_offset.max_imu_gap)) {
        m_kf->report = TimeOffsetReport();
        m_kf->report.reason = "imu_coverage";
        return false;
    }
    std::vector<double> targets;
    for (const auto &imu : package.imus)
        if (imu.time > x.time && imu.time < end)
            targets.push_back(imu.time);
    targets.push_back(end);
    for (double target : targets) {
        IMUData a, b;
        fast_time::interpolate(package.imus, x.time, m_config.time_offset.max_imu_gap, a);
        fast_time::interpolate(package.imus, target, m_config.time_offset.max_imu_gap, b);
        Input inp;
        inp.acc = .5 * (a.acc + b.acc);
        inp.gyro = .5 * (a.gyro + b.gyro);
        m_kf->predict(inp, target - x.time, m_Q);
        x.time = target;
    }
    V3D a = V3D::Zero(), w = a;
    int count = 0;
    for (const auto &imu : package.imus)
        if (imu.time >= end - .5 && imu.time <= end) {
            a += imu.acc;
            w += imu.gyro;
            ++count;
        }
    double va = 0, vw = 0;
    if (count > 1) {
        a /= count;
        w /= count;
        for (const auto &imu : package.imus)
            if (imu.time >= end - .5 && imu.time <= end) {
                va += (imu.acc - a).squaredNorm();
                vw += (imu.gyro - w).squaredNorm();
            }
    }
    m_kf->time_excited =
        count > 1 && (std::sqrt(va / count) >= m_config.time_offset.min_accel_std ||
                      std::sqrt(vw / count) >= m_config.time_offset.min_gyro_std);
    package.raw_cloud.reset(new CloudType(*package.cloud));
    return deskewNative(x, package.imus, *package.raw_cloud, package.cloud_start_time,
                        *package.cloud, m_config.time_offset.max_imu_gap);
}
