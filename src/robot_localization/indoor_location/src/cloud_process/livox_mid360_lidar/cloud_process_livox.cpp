#include "cloud_process_livox.h"
#include <pcl_conversions/pcl_conversions.h>
// 引入 Eigen 核心库，以支持点乘加运算
#include <Eigen/Dense> 

cloud_process_livox::cloud_process_livox(double leaf_size){
    down_size_filter_.setLeafSize(leaf_size, leaf_size, leaf_size);
}

pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_process_livox::livox_cloud_handler(const sensor_msgs::msg::PointCloud2::SharedPtr &msg)
{
    // 创建输出点云（带强度和曲率/时间戳信息）
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr pl_full(new pcl::PointCloud<pcl::PointXYZINormal>);
    pl_full->clear();

    pcl::PointCloud<livox_ros::Point>::Ptr livox_cloud(new pcl::PointCloud<livox_ros::Point>);
    pcl::fromROSMsg(*msg, *livox_cloud);
    
    last_scan_duration_sec_ = 0.0;
    if (!livox_cloud->points.empty())
    {
        last_scan_duration_sec_ = (livox_cloud->points.back().timestamp - livox_cloud->points.front().timestamp) * 1e-9;
        if (last_scan_duration_sec_ < 0.0)
            last_scan_duration_sec_ = 0.0;
    }

    // 提前记录本帧基准时间，防止如果第0个点被滤除导致时间轴跳动
    uint64_t base_timestamp = livox_cloud->points[0].timestamp;

    for(size_t i = 0; i < livox_cloud->points.size(); i++)
    {
        const auto& pt = livox_cloud->points[i];

        // 1. 检查线束和标签过滤条件
        if((pt.line < N_SCANS_) && ((pt.tag & 0x30) == 0x10 || (pt.tag & 0x30) == 0x00))
        {
            // 2. 距离和角度过滤（必须使用雷达自身坐标系下的原始值！）
            double dist_sq = pt.x * pt.x + pt.y * pt.y + pt.z * pt.z;
            if(dist_sq < min_range_ * min_range_ || dist_sq > max_range_ * max_range_) {
                continue;
            }

            double angle = atan2(pt.y, pt.x);
            if(angle < min_angle_ || angle > max_angle_) {
                continue;
            }

            // 3. 应用坐标变换，将 Lidar 系下的点转到 base_link 系下（使用 Eigen 更安全高效）
            Eigen::Vector3f pt_lidar(pt.x, pt.y, pt.z);
            Eigen::Vector3f pt_base = base2link_rot * pt_lidar + base2link_trans;
            
            // 4. 创建单个点（带强度和相对时间戳）
            pcl::PointXYZINormal p_new;
            p_new.x = pt_base.x();
            p_new.y = pt_base.y();
            p_new.z = pt_base.z();
            p_new.intensity = pt.intensity;
            
            // curvature 字段存储相对时间戳（毫秒）：Livox 时间戳为纳秒，这里换算为毫秒
            double rel_time_ms = (pt.timestamp - base_timestamp) * 1e-6;
            p_new.curvature = static_cast<float>(rel_time_ms);
            
            p_new.normal_x = 0.0f;
            p_new.normal_y = 0.0f;
            p_new.normal_z = 0.0f;
            
            pl_full->push_back(p_new);
        }
    }

    // 体素滤波降采样（保留时间戳信息）
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr res_cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
    res_cloud->clear();

    down_size_filter_.setInputCloud(pl_full);
    down_size_filter_.filter(*res_cloud);

    res_cloud->width = res_cloud->points.size();
    res_cloud->height = 1;

    return res_cloud;
}
