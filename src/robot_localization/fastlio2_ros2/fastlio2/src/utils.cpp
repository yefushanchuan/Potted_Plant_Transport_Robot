#include "utils.h"

namespace velodyne_ros
{
    struct EIGEN_ALIGN16 Point
    {
        PCL_ADD_POINT4D;
        float intensity;
        double timestamp;
        uint16_t ring;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
} // namespace velodyne_ros

POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_ros::Point,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(double, timestamp, timestamp)(uint16_t, ring, ring))

namespace livox_ros
{
    struct EIGEN_ALIGN16 Point
    {
        PCL_ADD_POINT4D;
        float intensity;
        uint8_t tag;
        uint8_t line;
        double timestamp;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
} // namespace livox_ros

POINT_CLOUD_REGISTER_POINT_STRUCT(livox_ros::Point,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint8_t, tag, tag)(uint8_t, line, line)(double, timestamp, timestamp))

/* 将 Livox Mid360 的 PointCloud2 转换为 PCL 点云 (pcl::PointCloud<pcl::PointXYZINormal>)
 参数说明：
 - msg: Livox 雷达订阅得到的 PointCloud2 消息
 - filter_num: 点云采样间隔，每隔 filter_num 个点保留一个，用于下采样
 - min_range: 点云最小距离阈值（低于此值的点会被过滤）
 - max_range: 点云最大距离阈值（超过此值的点会被过滤）
 - z_min: Z轴最小高度（低于此值的点会被过滤）
 - z_max: Z轴最大高度（超过此值的点会被过滤）*/
pcl::PointCloud<pcl::PointXYZINormal>::Ptr Utils::livox2PCL(const sensor_msgs::msg::PointCloud2 &msg,
                                                            int filter_num, double min_range, double max_range, double z_min, double z_max, Eigen::Matrix3d R, Eigen::Vector3d t)
{
    // 创建一个 PCL 点云对象（带强度和曲率信息）
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZINormal>);

    // 将 ROS PointCloud2 转换为 PCL 格式
    pcl::PointCloud<livox_ros::Point> input_cloud;
    pcl::fromROSMsg(msg, input_cloud);

    // 获取点的总数
    int point_num = input_cloud.size();

    // 预分配内存，加快 push_back 速度
    cloud->reserve(point_num / filter_num + 1);

    // 遍历点云，按 filter_num 进行下采样
    for (int i = 0; i < point_num; i += filter_num)
    {
        // 且只保留 tag 符合 0x10 或 0x00 的点（过滤异常或特殊点,只保留 line < 4 的点（前四线，Mid360 有多线激光）
        if ((input_cloud.points[i].line < 4) && ((input_cloud.points[i].tag & 0x30) == 0x10 || (input_cloud.points[i].tag & 0x30) == 0x00))
        {

            float x = input_cloud.points[i].x; // 获取点坐标
            float y = input_cloud.points[i].y;
            float z = input_cloud.points[i].z;

            // 构造 PCL 点
            pcl::PointXYZINormal p_new;

            // 计算相对时间（转换为纳秒）
            double rel_time = (input_cloud.points[i].timestamp - input_cloud.points[0].timestamp) * 1e-6; // 原始livox的时间是纳秒为单位的
            p_new.curvature = rel_time;                                                                   // 使用秒为单位的相对时间
            p_new.intensity = input_cloud.points[i].intensity;                                            // 点的反射强度

            // 空间变换
            Eigen::Vector3d point_in(x, y, z);
            Eigen::Vector3d point_out = R * point_in + t;
            p_new.x = point_out.x();
            p_new.y = point_out.y();
            p_new.z = point_out.z();
            
            // 打印每个点的Z轴变换信息（调试用）
            // RCLCPP_INFO(rclcpp::get_logger("Utils_Livox"), 
            //            "Point %d - Original z: %.3f -> Transformed z: %.3f", 
            //            i, z, p_new.z);

            float dist2 = p_new.x * p_new.x + p_new.y * p_new.y + p_new.z * p_new.z; // 计算点到原点距离平方

            // 根据 min_range 和 max_range 过滤掉过近或过远的点，同时根据 z_min 和 z_max 过滤Z轴范围
            if (dist2 < min_range * min_range || dist2 > max_range * max_range || p_new.z < z_min || p_new.z > z_max)
                continue;

            // std::cout << "curvature: " << p_new.curvature << std::endl;

            // 加入点云
            cloud->push_back(p_new);
        }
    }
    // RCLCPP_INFO(rclcpp::get_logger("Utils LIVOX_MID360convertAndComputeNormals close"), "convertAndComputeNormals IN ");
    //  返回生成的 PCL 点云
    return cloud;
}

double Utils::getSec(std_msgs::msg::Header &header)
{
    return static_cast<double>(header.stamp.sec) + static_cast<double>(header.stamp.nanosec) * 1e-9;
}
builtin_interfaces::msg::Time Utils::getTime(const double &sec)
{
    builtin_interfaces::msg::Time time_msg;
    time_msg.sec = static_cast<int32_t>(sec);
    time_msg.nanosec = static_cast<uint32_t>((sec - time_msg.sec) * 1e9);
    return time_msg;
}

pcl::PointCloud<pcl::PointXYZINormal>::Ptr Utils::convertAndComputeNormals(const sensor_msgs::msg::PointCloud2 &input_cloud_msg,
                                                                           int filter_num, double min_range, double max_range, double z_min, double z_max, Eigen::Matrix3d R, Eigen::Vector3d t)
{

    // RCLCPP_INFO(rclcpp::get_logger("Utils"), "convertAndComputeNormals IN ");
    pcl::PointCloud<velodyne_ros::Point> input_cloud;
    // 转换点云为速腾雷达数据格式
    pcl::fromROSMsg(input_cloud_msg, input_cloud);
    // velodyne_ros->PointXYZINormal
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
    int point_num = input_cloud.size();
    cloud->reserve(point_num / filter_num + 1);
    for (int i = 0; i < point_num; i += filter_num)
    {
        float x = input_cloud.points[i].x;
        float y = input_cloud.points[i].y;
        float z = input_cloud.points[i].z;

        pcl::PointXYZINormal p;
        p.x = x;
        p.y = y;
        p.z = z;
        p.curvature = (input_cloud.points[i].timestamp - input_cloud.points[0].timestamp) * 1000.0f;
        p.intensity = input_cloud.points[i].intensity;

        Eigen::Vector3d point_in(p.x, p.y, p.z);
        Eigen::Vector3d point_out = R * point_in + t; // 增加旋转平移

        pcl::PointXYZINormal p_new = p;
        p_new.x = point_out.x();
        p_new.y = point_out.y();
        p_new.z = point_out.z();
        p_new.intensity = input_cloud.points[i].intensity; // 点的反射强度
        p_new.curvature = (input_cloud.points[i].timestamp - input_cloud.points[0].timestamp) * 1000.0f;
        
        float dist2 = p_new.x * p_new.x + p_new.y * p_new.y + p_new.z * p_new.z; // 计算点到原点距离平方
        if (dist2 < min_range * min_range || dist2 > max_range * max_range || p_new.z < z_min || p_new.z > z_max)
            continue;

        // 加入点云
        cloud->push_back(p_new);
    }

    // RCLCPP_INFO(rclcpp::get_logger("Utils RS16convertAndComputeNormals close"), "convertAndComputeNormals IN ");
    return cloud;
}
