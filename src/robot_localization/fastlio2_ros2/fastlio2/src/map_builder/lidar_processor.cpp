#include "lidar_processor.h"
#include "timed_motion.h"
#include <set>
#include <map>
#include <limits>
#include <tuple>
#include <atomic>
#include <rclcpp/rclcpp.hpp>

//初始化点云处理器，创建必要的数据结构，并将滤波器配置好
LidarProcessor::LidarProcessor(Config &config, std::shared_ptr<IESKF> kf) : m_config(config), m_kf(kf)
{
    m_ikdtree = std::make_shared<KD_TREE<PointType>>();
    m_ikdtree->set_downsample_param(m_config.map_resolution);//创建 KD 树对象，并设置降采样体素大小（地图分辨率）
    // 分配内存,创建点云缓存，包括降采样后点云、转换后的点云、法向量、有效特征点等
    m_cloud_down_lidar.reset(new CloudType);
    m_cloud_down_world.reset(new CloudType(10000, 1));
    m_norm_vec.reset(new CloudType(10000, 1));
    m_effect_cloud_lidar.reset(new CloudType(10000, 1));
    m_effect_norm_vec.reset(new CloudType(10000, 1));
    //创建每个点的邻近点容器和是否被选中标志位
    m_nearest_points.resize(10000);
    m_point_selected_flag.resize(10000, false);
    //如果设置了激光点云降采样分辨率，则初始化滤波器
    if (m_config.scan_down_sampling_rate > 0.0)
    {
        m_scan_filter.setLeafSize(m_config.scan_down_sampling_rate, m_config.scan_down_sampling_rate, m_config.scan_down_sampling_rate);
    }
    // 设置滤波器的代价项函数和迭代停止准则
    m_kf->setLossFunction([&](State &s, SharedState &d)
                          { updateLossFunc(s, d); });
    m_kf->setStopFunction([&](const V22D &delta) -> bool {
        V3D rot_delta = delta.block<3, 1>(0, 0);
        V3D t_delta = delta.block<3, 1>(3, 0);

        static int iter_count = 0;
        iter_count++;

        double rot_deg = rot_delta.norm() * 57.3;
        double t_cm = t_delta.norm() * 100.0;
        bool converged = (rot_deg < m_config.ieskf_rotation_threshold_deg) &&
                         (t_cm < m_config.ieskf_translation_threshold_cm);

        // 每次迭代都记录，用于分析收敛过程
        if (m_config.enable_debug_logs && (iter_count % 5 == 0 || converged)) {
            RCLCPP_INFO(rclcpp::get_logger("lidar_processor"),
                        "[IESKF迭代 %d] 增量: 旋转=%.4f°(阈值=%.2f°), 平移=%.2fcm(阈值=%.1fcm) %s",
                        iter_count, rot_deg, m_config.ieskf_rotation_threshold_deg, t_cm,
                        m_config.ieskf_translation_threshold_cm,
                        converged ? "已收敛" : "继续迭代...");
        }

        // 检查收敛异常
        if (iter_count > 20 && !converged) {
            if (m_config.enable_debug_logs) {
                RCLCPP_WARN(rclcpp::get_logger("lidar_processor"),
                            "[IESKF警告] 收敛缓慢，已迭代%d次，增量: %.4f°/%.2fcm", iter_count,
                            rot_deg, t_cm);
            }
        }

        if (converged) {
            if (m_config.enable_debug_logs) {
                RCLCPP_INFO(rclcpp::get_logger("lidar_processor"), "[IESKF成功] 在%d次迭代后收敛",
                            iter_count);
            }
            iter_count = 0; // 重置计数器
        }

        return converged;
    });
}
//地图裁剪
void LidarProcessor::trimCloudMap()
{
    // 清空上一帧需要删除的立方体列表
    m_local_map.cub_to_rm.clear();

    // 获取当前状态（位姿 + 速度 + 偏置等），这里只用位姿部分
    const State &state = m_kf->x();

    // 计算 Lidar 在世界坐标系中的位置：pos_lidar = Lidar 相对于 IMU 的位移 + IMU 在世界坐标系的位姿
    Eigen::Vector3d pos_lidar = state.t_wi + state.r_wi * state.t_il;

    // -------------------- 地图初始化 --------------------
    if (!m_local_map.initialed)
    {
        // 如果局部地图尚未初始化
        // 以当前 Lidar 位置为中心，创建一个立方体局部地图
        for (int i = 0; i < 3; i++)
        {
            m_local_map.local_map_corner.vertex_min[i] = pos_lidar[i] - m_config.cube_len / 2.0;
            m_local_map.local_map_corner.vertex_max[i] = pos_lidar[i] + m_config.cube_len / 2.0;
        }
        m_local_map.initialed = true;
        return; // 地图初始化完成后直接返回
    }

    // -------------------- 判断是否需要移动局部地图 --------------------
    float dist_to_map_edge[3][2]; // 存储 Lidar 与局部地图六个面的距离
    bool need_move = false;        // 标记是否需要移动局部地图
    double det_thresh = m_config.move_thresh * m_config.det_range; // 移动阈值

    for (int i = 0; i < 3; i++)
    {
        // 计算 Lidar 到每个边界的距离
        dist_to_map_edge[i][0] = fabs(pos_lidar(i) - m_local_map.local_map_corner.vertex_min[i]);
        dist_to_map_edge[i][1] = fabs(pos_lidar(i) - m_local_map.local_map_corner.vertex_max[i]);

        // 如果 Lidar 靠近任意边界，则需要移动局部地图
        if (dist_to_map_edge[i][0] <= det_thresh || dist_to_map_edge[i][1] <= det_thresh)
            need_move = true;
    }

    // 如果 Lidar 离边界较远，不需要移动
    if (!need_move)
        return;

    // -------------------- 计算新的局部地图立方体 --------------------
    BoxPointType new_corner, temp_corner;
    new_corner = m_local_map.local_map_corner;

    // 计算移动距离
    float mov_dist = std::max((m_config.cube_len - 2.0 * m_config.move_thresh * m_config.det_range) * 0.5 * 0.9,double(m_config.det_range * (m_config.move_thresh - 1))
    );

    for (int i = 0; i < 3; i++)
    {
        temp_corner = m_local_map.local_map_corner;

        // 如果靠近最小边界，局部地图向负方向移动 mov_dist
        if (dist_to_map_edge[i][0] <= det_thresh)
        {
            new_corner.vertex_max[i] -= mov_dist;
            new_corner.vertex_min[i] -= mov_dist;

            // 将将要被删除的立方体记录下来，用于后续点云删除
            temp_corner.vertex_min[i] = m_local_map.local_map_corner.vertex_max[i] - mov_dist;
            m_local_map.cub_to_rm.push_back(temp_corner);
        }
            // 如果靠近最大边界，局部地图向正方向移动 mov_dist
        else if (dist_to_map_edge[i][1] <= det_thresh)
        {
            new_corner.vertex_max[i] += mov_dist;
            new_corner.vertex_min[i] += mov_dist;

            temp_corner.vertex_max[i] = m_local_map.local_map_corner.vertex_min[i] + mov_dist;
            m_local_map.cub_to_rm.push_back(temp_corner);
        }
    }

    // 更新局部地图边界
    m_local_map.local_map_corner = new_corner;

    // -------------------- 删除局部地图之外的点云 --------------------
    PointVec points_history;
    m_ikdtree->acquire_removed_points(points_history); // 获取历史被删除的点（如果有缓存机制）

    // 如果有需要删除的立方体，则从 KD 树中删除对应点
    if (m_local_map.cub_to_rm.size() > 0)
        m_ikdtree->Delete_Point_Boxes(m_local_map.cub_to_rm);

    return;
}

//增量更新地图
void LidarProcessor::incrCloudMap()
{
    if (m_cloud_down_lidar->empty())
        return;//若本帧点云为空，则跳过
    const State &state = m_kf->x();
    int size = m_cloud_down_lidar->size();
    PointVec point_to_add;
    PointVec point_no_need_downsample;
/*  将每个点转换到世界坐标系，并判断是否要加入地图：
    若附近没有点 => 一定加入
    若所在体素无近邻点 => 不降采样直接加入
    若近邻点距离中心更近 => 不加入
    否则加入*/
    for (int i = 0; i < size; i++)
    {
        const PointType &p = m_cloud_down_lidar->points[i];
        Eigen::Vector3d point(p.x, p.y, p.z);
        point = state.r_wi * (state.r_il * point + state.t_il) + state.t_wi;
        m_cloud_down_world->points[i].x = point(0);
        m_cloud_down_world->points[i].y = point(1);
        m_cloud_down_world->points[i].z = point(2);
        m_cloud_down_world->points[i].intensity = m_cloud_down_lidar->points[i].intensity;
        // 如果该点附近没有近邻点则需要添加到地图中
        if (m_nearest_points[i].empty())
        {
            point_to_add.push_back(m_cloud_down_world->points[i]);
            continue;
        }

        const PointVec &points_near = m_nearest_points[i];
        bool need_add = true;
        PointType downsample_result, mid_point;
        mid_point.x = std::floor(m_cloud_down_world->points[i].x / m_config.map_resolution) * m_config.map_resolution + 0.5 * m_config.map_resolution;
        mid_point.y = std::floor(m_cloud_down_world->points[i].y / m_config.map_resolution) * m_config.map_resolution + 0.5 * m_config.map_resolution;
        mid_point.z = std::floor(m_cloud_down_world->points[i].z / m_config.map_resolution) * m_config.map_resolution + 0.5 * m_config.map_resolution;

        // 如果该点所在的voxel没有点，则直接加入地图，且不需要降采样
        if (fabs(points_near[0].x - mid_point.x) > 0.5 * m_config.map_resolution && fabs(points_near[0].y - mid_point.y) > 0.5 * m_config.map_resolution && fabs(points_near[0].z - mid_point.z) > 0.5 * m_config.map_resolution)
        {
            point_no_need_downsample.push_back(m_cloud_down_world->points[i]);
            continue;
        }
        float dist = sq_dist(m_cloud_down_world->points[i], mid_point);

        for (int readd_i = 0; readd_i < m_config.near_search_num; readd_i++)
        {
            // 如果该点的近邻点较少，则需要加入到地图中
            if (points_near.size() < static_cast<size_t>(m_config.near_search_num))
                break;
            // 如果该点的近邻点距离voxel中心点的距离比该点距离voxel中心点更近，则不需要加入该点
            if (sq_dist(points_near[readd_i], mid_point) < dist)
            {
                need_add = false;
                break;
            }
        }
        if (need_add)
            point_to_add.push_back(m_cloud_down_world->points[i]);
    }
    m_ikdtree->Add_Points(point_to_add, true);
    m_ikdtree->Add_Points(point_no_need_downsample, false);
}

void LidarProcessor::initCloudMap(PointVec &point_vec)//用已有点云初始化地图
{
    m_ikdtree->Build(point_vec);
}

void LidarProcessor::process(SyncPackage &package)
{
    auto start_time = std::chrono::high_resolution_clock::now();

    int original_size = package.raw_cloud->size();
    m_package = &package;
    m_motion_anchor = m_kf->x();
    m_scan_filter.setSaveLeafLayout(true);
    m_scan_filter.setInputCloud(package.cloud);
    m_scan_filter.filter(*m_cloud_down_lidar);
    m_groups.assign(m_cloud_down_lidar->size(),{});
    for(size_t i=0;i<package.cloud->size();++i) {
        const int k=m_scan_filter.getCentroidIndex((*package.cloud)[i]);
        if(k<0 || k>=int(m_groups.size())) throw std::runtime_error("Voxel membership mismatch");
        m_groups[k].push_back(i);
    }
    for(const auto& group:m_groups) if(group.empty()) throw std::runtime_error("Empty voxel");
    m_query_begin=package.cloud_end_time;m_query_end=package.cloud_start_time;
    for(const auto& p:*package.raw_cloud) {
        const double t=package.cloud_start_time+p.curvature*.001;
        m_query_begin=std::min(m_query_begin,t);m_query_end=std::max(m_query_end,t);
    }
    m_linearization_valid = false; // Cache is never carried across scans.
    m_nearest_points.resize(m_cloud_down_lidar->size());
    m_cloud_down_world->resize(m_cloud_down_lidar->size());
    int processed_size = m_cloud_down_lidar->size();
    
    // 地图裁剪
    auto trim_start = std::chrono::high_resolution_clock::now();
    trimCloudMap();
    auto trim_end = std::chrono::high_resolution_clock::now();
    
    // IESKF 更新
    auto kf_start = std::chrono::high_resolution_clock::now();
    const bool accepted = m_kf->update();
    auto kf_end = std::chrono::high_resolution_clock::now();
    
    // 地图增量更新
    auto map_start = std::chrono::high_resolution_clock::now();
    // The same PCL voxel members are used in observation and map insertion.
    State deskew_anchor=m_kf->x();
    fast_time::Trajectory final_trajectory;
    const bool covered=final_trajectory.build(deskew_anchor,package.imus,
        m_query_begin+deskew_anchor.td,m_query_end+deskew_anchor.td,
        m_config.time_offset.max_imu_gap);
    if(accepted && covered) {
        for(size_t i=0;i<m_groups.size();++i) {
            Eigen::Vector3d world_mean=Eigen::Vector3d::Zero();
            for(size_t k:m_groups[i]) {
                const auto& p=(*package.raw_cloud)[k];Eigen::Vector3d world;
                if(!final_trajectory.point({Eigen::Vector3d(p.x,p.y,p.z),package.cloud_start_time+p.curvature*.001},deskew_anchor.td,world))
                    throw std::runtime_error("Missing final point");
                world_mean+=world;
            }
            world_mean/=double(m_groups[i].size());
            const Eigen::Vector3d local=deskew_anchor.r_il.transpose()*(deskew_anchor.r_wi.transpose()*(world_mean-deskew_anchor.t_wi)-deskew_anchor.t_il);
            auto& p=(*m_cloud_down_lidar)[i];p.x=local.x();p.y=local.y();p.z=local.z();
        }
        incrCloudMap();
    }
    deskewNative(deskew_anchor,package.imus,*package.raw_cloud,package.cloud_start_time,
                 *package.cloud,m_config.time_offset.max_imu_gap);
    auto map_end = std::chrono::high_resolution_clock::now();
    
    // 计算各模块耗时
    auto total_end = std::chrono::high_resolution_clock::now();
    double trim_time = std::chrono::duration<double, std::milli>(trim_end - trim_start).count();
    double kf_time = std::chrono::duration<double, std::milli>(kf_end - kf_start).count();
    double map_time = std::chrono::duration<double, std::milli>(map_end - map_start).count();
    double total_time = std::chrono::duration<double, std::milli>(total_end - start_time).count();
    
    // LiDAR性能和质量监控日志
    static int lidar_log_counter = 0;
    static double total_time_sum = 0.0, kf_time_sum = 0.0, map_time_sum = 0.0;
    static double max_total_time = 0.0, max_kf_time = 0.0;
    static int downsample_ratio_sum = 0;
    
    // 累积统计
    total_time_sum += total_time;
    kf_time_sum += kf_time;
    map_time_sum += map_time;
    max_total_time = std::max(max_total_time, total_time);
    max_kf_time = std::max(max_kf_time, kf_time);
    
    double downsample_ratio = (original_size > 0) ? (double)processed_size / original_size * 100.0 : 0.0;
    downsample_ratio_sum += downsample_ratio;
    
    if (m_config.enable_debug_logs && ++lidar_log_counter % m_config.lidar_log_interval == 0) {
        double avg_total = total_time_sum / m_config.lidar_log_interval;
        double avg_kf = kf_time_sum / m_config.lidar_log_interval;
        double avg_map = map_time_sum / m_config.lidar_log_interval;
        double avg_downsample = downsample_ratio_sum / m_config.lidar_log_interval;
        
        RCLCPP_INFO(rclcpp::get_logger("lidar_processor"), 
                   "[激光性能] ===== 处理性能报告 (第%d帧) =====", lidar_log_counter);
        RCLCPP_INFO(rclcpp::get_logger("lidar_processor"), 
                   "[激光性能] 点云数量: %d->%d (保留%.1f%%)，降采样率: %.2f",
                   original_size, processed_size, downsample_ratio, m_config.scan_down_sampling_rate);
        RCLCPP_INFO(rclcpp::get_logger("lidar_processor"), 
                   "[激光性能] 本帧耗时: 总计=%.1fms (IESKF=%.1fms, 地图=%.1fms, 裁剪=%.1fms)",
                   total_time, kf_time, map_time, trim_time);
        RCLCPP_INFO(rclcpp::get_logger("lidar_processor"), 
                   "[激光性能] 平均耗时: 总计=%.1fms, IESKF=%.1fms, 地图=%.1fms, 点云保留=%.0f%%",
                   avg_total, avg_kf, avg_map, avg_downsample);
        RCLCPP_INFO(rclcpp::get_logger("lidar_processor"), 
                   "[激光性能] 峰值耗时: 总计=%.1fms, IESKF=%.1fms",
                   max_total_time, max_kf_time);
        
        // 性能警告检查
        if (avg_total > 100.0) {
            RCLCPP_WARN(rclcpp::get_logger("lidar_processor"), 
                       "[激光性能] 警告: 平均处理时间过长 %.1fms (>100ms)，可能导致丢帧！", avg_total);
        }
        if (max_kf_time > 80.0) {
            RCLCPP_WARN(rclcpp::get_logger("lidar_processor"), 
                       "[激光性能] 警告: IESKF峰值时间过长 %.1fms (>80ms)，收敛问题？", max_kf_time);
        }
        if (avg_downsample < 30.0) {
            RCLCPP_WARN(rclcpp::get_logger("lidar_processor"), 
                       "[激光性能] 警告: 点云保留率过低 %.0f%% (<30%%)，检查扫描质量！", avg_downsample);
        }
        
        // 重置统计变量
        total_time_sum = kf_time_sum = map_time_sum = 0.0;
        max_total_time = max_kf_time = 0.0;
        downsample_ratio_sum = 0;
    }

}

/*- 版本2  优化点面匹配P2PICP 约束,增加鲁棒核函数
 * 转换到世界系；KD搜索最近邻；拟合平面；判断是否是好平面（投影误差小 + 点到原点距离大）；保存法向量、投影误差 */
inline double huberWeight(double r, double delta = 0.1)
{
    double abs_r = std::abs(r);
    return (abs_r <= delta) ? 1.0 : delta / abs_r;
}

void LidarProcessor::updateLossFunc(State &state, SharedState &obs) {
    obs.reason = "poor_match";
    obs.valid = false;
    obs.H.setZero();
    obs.b.setZero();
    obs.count = 0;
    obs.res = 0;
    if (!m_package || m_cloud_down_lidar->empty())
        return;
    // Rebuild once per scan, or when time moves outside the local trust interval.
    // No full-state Jacobian is cached: pose/extrinsic columns are recomputed below.
    if (!m_linearization_valid || std::abs(state.td-m_linearization_anchor.td)>0.0025) {
        fast_time::Trajectory trajectory;
        if(!trajectory.build(state,m_package->imus,m_query_begin+state.td,
                             m_query_end+state.td,m_config.time_offset.max_imu_gap)) {
            obs.reason="trajectory_coverage";return;
        }
        ++m_kf->report.motion_relinearizations;
        m_linearization_valid=false;
        m_time_points.assign(m_groups.size(),fast_time::TimePointLinearization{});
        std::atomic<bool> points_covered{true};
#pragma omp parallel for num_threads(MP_PROC_NUM) schedule(static) if(m_groups.size()>512)
        for(int i=0;i<int(m_groups.size());++i) {
            for(size_t k:m_groups[i]) {
                const auto& p=(*m_package->raw_cloud)[k];
                fast_time::TimePointLinearization point;
                if(!fast_time::linearizeTimePoint(trajectory,state,m_package->imus,
                    {Eigen::Vector3d(p.x,p.y,p.z),m_package->cloud_start_time+p.curvature*.001},
                    m_config.time_offset.max_imu_gap,point)) {
                    points_covered.store(false,std::memory_order_relaxed);break;
                }
                m_time_points[i].point+=point.point;
                m_time_points[i].rate+=point.rate;
            }
            m_time_points[i].point/=double(m_groups[i].size());
            m_time_points[i].rate/=double(m_groups[i].size());
        }
        if(!points_covered.load(std::memory_order_relaxed)) {obs.reason="point_coverage";return;}
        m_linearization_anchor=state;m_linearization_valid=true;
    }
    const double delta_td=state.td-m_linearization_anchor.td;
    std::vector<Eigen::Matrix<double,1,22>> rows(m_groups.size());
    std::vector<double> residuals(m_groups.size());
    std::vector<unsigned char> selected(m_groups.size(),0);
#pragma omp parallel for num_threads(MP_PROC_NUM) schedule(static) if(m_groups.size()>512)
    for(int i=0;i<int(m_groups.size());++i) {
        Eigen::Matrix<double,3,22> jp;
        const Eigen::Vector3d world=m_time_points[i].world(state,delta_td,&jp);
        PointType pw;
        pw.x = world.x();
        pw.y = world.y();
        pw.z = world.z();
        auto &near = m_nearest_points[i];
        std::vector<float> distances(m_config.near_search_num);
        m_ikdtree->Nearest_Search(pw, m_config.near_search_num, near, distances);
        if (int(near.size()) < m_config.near_search_num ||
            distances.back() > m_config.near_search_radius)
            continue;
        Eigen::Vector4d plane;
        if (!esti_plane(near, m_config.plane_fitting_tolerance, plane))
            continue;
        const double residual = plane.head<3>().dot(world) + plane(3);
        Eigen::Matrix<double, 1, 22> h = plane.head<3>().transpose() * jp;
        if (!m_config.esti_il)
            h.segment<6>(6).setZero();
        rows[i]=h;residuals[i]=residual;selected[i]=1;
    }
    // Keep the reduction order deterministic; only independent point work is parallel.
    for(size_t i=0;i<m_groups.size();++i) {
        if(!selected[i])continue;
        const auto& h=rows[i];const double residual=residuals[i];
        const double weight = huberWeight(residual) * m_config.lidar_cov_inv;
        obs.H.noalias() += weight * h.transpose() * h;
        obs.b.noalias() += weight * h.transpose() * residual;
        obs.res += residual * residual;
        ++obs.count;
    }
    obs.fitness = double(obs.count) / m_cloud_down_lidar->size();
    obs.res = std::sqrt(obs.res / std::max(1, obs.count));
    obs.valid = obs.count >= m_config.effect_feat_num && obs.H.allFinite() && obs.b.allFinite();
}

//将点云 inp 通过旋转矩阵 r 和位移向量 t 转换成世界坐标系下的新点云
CloudType::Ptr LidarProcessor::transformCloud(CloudType::Ptr inp, const M3D &r, const V3D &t)
{
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform.block<3, 3>(0, 0) = r.cast<float>();
    transform.block<3, 1>(0, 3) = t.cast<float>();
    CloudType::Ptr ret(new CloudType);
    pcl::transformPointCloud(*inp, *ret, transform);


    return ret;
}