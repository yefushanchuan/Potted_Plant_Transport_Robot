-- ========== Cartographer SLAM建图配置文件 ==========
-- 专为差分驱动机器人的2D SLAM建图优化
-- 适用场景: 室内环境地图构建、实时SLAM
-- 传感器配置: 激光雷达 + IMU + 里程计(EKF融合)
-- 硬件平台: 差分驱动机器人，矩形footprint

-- 引入Cartographer核心配置文件
include "map_builder.lua"        -- 地图构建器基础配置
include "trajectory_builder.lua" -- 轨迹构建器基础配置

-- ========== 主要配置选项 ==========
options = {
  -- === 核心组件配置 ===
  map_builder = MAP_BUILDER,                -- 地图构建器实例，负责后端优化
  trajectory_builder = TRAJECTORY_BUILDER,  -- 轨迹构建器实例，负责前端处理

  -- === 坐标系配置 ===
  map_frame = "map",                        -- 地图坐标系名称，全局固定参考系
  tracking_frame = "imu_link",              -- 跟踪坐标系，使用IMU坐标系提高姿态精度
  published_frame = "odom",                 -- 发布的位姿坐标系，形成map->odom变换
  odom_frame = "odom",                      -- 里程计坐标系名称

  -- === TF变换配置 ===
  provide_odom_frame = false,               -- 不提供odom->base_footprint变换
                                           -- 让EKF统一管理此变换，避免TF冲突
                                           -- TF链: map->odom(Carto)->base_footprint(EKF)
  publish_frame_projected_to_2d = true,    -- 将3D位姿投影到2D平面，适合地面机器人
  use_pose_extrapolator = true,            -- 使用位姿外推器进行TF发布，提高实时性
  publish_tracked_pose = true,             -- 发布跟踪的位姿信息到话题
  publish_to_tf = true,                    -- 发布TF变换到tf树

  -- === 传感器数据源配置 ===
  use_odometry = true,                     -- 使用里程计数据，来自EKF融合后的高质量数据
  use_nav_sat = false,                     -- 不使用GPS数据(室内环境)
  use_landmarks = false,                    -- {{ AURA-X: Modify - 启用地标数据用于反光板定位约束. }}
  -- 注意: 地标数据作为辅助约束，与里程计数据协同工作

  -- === 激光雷达配置 ===
  num_laser_scans = 1,                     -- 使用1个单线激光雷达
  num_multi_echo_laser_scans = 0,          -- 不使用多回波激光雷达
  num_subdivisions_per_laser_scan = 1,     -- 每帧激光数据的处理分割数，通常为1
  num_point_clouds = 0,                    -- 不使用3D点云数据
  -- 注意: 激光雷达相关参数不能同时为0

  -- === 传感器采样率配置 ===
  -- 控制各传感器数据的使用频率，1.0表示使用所有数据
  rangefinder_sampling_ratio = 1.0,        -- 激光雷达采样率，1.0使用所有25Hz数据
  odometry_sampling_ratio = 1.0,           -- 里程计采样率，1.0使用所有50Hz EKF数据
  fixed_frame_pose_sampling_ratio = 1.0,   -- 固定坐标系位姿采样率
  imu_sampling_ratio = 1.0,               -- IMU采样率，略微降低以适应实际95Hz频率
  landmarks_sampling_ratio = 1.0,          -- 地标采样率，使用所有反光板检测数据.

  -- === 时间和发布参数配置 ===
  lookup_transform_timeout_sec = 1.0,      -- TF查找超时时间(秒)，增加容错性
  submap_publish_period_sec = 0.3,         -- 子地图发布周期(秒)，约3.3Hz
  pose_publish_period_sec = 0.025,         -- 位姿发布周期(秒)，40Hz高频率发布
  trajectory_publish_period_sec = 0.1,     -- 轨迹发布周期(秒)，10Hz发布频率
}
-- ========== 地图构建器配置 ==========
-- 重写导入的默认配置，适配差分驱动机器人
MAP_BUILDER.use_trajectory_builder_2d = true  -- 启用2D轨迹构建器，适合地面机器人
MAP_BUILDER.num_background_threads = 2  

-- === 纯定位模式配置 (当前注释掉，使用SLAM建图模式) ===
-- 纯定位模式用于在已知地图中进行定位，不构建新地图
-- TRAJECTORY_BUILDER.pure_localization_trimmer = {
--   max_submaps_to_keep = 3,              -- 保留的最大子地图数量
-- }

-- ========== 2D轨迹构建器配置 ==========
-- === 前端扫描匹配配置 ===
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
-- 启用在线相关扫描匹配，计算量大但鲁棒性强
-- 在里程计或IMU不准确时仍能保持良好的定位效果，适合建图阶段

TRAJECTORY_BUILDER_2D.use_imu_data = true    -- 启用IMU数据融合，提高姿态估计精度
-- === IMU参数配置 ===
TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 5.0   -- IMU重力时间常数(秒)
-- 较小值使IMU更快适应重力方向变化，提高建图时的稳定性

-- === 激光雷达范围配置 ===
TRAJECTORY_BUILDER_2D.min_range = 0.1                   -- 激光最小有效距离(m)
TRAJECTORY_BUILDER_2D.max_range = 20.0                  -- 激光最大有效距离(m)，匹配雷达规格
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.0     -- 缺失数据光线长度(m)，适中值

-- === 高度过滤配置 ===
TRAJECTORY_BUILDER_2D.min_z = -0.8                      -- 最小Z高度(m)，过滤地面以下点
TRAJECTORY_BUILDER_2D.max_z = 2.0                       -- 最大Z高度(m)，过滤天花板以上点

-- === 点云处理参数配置 ===
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1    -- 累积的激光帧数，1表示实时处理
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.025         -- 体素滤波尺寸(m)，保留更多细节用于建图

-- === 实时相关扫描匹配器配置 ===
-- 用于前端实时位姿估计的快速扫描匹配
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.08
-- 线性搜索窗口(m)，减小搜索范围提高建图时的稳定性和速度

-- 以下参数已注释，使用默认值以保持建图稳定性
-- TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(30.)
-- 角度搜索窗口(rad)，控制旋转搜索范围
-- TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.0
-- 平移变化代价权重
-- TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 10.0
-- 旋转变化代价权重

-- === Ceres非线性优化扫描匹配器配置 ===
-- 用于精确的位姿优化，基于Ceres Solver非线性优化库
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 10.0  -- 占用空间匹配权重
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10.0     -- 平移约束权重
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 60.0        -- 旋转约束权重

-- === 自适应体素滤波器配置 ===
-- 根据点云密度自动调整滤波强度，适配激光雷达特性
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 0.5        -- 最大体素边长(m)
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 150     -- 最小点数要求，适应室内建图环境
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_range = 16.0         -- 最大滤波距离(m)，与激光最大距离一致

-- === 运动滤波器配置 ===
-- 控制何时向后端添加新的扫描数据，减少TF跳变的关键参数
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.8           -- 最大时间间隔(秒)，保留更多时间节点
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.10      -- 最大距离间隔(m)，提高位置精度
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(10.) -- 最大角度间隔(rad)，提高角度精度

-- === 子地图配置 ===
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 160              -- 每个子地图包含的激光帧数，提高建图质量
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.03 -- 子地图分辨率(m/pixel)，高精度建图

-- ========== 后端位姿图优化配置 ==========
-- === 优化频率配置 ===
POSE_GRAPH.optimize_every_n_nodes = 100                          -- 每30个节点执行一次优化，平衡性能和精度
-- === 约束构建器配置 ===
POSE_GRAPH.constraint_builder.sampling_ratio = 0.05             -- 局部约束采样率，增加以提高约束生成
POSE_GRAPH.global_sampling_ratio = 0.05                         -- 全局约束采样率，增加回环检测频率
POSE_GRAPH.constraint_builder.max_constraint_distance = 8.0    -- 最大约束距离(m)，减小以提高匹配质量

-- === 回环检测阈值配置 ===
POSE_GRAPH.constraint_builder.min_score = 0.65                  -- 局部约束最小匹配分数，进一步降低阈值
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.55  -- 全局定位最小分数，降低阈值提高成功率

-- === 回环闭合权重配置 ===
POSE_GRAPH.constraint_builder.loop_closure_translation_weight = 1.1e4  -- 回环闭合平移权重
POSE_GRAPH.constraint_builder.loop_closure_rotation_weight = 1.5e5     -- 回环闭合旋转权重
POSE_GRAPH.constraint_builder.log_matches = true                       -- 启用匹配日志输出，便于建图调试

-- === 全局搜索配置 ===
POSE_GRAPH.global_constraint_search_after_n_seconds = 3.               -- 全局约束搜索延迟(秒)，建图时增加搜索频率

-- === 全局匹配器权重配置 ===
POSE_GRAPH.matcher_translation_weight = 1e3                            -- 全局SLAM匹配器平移权重
POSE_GRAPH.matcher_rotation_weight = 5e2                             -- 全局SLAM匹配器旋转权重

-- === 优化问题配置 ===
POSE_GRAPH.optimization_problem.huber_scale = 1e2                      -- Huber损失函数尺度，提高异常值鲁棒性
POSE_GRAPH.optimization_problem.acceleration_weight = 5e2              -- 加速度约束权重
POSE_GRAPH.optimization_problem.rotation_weight = 2e4                  -- 旋转约束权重，减少角度抖动
POSE_GRAPH.optimization_problem.log_solver_summary = true              -- 输出Ceres求解器摘要，便于调试

-- === 里程计约束权重配置 ===
POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e2      -- 里程计平移约束权重
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 2e4         -- 里程计旋转约束权重

-- === 快速相关扫描匹配器配置 ===
-- 用于后端约束生成的分支定界算法参数
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 10.0    -- 线性搜索窗口(m)
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(45.)  -- 角度搜索窗口(rad)
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.branch_and_bound_depth = 9     -- 分支定界搜索深度

-- === 最终优化配置 ===                             -- 约束构建线程数，提升建图速度
POSE_GRAPH.max_num_final_iterations = 50                               -- 最终优化最大迭代次数，提高建图精度

-- === 时间戳同步优化配置 ===
TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 10.0                 -- IMU重力时间常数(秒)，提高建图稳定性
MAP_BUILDER.collate_by_trajectory = false                              -- 禁用轨迹排序，减少时间戳敏感性

-- ========== 地标定位配置 ==========
-- {{ AURA-X: Add - 地标定位相关配置，仅在启用地标时生效. }}
TRAJECTORY_BUILDER.collate_landmarks = false                           -- 非阻塞地标处理，避免地标数据延迟影响建图
-- 注意：地标权重参数使用Cartographer默认值，无需显式设置

return options  -- 返回完整配置选项





