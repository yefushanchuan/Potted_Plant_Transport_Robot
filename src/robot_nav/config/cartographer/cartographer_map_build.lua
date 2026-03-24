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
  imu_sampling_ratio = 1.0,               -- IMU采样率，略微降低以适应实际50Hz频率
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

TRAJECTORY_BUILDER_2D = {
  use_imu_data = true,--是否使用imu
  min_range = 0.1,--雷达距离配置
  max_range = 20.,
  min_z = -0.8,--雷达高度配置，将高度数据转换成2D
  max_z = 2.,
  missing_data_ray_length = 5.,--超出max_range将以此长度进行free插入，充分利用max_range外的数据，也可以不使用。
  num_accumulated_range_data = 1,--将一帧雷达数据分成几个ros发出来，减少运动畸变影响。
  voxel_filter_size = 0.025,--体素滤波，使远处和近处的点云权重一致
  adaptive_voxel_filter = {
    max_length = 0.5,--最大边长0.5
    min_num_points = 200,--大于此数据，则减小体素滤波器的大小。
    max_range = 16.,--大于max_range的值被移除
  },
  loop_closure_adaptive_voxel_filter = {--闭环的体素滤波器，同上
    max_length = 0.5,
    min_num_points = 200,
    max_range = 16.,
  },
  use_online_correlative_scan_matching = true,--csm算法解决在线扫描匹配问题，为ceres优化提供先验，如果无IMU或odom的情况下，如无此项前端效果较差。但是一旦使用该项，IMU和Odom的效果将会变得很弱。
  real_time_correlative_scan_matcher = {--开启online后使用，分配搜索窗口的参数
    linear_search_window = 0.1, --线窗口
    angular_search_window = math.rad(20.),--角度窗口
    translation_delta_cost_weight = 1e-1,--这两个为平移和旋转的比例，如你知道你的机器人旋转不多，则可以较少它的权重，一般情况下1：1.
    rotation_delta_cost_weight = 1e-1,
  },
--通过online或者imu/odom的先验输入ceres，然后进行优化，以下为优化的参数配置
  ceres_scan_matcher = {
    occupied_space_weight = 10.,--数据源的权重
    translation_weight = 10.,
    rotation_weight = 40.,
    ceres_solver_options = {--谷歌开发的最小二乘库ceres Solver配置
      use_nonmonotonic_steps = false,--是否使用非单调的方法
      max_num_iterations = 20,--迭代次数
      num_threads = 1,--使用线程数
    },
  },
--运动过滤器，避免静止的时候插入scans
  motion_filter = {
    max_time_seconds = 5.,--过滤的时间、距离、角度
    max_distance_meters = 0.2,
    max_angle_radians = math.rad(1.),
  },
  imu_gravity_time_constant = 10.,--一定时间内观察imu的重力，以确定是否使用imu数据
  submaps = {
    num_range_data = 160,
    grid_options_2d = {
      resolution = 0.03,
    },
  },
}


-- ========== 后端位姿图优化配置 ==========
POSE_GRAPH = {
  optimize_every_n_nodes = 0,--多少节点进行一次优化，0为关闭后端优化。
  --constraint_builder为约束项参数
  --非全局约束（intra submaps constraints）：一条轨迹上的不同节点约束。
  --全局约束（inter submaps constraints或者loop closure constrains）：新子图与先前的足够近（search window）的节点之间的约束。
  constraint_builder = {
    sampling_ratio = 0.3,--约束采样，太多则速度慢，太少则会导致约束丢失，闭环效果不好
    max_constraint_distance = 15.,--最大约束距离
    min_score = 0.55,--Fast csm的最低分数，高于此分数才进行优化。
    global_localization_min_score = 0.6,--全局定位最小分数，低于此分数则全局定位不可靠。
    loop_closure_translation_weight = 1.1e4,--闭环平移约束权重
    loop_closure_rotation_weight = 1e5,--闭环旋转约束权重
    log_matches = true,--除了rviz中的约束显示，还可以打开直方图约束，直方图的约束构建器是否开启
    fast_correlative_scan_matcher = {--fast csm的搜索窗口参数
      linear_search_window = 7.,
      angular_search_window = math.rad(30.),
      branch_and_bound_depth = 7,--分支定界的最深节点，深度优先搜索
    },
    ceres_scan_matcher = {--ceres优化器的参数，可以参考前端
      occupied_space_weight = 10.,
      translation_weight = 10.,
      rotation_weight = 1.,
      ceres_solver_options = {
        use_nonmonotonic_steps = true,
        max_num_iterations = 10,
        num_threads = 1,
      },
    },
  },
  matcher_translation_weight = 5e2,--匹配的平移权重（这个是非全局的约束，区别于上面的全局约束）
  matcher_rotation_weight = 1.6e3,--匹配的旋转权重
  --残差方程的参数配置
  optimization_problem = {
    huber_scale = 1e1,--huber损失函数，数值越大，离群值（异常值）影响越大
    acceleration_weight = 1.1e2,--imu的加速度权重
    rotation_weight = 1.6e4,--imu的旋转权重
    local_slam_pose_translation_weight = 1e5,--局部匹配的平移权重（局部匹配的粗略估计）
    local_slam_pose_rotation_weight = 1e5,--局部匹配的旋转权重
    odometry_translation_weight = 1e5,--odom的平移权重
    odometry_rotation_weight = 1e5,--odom的旋转权重
    fixed_frame_pose_translation_weight = 1e1,--固定帧的平移权重（类似于gps），以下都是固定帧，目前我们还没有用全局定位的传感器
    fixed_frame_pose_rotation_weight = 1e2,
    fixed_frame_pose_use_tolerant_loss = false,
    fixed_frame_pose_tolerant_loss_param_a = 1,
    fixed_frame_pose_tolerant_loss_param_b = 1,
    log_solver_summary = false,--是否记录ceres的全局优化结果，用于改进外部校准。
    use_online_imu_extrinsics_in_3d = true,--是否使用全局优化结果对imu进行优化。
    fix_z_in_3d = false,--3d雷达的z轴是否可以变化？？？
    ceres_solver_options = {--可以参考前端
      use_nonmonotonic_steps = false,
      max_num_iterations = 50,
      num_threads = 7,
    },
  },
  max_num_final_iterations = 200,--结束建图之后的优化迭代次数
  global_sampling_ratio = 0.003,--全局定位时使用，生成单个定位速率
  log_residual_histograms = true,--记录残差直方图
  global_constraint_search_after_n_seconds = 10.,
  -- overlapping_submaps_trimmer_2d = {
  -- fresh_submaps_count = 1,
  -- min_covered_area = 2,
  -- min_added_submaps_count = 5,
  -- },
}

return options  -- 返回完整配置选项





