import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, RegisterEventHandler
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, PushRosNamespace, SetRemap, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.conditions import IfCondition

def generate_launch_description():
    """启动 robot_base 相关底盘/IMU/EKF 节点."""
    pkg_robot_base = get_package_share_directory("robot_base")
    ekf_config_path = os.path.join(pkg_robot_base, "config", "ekf.yaml")

    declared_arguments =[
        DeclareLaunchArgument("use_sim_time", default_value="false", description="Use sim time if true"),
        DeclareLaunchArgument("namespace", default_value="", description="ROS2 namespace (optional)"),
        DeclareLaunchArgument("use_ekf",default_value="true",description="Whether to launch EKF node"),
    ]

    use_sim_time = LaunchConfiguration("use_sim_time")
    namespace = LaunchConfiguration("namespace")
    use_ekf = LaunchConfiguration("use_ekf")

    imu_config = os.path.join(
        get_package_share_directory("hipnuc_imu"), "config", "hipnuc_config.yaml"
    )

    base_params = {"use_sim_time": use_sim_time}

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
            PathJoinSubstitution([FindPackageShare("robot_base"), "urdf", "agrobot.xacro"]),
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str),
        "use_sim_time": use_sim_time,
    }

    robot_controllers = PathJoinSubstitution([FindPackageShare("robot_base"), "config", "agrobot_diff_controllers.yaml"]
    )

    # ================= 核心节点定义 =================

    # 发布 TF 变换
    # 作用: 从参数读取 URDF，解析连杆和关节结构；
    #       订阅 /joint_states 话题获取关节角度，
    #       发布 TF 变换实现可视化（静态+动态）
    #       将 URDF 字符串以锁存话题的形式对外广播，供控制器管理器和 RViz 读取。
    # 输入: 参数 robot_description (URDF字符串), 话题 /joint_states
    # 输出: /tf, /tf_static, /robot_description
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, base_params],
    )

    # 启动控制器管理器
    # 作用: ros2_control 的核心节点，加载硬件接口和控制器框架，
    #       从参数 robot_controllers 加载控制器配置，
    #       通过 remapping 从 /robot_description 获取 URDF 中的 <ros2_control> 硬件配置
    # 注意: 必须先于所有 spawner 启动，提供 /controller_manager 服务
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers, base_params],
        output="both",
        remappings=[("~/robot_description", "/robot_description")],
    )

    # 加载关节状态广播器
    # 作用: 向 controller_manager 请求加载 joint_state_broadcaster 控制器，spawner仅生命周期管理，不执行控制逻辑，
    #       读取硬件接口中的关节状态，发布到 /joint_states 话题
    # 依赖: 必须在 ros2_control_node 启动后执行（由 delay_joint_state_spawner 控制时序）
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        name="joint_state_broadcaster_spawner",
    )

    # 加载差分驱动控制器
    # 作用: 向 controller_manager 请求加载 diff_drive_controller，
    #       订阅 /diff_drive_controller/cmd_vel_unstamped 话题控制轮子速度，发布轮式里程计 /diff_drive_controller/odom 话题
    # 依赖: 必须在 joint_state_broadcaster 加载完成后执行（由 delay_robot_controller_spawner 控制时序）
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "--controller-manager", "/controller_manager"],
        name="robot_controller_spawner",
    )

    # IMU 数据发布 (硬件驱动)
    # 作用: 驱动 hipnuc IMU 硬件，读取原始传感器数据。
    # 输出: 原本直接发 /imu，现在重映射为 /imu_raw，把它交给下面的滤波节点。
    imu_node = Node(
        package="hipnuc_imu",
        executable="talker",
        name="IMU_publisher",
        parameters=[imu_config],
        output="screen",
        remappings=[('/imu', '/imu_raw')]
    )

    # IMU 互补滤波节点
    # 作用: 接收原始 IMU 数据，过滤高频噪声，融合加速度计和陀螺仪估算平滑姿态。
    # 输入: /imu_raw (订阅上一级的原始数据)
    # 输出: /imu (发布干净的数据给下一级的 EKF)
    imu_filter_node = Node(
        package='imu_complementary_filter',
        executable='complementary_filter_node',
        name='imu_filter_node',
        output='screen',
        parameters=[{
            'use_mag': False,           # 室内不用磁力计
            'publish_tf': False,        # 坚决不发 TF，全权交给 EKF，避免冲突
            'do_bias_estimation': True, # 消除陀螺仪静态漂移
            'do_adaptive_gain': True
        }],
        remappings=[
            ('imu/data_raw', '/imu_raw'),  
            ('imu/data', '/imu')           
        ]
    )

    # EKF 融合定位
    # 作用: 融合 IMU 和轮式里程计数据，输出滤波后的位姿估计，
    #       发布到 /odom 话题，并广播 odom → base_footprint 的 TF 变换
    # 输入: /imu (经过上面过滤后的干净IMU数据), /diff_drive_controller/odom
    # 输出: /odom (融合后的里程计), TF: odom → base_footprint (动态更新)
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path, base_params],
        remappings=[('odometry/filtered', 'odom')],
        condition=IfCondition(use_ekf) # 保持按需启动的逻辑
    )

    # ========================================================================
    # 容器 感知流水线组件容器 (Perception Pipeline Container)
    # 包含: Livox驱动 -> CropBox -> PC2Scan -> 2D Lidar Filter
    # 特点: 高带宽点云数据处理，零拷贝极大节省 CPU 内存拷贝开销
    # ========================================================================
    lidar_and_filter_container = ComposableNodeContainer(
        name='lidar_and_filter_container',   
        namespace='',                        
        package='rclcpp_components',         
        executable='component_container_mt',   
        output='screen',
        composable_node_descriptions=[
            # 1. Livox 激光雷达驱动
            ComposableNode(
                package='livox_ros_driver2',
                plugin='livox_ros::DriverNode',
                name='livox_lidar_publisher',
                parameters=[
                    {'xfer_format': 0},      
                    {'multi_topic': 0},     
                    {'data_src': 0},        
                    {'publish_freq': 10.0}, 
                    {'output_data_type': 0}, 
                    {'frame_id': 'front_laser_link'},
                    {'user_config_path': PathJoinSubstitution([
                        FindPackageShare("livox_ros_driver2"),
                        "config",
                        "MID360_config.json"
                    ])}
                ],
                remappings=[('/livox/lidar', '/livox/lidar_raw')],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),

            # 2. CropBox 点云过滤
            ComposableNode(
                package='robot_base',
                plugin='robot_base_utils::CropBoxComponent',
                name='livox_crop_box',
                parameters=[{
                    'target_frame': 'base_link',
                    'min_x': -0.5, 'max_x': 0.25,
                    'min_y': -0.275, 'max_y': 0.275,
                    'min_z': -0.1, 'max_z': 0.6,
                    'negative': True
                }],
                remappings=[
                    ('input', '/livox/lidar_raw'),
                    ('output', '/livox/lidar')
                ],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),

            # 3. PointCloud to LaserScan 
            ComposableNode(
                package='pointcloud_to_laserscan', 
                plugin='pointcloud_to_laserscan::PointCloudToLaserScanNode',
                name='pointcloud_to_laserscan_node',
                remappings=[
                    ('cloud_in', '/livox/lidar'),
                    ('scan', '/scan_raw')  
                ],
                parameters=[{
                    'target_frame': 'base_footprint',
                    'transform_tolerance': 0.05,       
                    'min_height': 0.15,                
                    'max_height': 1.0,                 
                    'angle_min': -3.14159,             
                    'angle_max': 3.14159,              
                    'angle_increment': 0.0087,         
                    'scan_time': 0.1,                  
                    'range_min': 0.2,                  
                    'range_max': 15.0,                 
                    'use_inf': True,
                    'inf_epsilon': 1.0
                }],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),

            # 4. 2D Lidar Filter 
            ComposableNode(
                package='robot_base',
                plugin='robot_base_utils::LidarFilter2D', 
                name='lidar_filter2d_node',
                parameters=[{
                    'source_topic': '/scan_raw', 
                    'pub_topic': '/scan',        
                    'outlier_threshold': 0.1
                }],
                extra_arguments=[{'use_intra_process_comms': True}]
            )
        ]
    )

    # ================= 时序依赖控制 (Event Handlers) =================
    
    # 1. 只有当 ros2_control_node 启动后，才去启动 joint_state_broadcaster
    delay_joint_state_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=ros2_control_node,
            on_start=[joint_state_broadcaster_spawner],
        )
    )

    # 2. 只有当 joint_state_broadcaster 加载完毕（即退出后），才去加载 diff_drive_controller
    delay_robot_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    # 组装节点列表
    nodes_list =[
        robot_state_publisher_node,
        ros2_control_node,
        delay_joint_state_spawner,       # 时序控制1
        delay_robot_controller_spawner,  # 时序控制2
        imu_node,
        imu_filter_node,
        ekf_node,
        lidar_and_filter_container   
    ]

    # 根据命名空间是否为空来决定是否使用 PushRosNamespace 和 TF 重映射
    if namespace:
        bringup_cmd_group = GroupAction([
            PushRosNamespace(namespace=namespace),
            SetRemap("tf", "/tf"),
            SetRemap("tf_static", "/tf_static"),
        ] + nodes_list)
    else:
        bringup_cmd_group = GroupAction(nodes_list)

    return LaunchDescription(declared_arguments + [bringup_cmd_group])
