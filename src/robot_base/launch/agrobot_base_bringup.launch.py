import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, RegisterEventHandler
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, PushRosNamespace, SetRemap

def generate_launch_description():
    """启动 robot_base 相关底盘/IMU/EKF 节点."""
    pkg_robot_base = get_package_share_directory("robot_base")
    ekf_config_path = os.path.join(pkg_robot_base, "config", "ekf.yaml")

    declared_arguments =[
        DeclareLaunchArgument("use_sim_time", default_value="false", description="Use sim time if true"),
        DeclareLaunchArgument("imu_topic", default_value="imu", description="The topic name for the IMU"),
        DeclareLaunchArgument("namespace", default_value="", description="ROS2 namespace (optional)"),
    ]

    use_sim_time = LaunchConfiguration("use_sim_time")
    namespace = LaunchConfiguration("namespace")

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

    # EKF 融合定位
    # 作用: 融合 IMU 和轮式里程计数据，输出滤波后的位姿估计，
    #       发布到 /odom 话题，并广播 odom → base_footprint 的 TF 变换
    # 输入: /imu (IMU数据), /diff_drive_controller/odom (轮式里程计)
    # 输出: /odom (融合后的里程计), TF: odom → base_footprint (动态更新)
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path, base_params],
        remappings=[('odometry/filtered', 'odom')]
    )

    # IMU 数据发布
    # 作用: 驱动 hipnuc IMU 硬件，读取原始传感器数据，
    #       发布 IMU 消息到 /imu 话题，供 EKF 和其他节点使用
    # 输出: /imu 
    imu_node = Node(
        package="hipnuc_imu",
        executable="talker",
        name="IMU_publisher",
        parameters=[imu_config],
        output="screen",
    )

    # Livox 激光雷达驱动
    # 作用: 驱动 Livox MID360 激光雷达，发布点云数据
    # 输出: /livox/lidar
    # 坐标系: 点云数据的 frame_id = 'front_laser_link'（需在 URDF 中定义该 link）
    livox_lidar_node = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=[
            {'xfer_format': 0}, # 0=标准点云格式 /livox/lidar, 1=Livox自定义格式 /livox/lidar
            {'multi_topic': 0}, # 0=单话题模式, 1=多话题模式（多雷达时使用）
            {'data_src': 0}, # 0=在线雷达数据, 1=从ROS bag读取, 2=从LVX文件读取
            {'publish_freq': 10.0}, # 点云发布频率 (Hz)
            {'output_data_type': 0}, # 0=PointCloud2, 1=自定义 Livox 点云格式
            {'frame_id': 'front_laser_link'}, # 点云数据的坐标系ID，必须与 URDF 中的 link 名称一致
            {'user_config_path': PathJoinSubstitution([
                FindPackageShare("livox_ros_driver2"), "config", "MID360_config.json"
            ])}, # 雷达硬件配置文件路径（包含IP、端口等参数）
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
        ekf_node,
        imu_node,
        livox_lidar_node
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
