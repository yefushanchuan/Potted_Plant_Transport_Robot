import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, PushRosNamespace, SetRemap


def generate_launch_description():
    """启动 robot_base 相关底盘/IMU/EKF 节点."""
    pkg_robot_base = get_package_share_directory("robot_bringup")
    ekf_config_path = os.path.join(pkg_robot_base, "config", "base", "ekf.yaml")

    declared_arguments = [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use sim time if true",
        ),
        DeclareLaunchArgument(
            "imu_port",
            default_value="/dev/IMU",
            description="The serial port for the IMU",
        ),
        DeclareLaunchArgument(
            "imu_topic",
            default_value="imu",
            description="The topic name for the IMU",
        ),
        DeclareLaunchArgument(
            "namespace",
            default_value="",
            description="ROS2 namespace (optional, default: empty for root namespace)",
        ),
    ]

    use_sim_time = LaunchConfiguration("use_sim_time")
    namespace = LaunchConfiguration("namespace")

    imu_config = os.path.join(
        get_package_share_directory("hipnuc_imu"),
        "config",
        "hipnuc_config.yaml",
    )

    # 创建基础参数字典
    base_params = {
        "use_sim_time": use_sim_time,
    }

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            os.path.join(
                pkg_robot_base, "urdf", "agrobot.xacro"
            ),
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str),
        "use_sim_time": use_sim_time,
    }

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("robot_base"),
            "config",
            "agrobot_diff_controllers.yaml",
        ]
    )

    nodes_list = [
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[robot_controllers, base_params],
            output="screen",
            remappings=[
                ("~/robot_description", "/robot_description")
            ],
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="both",
            parameters=[
                robot_description,
                base_params,
            ],
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "joint_state_broadcaster",
                "--controller-manager",
                "controller_manager",
            ],
            name="joint_state_broadcaster_spawner",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "diff_drive_controller",
                "--controller-manager",
                "controller_manager",
            ],
            name="robot_controller_spawner",
        ),
        Node(
            package="hipnuc_imu",
            executable="talker",
            name="IMU_publisher",
            parameters=[imu_config],
            output="screen",
        ),

        # EKF 已禁用 - 直接使用差速控制器的 odom
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[
                ekf_config_path, 
                base_params,
                ],
            remappings=[
                ('odometry/filtered', 'odom')
            ]
        ),

        # Livox MID360 Driver
        Node(
            package='livox_ros_driver2',
            executable='livox_ros_driver2_node',
            name='livox_lidar_publisher',
            output='screen',
            parameters=[
                {'xfer_format': 4},    # 0-Pointcloud2(PointXYZRTL), 1-customized pointcloud format
                {'multi_topic': 0},    # 0-All LiDARs share the same topic
                {'data_src': 0},       # 0-lidar
                {'publish_freq': 20.0},
                {'output_data_type': 0},
                {'frame_id': 'front_laser_link'},
                {'user_config_path': PathJoinSubstitution([
                    FindPackageShare("livox_ros_driver2"),
                    "config",
                    "MID360_config.json"
                ])},
                {'cmdline_input_bd_code': 'livox0000000001'}
            ]
        ),

        Node(
            package="realsense2_camera",
            executable="realsense2_camera_node",
            name="camera",
            output="screen",
            parameters=[
                {"camera_name": "camera"},  # 话题前缀
                {"camera_namespace": ""},  # 不使用命名空间前缀
                {"base_frame_id": "front_camera_link"},  # 连接到 URDF
                # 使用设备同步模式，让输出时间戳对齐 ROS 时间基（避免 costmap MessageFilter 丢帧）
                {"enable_sync": True},
                {"depth_module.global_time_enabled": True},
                {"use_sim_time": use_sim_time},
                {"depth_module.depth_profile": "424x240x30"},  # 降低分辨率提升点云频率（默认848x480太吃CPU）
                {"enable_color": False},
                {"enable_depth": True},
                {"enable_infra": False},
                {"enable_infra1": False},
                {"enable_infra2": False},
                {"pointcloud.enable": True},
                {"pointcloud.stream_filter": 0},  # 使用哪个数据流作为纹理来源。 0-任意流(禁用纹理), 1-深度流(因数据格式限制、仅近红外流可用), 2-彩色流
                {"pointcloud.allow_no_texture_points": True},
                {"publish_tf": True},
            ]
        ),

        # D435 点云 FOV 过滤器: 87° → 16°（只保留正前方 ±8°）
        Node(
            package="pointcloud_proc",
            executable="pointcloud_fov_filter_exe",
            name="d435_fov_filter",
            output="screen",
            parameters=[
                {"input_topic": "/camera/depth/color/points"},
                {"output_topic": "/camera/depth/color/points_fov"},
                {"fov_angle_deg": 18.0},
                {"coordinate_frame": "camera"},
            ],
        ),
        
        # 静态 TF: front_camera_link -> camera_front_camera_link (RealSense Base)
        # RealSense 节点生成的 base frame 是 {camera_name}_{base_frame_id} = camera_front_camera_link
        # 我们需要将其连接到 URDF 的 front_camera_link
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="camera_base_bridge",
            arguments=[
                "--x", "0.0",
                "--y", "0.0", 
                "--z", "0.0",
                "--roll", "0.0",
                "--pitch", "0.0",
                "--yaw", "0.0",
                "--frame-id", "front_camera_link",
                "--child-frame-id", "camera_front_camera_link"
            ],
            output="screen"
        ),

        Node(
            package="topic_tools",
            executable="relay",
            name="cmd_vel_relay",
            output="screen",
            namespace=namespace,
            arguments=["cmd_vel", "diff_drive_controller/cmd_vel_unstamped"],
            parameters=[base_params],
        ),

        # # Tilt Compensator Node (C++) - 只处理 CustomMsg
        # Node(
        #     package='robot_base',
        #     executable='tilt_compensator',
        #     name='tilt_compensator',
        #     output='screen',
        #     parameters=[
        #         {'pitch_deg': 45.0}  # 正值：将向下倾斜的点云"抬起"
        #     ]
        # ),

        # robot_state_shower - 可视化机器人状态（用于调试，正式使用时可注释掉）
        Node(
            package="robot_state_shower",
            executable="state_web_server",
            name="state_web_server",
            output="screen",
            parameters=[
                {"topic": "/agrobot_base_info"},
                {"host": "0.0.0.0"},
                {"port": 8000},
            ],
        )

    ]

    # 根据命名空间是否为空来决定是否使用PushRosNamespace和TF重映射
    if namespace:
        # 有命名空间时使用完整的命名空间配置
        bringup_cmd_group = GroupAction([
            PushRosNamespace(namespace=namespace),
            SetRemap("tf", "/tf"),  # TF重映射，将命名空间内的tf重映射到全局
            SetRemap("tf_static", "/tf_static"),  # TF重映射，将命名空间内的tf_static重映射到全局
        ] + nodes_list)
    else:
        # 无命名空间时直接使用节点列表
        bringup_cmd_group = GroupAction(nodes_list)

    nodes = [
        bringup_cmd_group,  # {{ AURA-X: Replace - 使用GroupAction替代单独的节点. }}
    ]

    return LaunchDescription(declared_arguments + nodes) 
