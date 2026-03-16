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
    pkg_robot_base = get_package_share_directory("robot_base")
    ekf_config_path = os.path.join(pkg_robot_base, "config", "ekf.yaml")

    declared_arguments = [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use sim time if true",
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
            PathJoinSubstitution(
                [FindPackageShare("robot_base"), "urdf", "agrobot.xacro"]
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
                ("~/robot_description", "/robot_description"),
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
        
        ####################################    传感器    ######################################
        # sensor: IMU
        Node(
            package="hipnuc_imu",
            executable="talker",
            name="IMU_publisher",
            parameters=[imu_config],
            output="screen",
        ),

        # sensor: LiDAR 3D
        Node(
            package='livox_ros_driver2',
            executable='livox_ros_driver2_node',
            name='livox_lidar_publisher',
            output='screen',
            parameters=[
                {'xfer_format': 0},    # 0-Pointcloud2(PointXYZRTL), 1-customized pointcloud format
                {'multi_topic': 0},    # 0-All LiDARs share the same topic
                {'data_src': 0},       # 0-lidar
                {'publish_freq': 10.0},
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

        # tools
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
