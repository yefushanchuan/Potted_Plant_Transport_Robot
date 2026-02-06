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
            package="hipnuc_imu",
            executable="talker",
            name="IMU_publisher",
            parameters=[imu_config],
            output="screen",
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
