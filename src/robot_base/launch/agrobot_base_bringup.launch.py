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
    
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers, base_params],
        output="both",
        remappings=[("~/robot_description", "/robot_description")],
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, base_params],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        name="joint_state_broadcaster_spawner",
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "--controller-manager", "/controller_manager"],
        name="robot_controller_spawner",
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path, base_params],
        remappings=[('odometry/filtered', 'odom')]
    )

    imu_node = Node(
        package="hipnuc_imu",
        executable="talker",
        name="IMU_publisher",
        parameters=[imu_config],
        output="screen",
    )

    livox_lidar_node = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=[
            {'xfer_format': 0},
            {'multi_topic': 0},
            {'data_src': 0},
            {'publish_freq': 10.0},
            {'output_data_type': 0},
            {'frame_id': 'front_laser_link'},
            {'user_config_path': PathJoinSubstitution([
                FindPackageShare("livox_ros_driver2"), "config", "MID360_config.json"
            ])},
            {'cmdline_input_bd_code': 'livox0000000001'}
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
