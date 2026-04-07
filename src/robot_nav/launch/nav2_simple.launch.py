"""
Nav2 Launch File导航启动文件

使用方法：
1. 先启动机器人基础节点 (感知容器与状态估计容器)：
   ros2 launch robot_base agrobot_base_bringup.launch.py

2. 再启动定位节点 (FAST-LIO / ICP 等)：
   ros2 launch indoor_location run_argbot_mid360.launch.py

3. 最后启动此文件进行导航：
   ros2 launch robot_navigation nav2_only.launch.py map:=/path/to/map.yaml

注意：此 launch 文件假设 TF 树已经由上述两个 launch 文件正确建立：
  map -> odom -> base_footprint -> base_link -> sensors
"""

import os
import glob

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# ========== 地图目录配置 ==========
HOME = os.path.expanduser('~')
WORKSPACE_NAME = "Potted_Plant_Transport_Robot"
DEFAULT_MAP_DIR = os.path.join(HOME, WORKSPACE_NAME, "map", "pgm")
FALLBACK_MAP_PATH = os.path.join(DEFAULT_MAP_DIR, "default/map.yaml")

def _find_latest_map(map_dir):
    if not os.path.isdir(map_dir):
        return None
    candidates = glob.glob(os.path.join(map_dir, "**", "map.yaml"), recursive=True)
    if not candidates:
        return None
    candidates.sort(key=os.path.getmtime)
    return candidates[-1]

def generate_launch_description():
    robot_navigation_dir = get_package_share_directory('robot_navigation')

    default_map_path = FALLBACK_MAP_PATH
    print(f"[nav2_only] 使用地图: {default_map_path}")

    # Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_rviz = LaunchConfiguration('use_rviz', default='true')

    map_yaml_path = LaunchConfiguration('map', default=default_map_path)

    nav2_params_file = LaunchConfiguration(
        'nav2_params_file',
        default=os.path.join(robot_navigation_dir, 'params', 'nav2_params.yaml'),
    )
    autostart = LaunchConfiguration('autostart', default='true')

    nav_to_pose_bt_xml_default = PathJoinSubstitution([
        FindPackageShare('nav2_bt_navigator'),
        'behavior_trees',
        'navigate_to_pose_w_replanning_and_recovery.xml',
    ])
    nav_to_pose_bt_xml = LaunchConfiguration(
        'nav_to_pose_bt_xml',
        default=nav_to_pose_bt_xml_default,
    )

    lifecycle_nodes_navigation = [
        'controller_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother',
        'smoother_server'
    ]

    lifecycle_nodes_localization = ['map_server']

    # ========== 辅助工具节点 ==========

    # Clear Costmap Caller - 定时清除过期的代价地图数据 (依赖 Nav2 的 service，放在这里非常合适)
    clear_costmap_caller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('clear_costmap_caller'),
                'launch',
                'clear_costmap_caller.launch.py'
            )
        )
    )

    # ========== Nav2 核心栈 ==========

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[nav2_params_file, {'yaml_filename': map_yaml_path}, {'use_sim_time': use_sim_time}],
    )

    controller_server_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': use_sim_time}],
        remappings=[('cmd_vel', 'cmd_vel_nav')]
    )

    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': use_sim_time}]
    )

    behavior_server_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': use_sim_time}],
        remappings=[('cmd_vel', 'cmd_vel_nav')]
    )

    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[
            nav2_params_file,
            {'use_sim_time': use_sim_time},
            {'default_nav_to_pose_bt_xml': nav_to_pose_bt_xml},
        ]
    )

    waypoint_follower_node = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': use_sim_time}]
    )

    velocity_smoother_node = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': use_sim_time}],
        # 将平滑后的速度直接输出给 L1 层的 diff_drive_controller
        remappings=[
            ('cmd_vel', 'cmd_vel_nav'),
            ('cmd_vel_smoothed', '/diff_drive_controller/cmd_vel_unstamped')
        ]
    )

    smoother_server_node = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': use_sim_time}]
    )

    # ========== 生命周期管理器 ==========

    lifecycle_manager_localization_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': autostart},
            {'bond_timeout': 10.0},
            {'node_names': lifecycle_nodes_localization}
        ]
    )

    lifecycle_manager_navigation_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': autostart},
            {'bond_timeout': 10.0},
            {'node_names': lifecycle_nodes_navigation}
        ]
    )

    # ========== RViz ==========
    rviz_config = PathJoinSubstitution([FindPackageShare('robot_navigation'), 'rviz', 'nav2_default_view.rviz'])

    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ('/map', 'map'),
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
            ('/goal_pose', 'goal_pose'),
            ('/clicked_point', 'clicked_point'),
            ('/initialpose', 'initialpose'),
        ],
    )
    rviz_delayed = TimerAction(period=3.0, actions=[rviz_node])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('map', default_value=default_map_path),
        DeclareLaunchArgument('nav2_params_file', default_value=os.path.join(robot_navigation_dir, 'params', 'nav2_params.yaml')),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('nav_to_pose_bt_xml', default_value=nav_to_pose_bt_xml_default),

        # 辅助节点
        clear_costmap_caller_launch,

        # Nav2 Nodes
        map_server_node,
        controller_server_node,
        planner_server_node,
        behavior_server_node,
        bt_navigator_node,
        waypoint_follower_node,
        velocity_smoother_node,
        smoother_server_node,

        # Lifecycle Managers
        lifecycle_manager_localization_node,
        lifecycle_manager_navigation_node,
        rviz_delayed
    ])
