"""
Nav2 Launch File导航启动文件

使用方法：
1. 先启动机器人基础节点 (感知容器与状态估计容器)：
   ros2 launch robot_base agrobot_base_bringup.launch.py

2. 再启动定位节点 (FAST-LIO / ICP 等)：
   ros2 launch indoor_location run_argbot_mid360.launch.py

3. 最后启动此文件进行导航：

   # 方式1: 使用相对路径（基于工作空间 ~/Potted_Plant_Transport_Robot/）
   ros2 launch robot_nav nav2_simple.launch.py map:=map/pgm/office/map.yaml
   
   # 方式2: 使用绝对路径
   ros2 launch robot_nav nav2_simple.launch.py map:=/home/user/maps/office.yaml
   
   # 方式3: 不传参，自动查找最新地图
   ros2 launch robot_nav nav2_simple.launch.py

注意：此 launch 文件假设 TF 树已经由上述两个 launch 文件正确建立：
  map -> odom -> base_footprint -> base_link -> sensors
"""

import os
import glob

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# ========== 地图路径处理 ==========
HOME = os.path.expanduser('~')
WORKSPACE_NAME = "Potted_Plant_Transport_Robot"
WORKSPACE_DIR = os.path.join(HOME, WORKSPACE_NAME)
DEFAULT_MAP_DIR = os.path.join(WORKSPACE_DIR, "map", "pgm")

def _find_latest_map(map_dir):
    """查找地图目录中最新的 map.yaml"""
    if not os.path.isdir(map_dir):
        print(f"[nav2_only] 警告: 地图目录不存在: {map_dir}")
        return None
    
    candidates = glob.glob(os.path.join(map_dir, "**", "map.yaml"), recursive=True)
    if not candidates:
        print(f"[nav2_only] 警告: 在 {map_dir} 中未找到 map.yaml")
        return None
    
    candidates.sort(key=os.path.getmtime)
    latest = candidates[-1]
    print(f"[nav2_only] 自动选择最新地图: {latest}")
    return latest

def _resolve_map_path(map_path):
    """
    解析地图路径：
    - 绝对路径：直接使用（检查存在性且必须是.yaml文件）
    - 相对路径：基于工作空间目录拼接（检查存在性且必须是.yaml文件）
    - 空/None：自动查找最新地图
    """
    if not map_path:
        return _find_latest_map(DEFAULT_MAP_DIR)
    
    # 已经是绝对路径
    if os.path.isabs(map_path):
        # 将 exists 改为 isfile，并限制后缀为 .yaml
        if os.path.isfile(map_path) and map_path.endswith('.yaml'):
            return map_path
        else:
            print(f"[nav2_only] 警告: 指定的绝对路径不存在、是一个目录或不是 .yaml 文件: {map_path}")
            return None
    
    # 相对路径 -> 基于工作空间拼接
    resolved = os.path.normpath(os.path.join(WORKSPACE_DIR, map_path))
    
    # 同样将 exists 改为 isfile，并限制后缀为 .yaml
    if os.path.isfile(resolved) and resolved.endswith('.yaml'):
        return resolved
    else:
        print(f"[nav2_only] 警告: 相对路径解析后不存在、是一个目录或不是 .yaml 文件: {resolved} (原始输入: {map_path})")
        return None

def _get_map_yaml_path(context):
    """OpaqueFunction: 运行时解析地图路径"""
    # 获取 launch 参数的实际值
    map_val = context.launch_configurations.get('map', '')
    
    resolved = _resolve_map_path(map_val)
    
    if resolved is None:
        # 所有策略都失败，使用硬编码默认
        fallback = os.path.join(DEFAULT_MAP_DIR, "default", "map.yaml")
        print(f"[nav2_only] 回退到默认地图: {fallback}")
        resolved = fallback
    
    # 将解析后的路径设置到配置中，供后续使用
    context.launch_configurations['map_yaml_resolved'] = resolved
    return []

def generate_launch_description():
    robot_navigation_dir = get_package_share_directory('robot_nav')

    # ========== Launch Arguments ==========
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    
    # map 参数：空字符串表示自动查找
    map_arg = LaunchConfiguration('map', default='')

    nav2_params_file = LaunchConfiguration(
        'nav2_params_file',
        default=os.path.join(robot_navigation_dir, 'config', 'navigation', 'Nav_DWB_low.yaml'),
    )
    autostart = LaunchConfiguration('autostart', default='true')

    nav_to_pose_bt_xml_default = PathJoinSubstitution([
        FindPackageShare('robot_nav'),
        'config',
        'navigation',
        'navigate_to_pose.xml',
    ])
    nav_to_pose_bt_xml = LaunchConfiguration(
        'nav_to_pose_bt_xml',
        default=nav_to_pose_bt_xml_default,
    )

    # ========== 地图路径解析 Action ==========
    # 使用 OpaqueFunction 在运行时解析路径
    resolve_map_action = OpaqueFunction(function=_get_map_yaml_path)

    # ========== 辅助工具节点 ==========
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
    
    # 使用 LaunchConfiguration 引用解析后的路径
    map_yaml_path = LaunchConfiguration('map_yaml_resolved')

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
    rviz_config = PathJoinSubstitution([FindPackageShare('robot_nav'), 'rviz', 'nav2_default_view.rviz'])

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
        DeclareLaunchArgument('map', default_value='', description='地图路径（绝对路径或相对于 ~/Potted_Plant_Transport_Robot/ 的路径，留空则自动查找最新）'),
        DeclareLaunchArgument('nav2_params_file', default_value=os.path.join(robot_navigation_dir, 'config', 'navigation', 'Nav_DWB_low.yaml')),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('nav_to_pose_bt_xml', default_value=nav_to_pose_bt_xml_default),

        # 先执行路径解析
        resolve_map_action,

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
