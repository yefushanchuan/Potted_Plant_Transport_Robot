"""
Nav2 Launch File导航启动文件

使用方法：
1. 先启动机器人基础节点：
   ros2 launch robot_base agrobot_base_bringup.launch.py

2. 再启动定位节点：
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
# 获取当前用户的家目录（即 /home/zx）
HOME = os.path.expanduser('~')
# 动态拼接你的工作空间和地图目录
WORKSPACE_NAME = "Potted_Plant_Transport_Robot"
DEFAULT_MAP_DIR = os.path.join(HOME, WORKSPACE_NAME, "map", "pgm")

# 备用固定地图路径（可以指向一个确定存在的初始图）
FALLBACK_MAP_PATH = os.path.join(DEFAULT_MAP_DIR, "default/map.yaml")


def _find_latest_map(map_dir):
    """
    查找指定目录下最新的 map.yaml 地图文件（递归搜索子目录）
    
    Args:
        map_dir: 地图目录路径
    
    Returns:
        最新地图文件的完整路径，如果未找到返回 None
    """
    if not os.path.isdir(map_dir):
        return None
    
    # 递归查找所有 map.yaml 地图配置文件
    candidates = glob.glob(os.path.join(map_dir, "**", "map.yaml"), recursive=True)
    if not candidates:
        return None
    
    # 按修改时间排序，返回最新的
    candidates.sort(key=os.path.getmtime)
    return candidates[-1]


def generate_launch_description():
    # Directories
    robot_navigation_dir = get_package_share_directory('robot_navigation')

    # 自动查找最新地图
    # latest_map = _find_latest_map(DEFAULT_MAP_DIR)
    default_map_path = FALLBACK_MAP_PATH
    print(f"[nav2_only] 使用地图: {default_map_path}")

    # Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_rviz = LaunchConfiguration('use_rviz', default='true')

    # 2D Map for Nav2 (Costmaps)
    map_yaml_path = LaunchConfiguration(
        'map',
        default=default_map_path
    )

    nav2_params_file = LaunchConfiguration(
        'nav2_params_file',
        default=os.path.join(robot_navigation_dir, 'params', 'nav2_params.yaml'),
    )
    autostart = LaunchConfiguration('autostart', default='true')

    # 行为树配置
    nav_to_pose_bt_xml_default = PathJoinSubstitution(
        [
            FindPackageShare('nav2_bt_navigator'),
            'behavior_trees',
            'navigate_to_pose_w_replanning_and_recovery.xml',
        ]
    )
    nav_to_pose_bt_xml = LaunchConfiguration(
        'nav_to_pose_bt_xml',
        default=nav_to_pose_bt_xml_default,
    )

    # Lifecycle nodes to be managed
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

    # ========== Perception Nodes ==========

    # IMU Complementary Filter - 融合 IMU 数据
    imu_complementary_filter_node = Node(
        package='imu_complementary_filter',
        executable='complementary_filter_node',
        name='complementary_filter_gain_node',
        output='screen',
        parameters=[
            {'do_bias_estimation': True},
            {'do_adaptive_gain': True},
            {'use_mag': False},
            {'gain_acc': 0.01},
            {'gain_mag': 0.01},
            {'publish_tf': False},  # 禁用TF发布，避免与Fast-LIO冲突
        ],
        remappings=[
            ('/imu/data_raw', '/livox/imu'),
        ]
    )

    # Ground Segmentation - 地面分割
    # ground_segmentation_params_file = os.path.join(
    #     get_package_share_directory('linefit_ground_segmentation_ros'),
    #     'launch',
    #     'segmentation_params.yaml'
    # )
    # ground_segmentation_node = Node(
    #     package='linefit_ground_segmentation_ros',
    #     executable='ground_segmentation_node',
    #     name='ground_segmentation',
    #     output='screen',
    #     parameters=[ground_segmentation_params_file]
    # )

    # Pointcloud to LaserScan - 直接使用原始点云（不经过地面分割）
    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan', 
        executable='pointcloud_to_laserscan_node',
        remappings=[('cloud_in', '/livox/lidar'),
                    ('scan', '/scan_raw')],
        parameters=[{
            'target_frame': 'base_footprint',  # 用地面中心作为基准
            'transform_tolerance': 0.05,       # 稍微加大一点容忍度，防止TF延迟导致丢帧
            'min_height': 0.15,                # 离地15cm以上
            'max_height': 1.0,                 # 只要不超过车身高度，尽量设高点，弥补雷达倾斜带来的数据丢失
            'angle_min': -3.14159,             # 小车底盘的正后方(右转180度)
            'angle_max': 3.14159,              # 小车底盘的正后方(左转180度)
            'angle_increment': 0.0087,         # 0.5度
            'scan_time': 0.1,                  # 10Hz
            'range_min': 0.2,                  # 滤掉车体自身
            'range_max': 15.0,                 # 探测15米
            'use_inf': True,
            'inf_epsilon': 1.0
        }],
    )

    # Laser Filter - 过滤离群点
    laser_filter_node = Node(
        package='robot_base',
        executable='laser_filter_node_exe',
        name='lidar_filter',
        output='screen',
        parameters=[
            {'source_topic': '/scan_raw'},
            # Nav2 默认使用 /scan，这里直接发布为 /scan 以对齐 costmap 配置
            {'pub_topic': '/scan'},
            {'outlier_threshold': 0.1},
        ],
    )

    # Clear Costmap Caller - 定时清除过期的代价地图数据
    clear_costmap_caller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('clear_costmap_caller'),
                'launch',
                'clear_costmap_caller.launch.py'
            )
        )
    )

    # ========== Nav2 Nodes ==========

    # Map Server
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            nav2_params_file,
            {'yaml_filename': map_yaml_path},
            {'use_sim_time': use_sim_time},
        ],
    )

    # Controller Server
    controller_server_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': use_sim_time}],
        remappings=[('cmd_vel', 'cmd_vel_nav')]
    )

    # Planner Server
    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': use_sim_time}]
    )

    # Behavior Server
    behavior_server_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': use_sim_time}],
        remappings=[('cmd_vel', 'cmd_vel_nav')]
    )

    # BT Navigator
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

    # Waypoint Follower
    waypoint_follower_node = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': use_sim_time}]
    )

    # Velocity Smoother
    velocity_smoother_node = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': use_sim_time}],
        # Humble: 输入 cmd_vel_nav，输出到 diff_drive_controller
        remappings=[
            ('cmd_vel', 'cmd_vel_nav'),
            ('cmd_vel_smoothed', '/diff_drive_controller/cmd_vel_unstamped')
        ]
    )

    # Smoother Server
    smoother_server_node = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': use_sim_time}]
    )

    # Lifecycle Manager for Localization (Map Server)
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

    # Lifecycle Manager for Navigation
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
    rviz_config = PathJoinSubstitution(
        [FindPackageShare('robot_navigation'), 'rviz', 'nav2_default_view.rviz']
    )

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

    # RViz 依赖 TF（尤其是 map->odom）与 /map 初始化；启动过早时容易出现“看不到地图/局部代价地图需要点两次”的体验。
    rviz_delayed = TimerAction(period=3.0, actions=[rviz_node])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'map',
            default_value=default_map_path,
            description='Path to 2D Map YAML file for Nav2 (默认读取最新地图)',
        ),
        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=os.path.join(robot_navigation_dir, 'params', 'nav2_params.yaml'),
            description='Nav2 parameters',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Whether to start RVIZ',
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically startup the nav2 stack',
        ),
        DeclareLaunchArgument(
            'nav_to_pose_bt_xml',
            default_value=nav_to_pose_bt_xml_default,
            description='BT XML used by NavigateToPose in bt_navigator'
        ),

        # Perception Nodes
        imu_complementary_filter_node,
        # ground_segmentation_node,  # 已禁用：直接使用原始点云
        pointcloud_to_laserscan_node,
        laser_filter_node,
        # clear_costmap_caller_launch,

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
