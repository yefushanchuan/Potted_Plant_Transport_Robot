import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg_robot_nav = get_package_share_directory("robot_nav")
    pkg_robot_route = get_package_share_directory("robot_route")

    use_sim_time = LaunchConfiguration("use_sim_time")
    namespace = LaunchConfiguration("namespace")

    map_file = LaunchConfiguration("map_file")

    graph_filepath = LaunchConfiguration("graph_filepath")
    route_frame = LaunchConfiguration("route_frame")
    base_frame = LaunchConfiguration("base_frame")
    sample_resolution = LaunchConfiguration("sample_resolution")
    autostart = LaunchConfiguration("autostart")
    start_route_server_delay = LaunchConfiguration("start_route_server_delay")

    use_nav2 = LaunchConfiguration("use_nav2")
    nav2_params_file = LaunchConfiguration("nav2_params_file")
    start_nav2_delay = LaunchConfiguration("start_nav2_delay")

    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    start_rviz_delay = LaunchConfiguration("start_rviz_delay")
 
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
        name='pointcloud_to_laserscan',
        output='screen',
        parameters=[{
            'target_frame': 'base_link',
            'transform_tolerance': 0.1,
            'min_height': 0.05,            # 过滤地面点：只保留 base_link 上方 5cm 以上的点（地面约在 -0.11m）
            'max_height': 1.5,             # 最大高度 1.5m，过滤天花板等高处障碍
            'angle_min': -3.14159,         # 360度视野
            'angle_max': 3.14159,
            'angle_increment': 0.0043,
            'scan_time': 0.1,
            'range_min': 0.6,              # 过滤车体附近的点
            'range_max': 100.0,
            'use_inf': True,
            'inf_epsilon': 1.0
        }],
        remappings=[
            ('cloud_in', '/livox/lidar'),  # 直接使用原始点云
            ('scan', '/scan_raw')                      # 输出未过滤的激光数据
        ]
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

    nav2_rewritten_yaml = RewrittenYaml(
        source_file=nav2_params_file,
        root_key=namespace,
        param_rewrites={
            "use_sim_time": use_sim_time,
            "bt_navigator.ros__parameters.default_nav_to_pose_bt_xml": PathJoinSubstitution(
                [pkg_robot_nav, "config", "navigation", "nav_to_pose_w_route_tree.xml"]
            ),
        },
        convert_types=True,
    )
    nav2_configured_params = ParameterFile(nav2_rewritten_yaml, allow_substs=True)

    nav2_remaps = [
        ("/tf", "tf"),
        ("/tf_static", "tf_static"),
    ]

    nav2_nodes = GroupAction(
        scoped=True,
        actions=[
            PushRosNamespace(namespace),
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                parameters=[
                    nav2_params_file,
                    {'yaml_filename': map_file},
                ],
            ),
            Node(
                package="nav2_controller",
                executable="controller_server",
                name="controller_server",
                output="screen",
                parameters=[nav2_configured_params],
                remappings=nav2_remaps + [("cmd_vel", "/cmd_vel_nav")],
            ),
            Node(
                package="nav2_smoother",
                executable="smoother_server",
                name="smoother_server",
                output="screen",
                parameters=[nav2_configured_params],
                remappings=nav2_remaps,
            ),
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                output="screen",
                parameters=[nav2_configured_params],
                remappings=nav2_remaps,
            ),
            Node(
                package="nav2_behaviors",
                executable="behavior_server",
                name="behavior_server",
                output="screen",
                parameters=[nav2_configured_params],
                remappings=nav2_remaps + [("cmd_vel", "/cmd_vel_nav")],
            ),
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator",
                output="screen",
                parameters=[nav2_configured_params],
                remappings=nav2_remaps,
            ),
            Node(
                package="nav2_waypoint_follower",
                executable="waypoint_follower",
                name="waypoint_follower",
                output="screen",
                parameters=[nav2_configured_params],
                remappings=nav2_remaps,
            ),
            Node(
                package="nav2_velocity_smoother",
                executable="velocity_smoother",
                name="velocity_smoother",
                output="screen",
                parameters=[nav2_configured_params],
                remappings=nav2_remaps + [("cmd_vel", "cmd_vel_nav"), ("cmd_vel_smoothed", "/diff_drive_controller/cmd_vel_unstamped")],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                output="screen",
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"autostart": True},
                    {
                        "node_names": [
                            "map_server",
                            "controller_server",
                            "smoother_server",
                            "planner_server",
                            "behavior_server",
                            "bt_navigator",
                            "waypoint_follower",
                            "velocity_smoother",
                        ]
                    },
                ],
            ),
        ],
        condition=IfCondition(use_nav2),
    )

    nav2_navigation_delayed = TimerAction(
        period=start_nav2_delay,
        actions=[nav2_nodes],
    )

    route_graph_server_launch = TimerAction(
        period=start_route_server_delay,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_robot_route, "launch", "route_graph_route_server.launch.py")
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "namespace": namespace,
                    "graph_filepath": graph_filepath,
                    "route_frame": route_frame,
                    "base_frame": base_frame,
                    "sample_resolution": sample_resolution,
                    "autostart": autostart,
                }.items(),
            ),
        ],
    )

    rviz2_node = TimerAction(
        period=start_rviz_delay,
        actions=[
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                namespace=namespace,
                output="screen",
                arguments=["-d", rviz_config],
                parameters=[{"use_sim_time": use_sim_time}],
                condition=IfCondition(use_rviz),
            )
        ],
    )

    default_map_file = PathJoinSubstitution([pkg_robot_nav, "maps", "map.yaml"])
    default_graph_file = os.path.join(pkg_robot_route, "route", "test_arc.yaml")
    default_rviz_config = PathJoinSubstitution([pkg_robot_nav, "rviz", "nav.rviz"])
    default_nav2_params = PathJoinSubstitution([pkg_robot_nav, "config", "navigation", "Nav_route.yaml"])

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="False",
                description="Use simulation clock if true",
            ),
            DeclareLaunchArgument(
                "namespace",
                default_value="",
                description="Top-level namespace for all nodes",
            ),
            DeclareLaunchArgument(
                "map_file",
                default_value=default_map_file,
                description="Path to map_server serialized state (*.yaml)",
            ),
            DeclareLaunchArgument(
                "graph_filepath",
                default_value=default_graph_file,
                description="Route graph YAML filepath",
            ),
            DeclareLaunchArgument(
                "route_frame",
                default_value="map",
                description="Route graph frame (output path/frame_id)",
            ),
            DeclareLaunchArgument(
                "base_frame",
                default_value="base_footprint",
                description="Robot base frame for TF lookup",
            ),
            DeclareLaunchArgument(
                "sample_resolution",
                default_value="0.02",
                description="Path densification resolution (m)",
            ),
            DeclareLaunchArgument(
                "start_route_server_delay",
                default_value="6.0",
                description="Seconds to delay starting route graph server after base/carto startup",
            ),
            DeclareLaunchArgument(
                "autostart",
                default_value="True",
                description="Lifecycle manager autostart for route graph server",
            ),
            DeclareLaunchArgument(
                "use_nav2",
                default_value="True",
                description="Launch Nav2 navigation stack if true",
            ),
            DeclareLaunchArgument(
                "nav2_params_file",
                default_value=default_nav2_params,
                description="Nav2 params YAML (bt_navigator uses route tree inside)",
            ),
            DeclareLaunchArgument(
                "start_nav2_delay",
                default_value="8.0",
                description="Seconds to delay starting Nav2 after base/carto startup",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="True",
                description="Launch RViz2 if true",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=default_rviz_config,
                description="Path to RViz2 configuration file",
            ),
            DeclareLaunchArgument(
                "start_rviz_delay",
                default_value="12.0",
                description="Seconds to delay starting RViz2",
            ),
            pointcloud_to_laserscan_node,
            laser_filter_node,
            nav2_navigation_delayed,
            route_graph_server_launch,
            rviz2_node,
        ]
    )
