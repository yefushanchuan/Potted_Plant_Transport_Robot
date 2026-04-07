import os
import launch
import launch_ros.actions
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetLaunchConfiguration, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    play_bag = LaunchConfiguration("play_bag", default="false")
    bag_path = LaunchConfiguration("bag_path", default="")
    enable_tilt_compensator = LaunchConfiguration("enable_tilt_compensator", default="false")

    # Paths
    rviz_cfg = PathJoinSubstitution(
        [FindPackageShare("robot_bringup"), "rviz", "BA.rviz"]
    )
    ba_config_path = PathJoinSubstitution(
        [FindPackageShare("robot_bringup"), "config", "3d_ba_optimize", "ba_agrobot_mid360.yaml"] 
    )
    lio_config_path = PathJoinSubstitution(
        [FindPackageShare("robot_bringup"), "config", "3d_fastlio2", "agrobot_mid360.yaml"]   
    )

    return launch.LaunchDescription(
        [
            # Args
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use sim time if true",
            ),
            DeclareLaunchArgument(
                "play_bag",
                default_value="false",
                description="Play rosbag with /clock; forces use_sim_time:=true",
            ),
            DeclareLaunchArgument(
                "bag_path",
                default_value="",
                description="rosbag2 folder or .db3 path when play_bag:=true",
            ),
            DeclareLaunchArgument(
                "enable_tilt_compensator",
                default_value="false",
                description="Enable robot_base tilt_compensator (default: false)",
            ),
            SetLaunchConfiguration(
                "use_sim_time",
                "true",
                condition=IfCondition(play_bag),
            ),

            # FastLIO2 Node
            launch_ros.actions.Node(
                package="fastlio2",
                namespace="fastlio2",
                executable="fastlio2_node",
                name="fastlio2_node",
                output="screen",
                remappings=[
                    ("tf", "/tf"),
                    ("tf_static", "/tf_static"),
                ],
                parameters=[{
                    "config_path": lio_config_path,
                    "use_sim_time": use_sim_time}]
            ),

            # BA Optimize Node
            launch_ros.actions.Node(
                package="ba_optimize",
                namespace="ba_optimize",
                executable="ba_node",
                name="ba_node",
                output="screen",
                remappings=[
                    ("tf", "/tf"),
                    ("tf_static", "/tf_static"),
                ],
                parameters=[{"config_path": ba_config_path,
                            "use_sim_time": use_sim_time}],
                arguments=['--ros-args', '--log-level', 'WARN'],
            ),

            # OctoMap Server
            launch_ros.actions.Node(
                package='octomap_server',
                executable='octomap_server_node',
                name='octomap_server',
                output='screen',
                parameters=[
                    {'resolution': 0.05}, # From pcd2pgm.yaml map_resolution
                    {'frame_id': 'map'},
                    {'base_frame_id': 'base_footprint'},
                    {'sensor_model/max_range': 40.0},
                    {'incremental_2D_projection': False},
                    {'occupancy_min_z': 0.2}, # Standard config
                    {'occupancy_max_z': 1.4}, # Standard config
                    {'sensor_model/miss': 0.1},
                    {'sensor_model/hit': 1.5},
                    
                    # Filtering params derived from pcd2pgm.yaml / original launch
                    {'pointcloud_min_x': -100.0}, # Increased range
                    {'pointcloud_max_x': 100.0},
                    {'pointcloud_min_y': -100.0},
                    {'pointcloud_max_y': 100.0},
                    {'pointcloud_min_z': 0.0}, # From pcd2pgm.yaml thre_z_min
                    {'pointcloud_max_z': 1.0}, # From pcd2pgm.yaml thre_z_max
                    {'filter_ground': True},
                    {'ground_filter/distance': 0.04},
                    {'ground_filter/angle': 0.15},
                    {'ground_filter/plane_distance': 0.07},
                    {'use_sim_time': use_sim_time}
                ],
                remappings=[
                    ('cloud_in', '/fastlio2/world_cloud'),
                    ('projected_map', 'map') # Standardize map topic
                ]
            ),

            # Nav2 Map Saver Server
            launch_ros.actions.Node(
                package='nav2_map_server',
                executable='map_saver_server',
                name='map_saver',
                output='screen',
                parameters=[{'save_map_timeout': 10.0}, 
                           {'use_sim_time': use_sim_time}]
            ),

            # Lifecycle Manager for Map Saver (delayed to ensure map_saver_server is ready)
            TimerAction(
                period=3.0,
                actions=[
                    launch_ros.actions.Node(
                        package='nav2_lifecycle_manager',
                        executable='lifecycle_manager',
                        name='lifecycle_manager_map_saver',
                        output='screen',
                        parameters=[
                            {'use_sim_time': use_sim_time},
                            {'autostart': True},
                            {'node_names': ['map_saver']}
                        ]
                    ),
                ]
            ),

            # # Map Save Service Node (统一地图保存服务)
            # launch_ros.actions.Node(
            #     package='ba_optimize',
            #     executable='map_save_service_node.py',
            #     name='map_save_service_node',
            #     output='screen',
            #     parameters=[{'use_sim_time': use_sim_time}]
            # ),

            # RViz2
            launch_ros.actions.Node(
                package="rviz2",
                namespace="ba_optimize",
                executable="rviz2",
                name="rviz2",
                output="screen",
                remappings=[
                    ("tf", "/tf"),
                    ("tf_static", "/tf_static"),
                ],
                arguments=["-d", rviz_cfg],
                parameters=[
                    {"use_sim_time": use_sim_time}
                ],
            ),

            # Play Bag
            ExecuteProcess(
                condition=IfCondition(play_bag),
                cmd=["ros2", "bag", "play", bag_path, "--clock", "--rate", "1.0"],
                output="screen"
            ),
        ]
    )
