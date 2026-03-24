import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rviz_cfg = os.path.join(get_package_share_directory("slam_3d"), "rviz", "mapping.rviz")
    lio_config_path = os.path.join(get_package_share_directory("slam_3d"), "config", "qr100_livox_mid360.yaml")
    ba_config_path = os.path.join(get_package_share_directory("slam_3d"), "config", "ba_qr100_livox_mid360.yaml")

    return LaunchDescription(
        [
             IncludeLaunchDescription(
                 PythonLaunchDescriptionSource(
                     [
                         PathJoinSubstitution(
                             [
                                 FindPackageShare("robot_bringup"),
                                 "launch",
                                 "robot_driver.launch.py",
                             ]
                         )
                     ]
                 ),
             ),
            Node(
                package="fastlio2",
                namespace="fastlio2",
                executable="fastlio2_node",
                name="fastlio2_node",
                output="screen",
                parameters=[{"config_path": lio_config_path}],

            ),
            Node(
                package="ba_optimize",
                namespace="ba_optimize",
                executable="ba_node",
                name="ba_node",
                output="screen",
                parameters=[{"config_path": ba_config_path}],
                arguments=['--ros-args', '--log-level', 'WARN'],#终端仅仅输出WARN调试日志
                
            ),
            Node(
                package="rviz2",
                namespace="ba_optimize",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_cfg],
            ),
            Node(
                package="octomap_server",
                executable="octomap_server_node",
                name="octomap_server",
                output="screen",
                parameters=[
                    {"resolution": 0.05},
                    {"frame_id": "map"},
                    {"base_frame_id": "base_link"},
                    {"pointcloud_max_z": 2.0},
                    {"pointcloud_min_z": -1.0},
                    {"occupancy_max_z": 0.5},
                    {"occupancy_min_z": -0.2},
                    {"filter_ground_plane": False},
                ],
                remappings=[("cloud_in", "/fastlio2/world_cloud")],
                arguments=[],
            ),
        ]
    )
