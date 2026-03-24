import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rviz_cfg = os.path.join(get_package_share_directory("slam_3d"), "rviz", "view_map.rviz")
    return LaunchDescription(
        [
            Node(
                package="pcl_ros",
                executable="pcd_to_pointcloud",
                output="screen",
                parameters=[
                    {
                        "file_name": "/home/rpp/rpp_data/map/map.pcd",
                        "tf_frame": "map",
                        "publishing_period_ms": 1000,
                    }
                ],
                remappings=[("cloud_pcd", "pcd_map")],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_cfg],
            ),
        ]
    )
