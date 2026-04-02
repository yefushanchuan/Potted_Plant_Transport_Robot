import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory("pcd2pgm")

    params_file = LaunchConfiguration("params_file")

    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(bringup_dir, "config", "pcd2pgm_online.yaml"),
        description="Full path to the ROS2 parameters file to use",
    )

    start_pcd2pgm_online_cmd = Node(
        package="pcd2pgm",
        executable="pcd2pgm_online_node",
        name="pcd2pgm_online",
        output="screen",
        parameters=[params_file],
    )

    ld = LaunchDescription()
    ld.add_action(declare_params_file_cmd)
    ld.add_action(start_pcd2pgm_online_cmd)
    return ld
