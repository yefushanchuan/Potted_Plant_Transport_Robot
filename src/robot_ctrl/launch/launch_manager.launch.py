from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            "default_mode",
            default_value="navigation",
            description="launch_manager default mode: navigation or mapping",
        ),
        DeclareLaunchArgument(
            "default_launch_package",
            default_value="robot_bringup",
            description="default package for launch entries without package prefix",
        ),
        DeclareLaunchArgument(
            "common_launches",
            default_value="1_agrobot_base_bringup.launch.py",
            description="comma separated launch list, format launch.py or pkg:launch.py",
        ),
        DeclareLaunchArgument(
            "navigation_launches",
            default_value=(
                "agrobot_localization_bringup.launch.py,"
                "3_agrobot_nav_by_route_bringup.launch.py,"
                "pheno_module_controller.launch.py"
            ),
            description="comma separated launch list for navigation mode",
        ),
        DeclareLaunchArgument(
            "mapping_launches",
            default_value="2_agrobot_mapping_bringup.launch.py",
            description="comma separated launch list for mapping mode",
        ),
        Node(
            package="robot_ctrl",
            executable="launch_manager.py",
            name="launch_manager",
            output="screen",
            parameters=[{
                "default_mode": LaunchConfiguration("default_mode"),
                "default_launch_package": LaunchConfiguration("default_launch_package"),
                "common_launches": LaunchConfiguration("common_launches"),
                "navigation_launches": LaunchConfiguration("navigation_launches"),
                "mapping_launches": LaunchConfiguration("mapping_launches"),
            }],
        )
    ])
