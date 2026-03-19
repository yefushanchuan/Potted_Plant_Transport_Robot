from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg = FindPackageShare("plant_detector")

    # ── Arguments ──────────────────────────────────────────────────────────
    lidar_topic_arg = DeclareLaunchArgument(
        "lidar_topic",
        default_value="/livox/lidar",
        description="PointCloud2 topic published by livox_ros_driver2",
    )

    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution([pkg, "config", "params.yaml"]),
        description="Path to the YAML parameter file",
    )

    # ── Node ───────────────────────────────────────────────────────────────
    detector_node = Node(
        package="plant_detector",
        executable="plant_detector_node",
        name="plant_detector_node",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            {"lidar_topic": LaunchConfiguration("lidar_topic")},
        ],
        remappings=[
            # If your driver publishes on a different topic, add remappings here:
            # ("/livox/lidar", "/your/custom/topic"),
        ],
    )

    return LaunchDescription([
        lidar_topic_arg,
        params_file_arg,
        detector_node,
    ])
