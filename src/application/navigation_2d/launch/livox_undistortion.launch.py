from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="livox_undistortion",
                executable="livox_undistortion_node",
                name="livox_undistortion_node",
                output="screen",
                parameters=[{"ExtIL": [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}],  # Default identity transform
                remappings=[
                    ("/livox/lidar", "/lidar_F/pointcloud"),
                    ("/livox/imu", "/lidar_F/imu_data"),
                    ("/livox/lidar_undistort", "/lidar_F/pointcloud_undistort"),
                ],
            ),
        ]
    )
