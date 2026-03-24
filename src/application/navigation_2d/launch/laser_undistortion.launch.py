from sympy import false
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="lidar_undistortion_2d",
                executable="lidar_undistortion_2d_node",
                name="lidar_undistortion_2d_node_front",
                output="screen",
                parameters=[
                    {
                        "laser_frame": "laser_F_link",
                        "imu_frame": "lidar_F_imu_link",
                    }
                ],
                remappings=[
                    ("/scan", "/laser_F/scan"),
                    ("/imu", "/lidar_F/imu_data"),
                    ("/scan_undistort", "/laser_F/scan_undistort"),
                ],
            ),
            Node(
                package="lidar_undistortion_2d",
                executable="lidar_undistortion_2d_node",
                name="lidar_undistortion_2d_node_back",
                output="screen",
                parameters=[
                    {
                        "laser_frame": "laser_B_link",
                        "imu_frame": "lidar_F_imu_link",
                    }
                ],
                remappings=[
                    ("/scan", "/laser_B/scan"),
                    ("/imu", "/lidar_F/imu_data"),
                    ("/scan_undistort", "/laser_B/scan_undistort"),
                ],
            ),
        ]
    )
