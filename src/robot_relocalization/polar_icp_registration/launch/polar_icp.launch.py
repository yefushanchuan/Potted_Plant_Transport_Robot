import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    declare_pcd_path = DeclareLaunchArgument(
        'pcd_path',
        default_value='',
        description='Path to the PCD map file'
    )

    params_file = os.path.join(
        get_package_share_directory('polar_icp_registration'),
        'config',
        'polar_icp.yaml'
    )

    icp_node = Node(
        package='polar_icp_registration',
        executable='polar_icp_registration_node',
        output='screen',
        parameters=[
            params_file,
            {'pcd_path': LaunchConfiguration('pcd_path')}
        ]
    )

    return LaunchDescription([
        declare_pcd_path,
        icp_node
    ])
