import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('icp_registration')
    params_file = os.path.join(pkg_dir, 'config', 'icp.yaml')
    
    map_filename_arg = DeclareLaunchArgument(
        'map_filename',
        default_value='',
        description='Absolute path to the map PCD file'
    )
    
    node = Node(
        package='icp_registration',
        executable='icp_registration_node',
        output='screen',
        parameters=[
            params_file,
            {'map_filename': LaunchConfiguration('map_filename')} 
        ]
    )
    
    return LaunchDescription([
        map_filename_arg,
        node
    ])
