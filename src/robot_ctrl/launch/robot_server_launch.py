from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_robot_ctrl = get_package_share_directory('robot_ctrl')
    roompose_file = LaunchConfiguration('roompose_file')

    declare_roompose_arg = DeclareLaunchArgument(
        'roompose_file',
        default_value=PathJoinSubstitution([pkg_robot_ctrl, 'roompose', 'roompose.json']),
        description='房间位姿 JSON 文件路径'
    )
    recorder_node = Node(
        package='robot_ctrl',
        executable='roompose_recorder',
        name='roompose_recorder',
        output='screen',
        parameters=[{'roompose_file_path': roompose_file}]  
    )
    loader_node = Node(
        package='robot_ctrl',
        executable='roompose_loader',
        name='roompose_loader',
        output='screen',
        parameters=[{'roompose_file_path': roompose_file}]  
    )

    return LaunchDescription([
        declare_roompose_arg,
        recorder_node,
        loader_node
    ])
