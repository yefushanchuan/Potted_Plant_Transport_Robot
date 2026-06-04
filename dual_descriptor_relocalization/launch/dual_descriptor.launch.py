import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context):
    pkg_share = get_package_share_directory('dual_descriptor_relocalization')
    params_file = os.path.join(pkg_share, 'config', 'dual_descriptor.yaml')

    return [
        Node(
            package='dual_descriptor_relocalization',
            executable='dual_descriptor_node',
            name='dual_descriptor_node',
            output='screen',
            parameters=[
                params_file,
                {
                    'keyframe_db': LaunchConfiguration('keyframe_db'),
                }
            ],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('keyframe_db', default_value='',
                              description='Path to keyframe database directory'),
        DeclareLaunchArgument('log_level', default_value='INFO',
                              description='ROS2 log level: DEBUG/INFO/WARN/ERROR/FATAL'),

        OpaqueFunction(function=_launch_setup),
    ])
