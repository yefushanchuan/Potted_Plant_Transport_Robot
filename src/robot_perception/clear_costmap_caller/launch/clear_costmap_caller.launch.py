"""
Clear Costmap Caller Launch File
定时清除 Nav2 代价地图中的过期障碍物
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # 获取配置文件路径
    pkg_dir = get_package_share_directory('clear_costmap_caller')
    config_file = os.path.join(pkg_dir, 'config', 'clear_costmap_caller.yaml')

    # 创建节点
    clear_costmap_caller_node = Node(
        package='clear_costmap_caller',
        executable='clear_costmap_caller',
        name='clear_costmap_caller',
        output='screen',
        parameters=[config_file]
    )

    return LaunchDescription([
        clear_costmap_caller_node
    ])
