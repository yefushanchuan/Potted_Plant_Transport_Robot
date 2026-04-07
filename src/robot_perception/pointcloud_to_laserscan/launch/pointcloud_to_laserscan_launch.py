from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            name='scanner', default_value='scanner',
            description='Namespace for sample topics'
        ),
        Node(
            package='pointcloud_to_laserscan', 
            executable='pointcloud_to_laserscan_node',
            remappings=[('cloud_in', '/livox/lidar'),
                        ('scan', '/scan')],
            parameters=[{
                'target_frame': 'base_footprint',  # 用地面中心作为基准
                'transform_tolerance': 0.05,       # 稍微加大一点容忍度，防止TF延迟导致丢帧
                'min_height': 0.15,                # 离地15cm以上
                'max_height': 1.0,                 # 只要不超过车身高度，尽量设高点，弥补雷达倾斜带来的数据丢失
                'angle_min': -3.14159,             # 小车底盘的正后方(右转180度)
                'angle_max': 3.14159,              # 小车底盘的正后方(左转180度)
                'angle_increment': 0.0087,         # 0.5度
                'scan_time': 0.1,                  # 10Hz
                'range_min': 0.2,                  # 滤掉车体自身
                'range_max': 15.0,                 # 探测15米
                'use_inf': True,
                'inf_epsilon': 1.0
            }],
        ),
    ])
