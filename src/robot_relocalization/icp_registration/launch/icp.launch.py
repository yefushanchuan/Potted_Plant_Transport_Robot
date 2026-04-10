import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    # 1. 变量命名更清晰（可选但推荐）
    declare_pcd_path = DeclareLaunchArgument(
        'pcd_path',
        default_value='',
        description='Path to the PCD map file'
    )
    
    # 2. 参数文件路径
    params_file = os.path.join(
        get_package_share_directory('icp_registration'), 
        'config', 
        'icp.yaml'
    )
    
    # 3. 创建节点
    icp_node = Node(
        package='icp_registration',
        executable='icp_registration_node',
        output='screen',
        parameters=[
            params_file,                           # 加载 YAML 配置
            {'pcd_path': LaunchConfiguration('pcd_path')}  # 覆盖参数
        ]
    )
    
    return LaunchDescription([
        declare_pcd_path,  # 先声明参数
        icp_node           # 再启动节点
    ])
