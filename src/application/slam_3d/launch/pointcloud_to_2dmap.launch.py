from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 直接写绝对路径
    input_pcd = "/home/rpp/rpp_data/map/map.pcd"
    dest_directory = "/home/rpp/rpp_data/map"

    return LaunchDescription([
        Node(
            package='pointcloud_to_2dmap_ros2',
            executable='pointcloud_to_2dmap_ros2',
            name='pointcloud_to_2dmap_node',
            output='screen',
            parameters=[{
                'resolution': 0.05,   #地图每个像素对应的实际距离（米/像素）。比如 0.1 表示每个像素代表 10cm
                'map_width': 2000,   #生成的二维地图宽度，单位是像素
                'map_height': 2000,  #生成的二维地图高度，单位是像素
                'min_points_in_pix': 2,  # 一个像素点上最少需要累积多少点云点才认为有效，用于过滤稀疏点
                'max_points_in_pix': 3,  # 一个像素点上最多累积多少点云点，超过会映射到最大灰度值，用于归一化
                'min_height': -0.2,   # 只保留高度大于该值的点（米），用于去掉地面以下或低矮噪声点
                'max_height': 1.0,   # 只保留高度小于该值的点（米），用于去掉高处异常点
                'dest_directory': dest_directory,      # 输出地图文件（PNG和YAML）保存的目录
                'input_pcd': input_pcd,                # 输入点云文件（PCD格式）的路径
                'map_name': 'map',                     # 输出地图的名字，用于生成 PNG 文件名和 YAML 文件名
            }]
        )
    ])
