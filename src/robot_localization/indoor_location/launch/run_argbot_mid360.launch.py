import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def _resolve_map_path(raw_map_file, raw_map_dir, raw_map_name):
    """仅仅负责拼接地图路径"""
    map_file = (raw_map_file or "").strip()
    map_dir = (raw_map_dir or "").strip()
    map_name = (raw_map_name or "").strip()

    if map_file:
        expanded_file = os.path.expandvars(os.path.expanduser(map_file))
        if os.path.isabs(expanded_file): return expanded_file
        if map_dir:
            expanded_dir = os.path.expandvars(os.path.expanduser(map_dir))
            return os.path.abspath(os.path.join(expanded_dir, expanded_file))
        return os.path.abspath(expanded_file)

    if map_name:
        if not map_dir: map_dir = os.environ.get("ROBOT_MAP_DIR", "")
        if not map_dir: map_dir = os.path.join(get_package_share_directory("robot_bringup"), "pcd")
        expanded_dir = os.path.expandvars(os.path.expanduser(map_dir))
        return os.path.abspath(os.path.join(expanded_dir, map_name))
    return ""

def _launch_setup(context):
    """在此动态推断参数并生成所有节点"""
    map_file = LaunchConfiguration("map_file").perform(context)
    map_dir = LaunchConfiguration("map_dir").perform(context)
    map_name = LaunchConfiguration("map_name").perform(context)
    bag_path = LaunchConfiguration("bag_path").perform(context).strip()
    
    # 获取 use_sim_time 参数
    use_sim_time_arg = LaunchConfiguration("use_sim_time").perform(context).lower()
    
    # 智能推断
    if use_sim_time_arg == 'auto':
        # 如果是 auto：有 bag_path 就是 True，没有 bag_path 就是 False
        use_sim_time = bool(bag_path) 
    else:
        # 否则尊重用户的强制设定 (true 或 false)
        use_sim_time = (use_sim_time_arg == 'true')

    pkg_share = get_package_share_directory("indoor_location")
    location_config_path = os.path.join(pkg_share, "config", "argbot_mid360.yaml")
    eskf_cfg_path = os.path.join(pkg_share, "config", "eskf_cfg.yaml")
    
    resolved_map_file = _resolve_map_path(map_file, map_dir, map_name)

    return[
        Node(
            package='rviz2', 
            executable='rviz2', 
            name='rviz2',
            arguments=['-d', LaunchConfiguration('rviz_config_path')],
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        
        # 定位核心节点
        Node(
            package='indoor_location',
            executable='location_node',
            name='location_node',
            output='screen',
            respawn=True,
            parameters=[{
                "config_file": location_config_path,
                "eskf_cfg_file": eskf_cfg_path,
                "use_sim_time": use_sim_time,
                "map_filename": resolved_map_file
            }],
            arguments=['--ros-args', '--log-level', 'WARN'],
        )
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='auto', 
                              description="true/false/auto. If 'auto', becomes true when bag_path is set."),
        DeclareLaunchArgument('bag_path', default_value=''), 
        DeclareLaunchArgument('map_file', default_value=''),
        DeclareLaunchArgument('map_dir', default_value=os.environ.get('ROBOT_MAP_DIR', '')),
        DeclareLaunchArgument('map_name', default_value='map.pcd'),
        DeclareLaunchArgument('rviz_config_path', default_value=PathJoinSubstitution([FindPackageShare('indoor_location'), 'config/rviz/livox_rviz_config.rviz'])),
        
        # 统一启动节点
        OpaqueFunction(function=_launch_setup),

        # 播放 Rosbag
        ExecuteProcess(
            condition=IfCondition(
                PythonExpression(["'", LaunchConfiguration('bag_path'), "' != ''"])
            ),
            cmd=['ros2', 'bag', 'play', LaunchConfiguration('bag_path'), '--clock', '--rate', '1.0'],
            output='screen'
        )
    ])
