import os
import time

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def _generate_config_with_map(src_path, map_path):
    """根据传入的地图路径生成配置文件，如果map_path为空则使用原配置"""
    if not map_path:
        return src_path
    ts = time.strftime("%Y%m%d_%H%M%S")
    dst_path = os.path.join("/tmp", f"argbot_mid360_{ts}.yaml")
    with open(src_path, "r", encoding="utf-8") as f:
        lines = f.readlines()
    replaced = False
    for idx, line in enumerate(lines):
        stripped = line.lstrip()
        if stripped.startswith("map_filename:"):
            prefix = line[: len(line) - len(stripped)]
            lines[idx] = f'{prefix}map_filename: "{map_path}"\n'
            replaced = True
            break
    if not replaced:
        lines.append(f'\nmap_filename: "{map_path}"\n')
    with open(dst_path, "w", encoding="utf-8") as f:
        f.writelines(lines)
    return dst_path


def _resolve_map_path(raw_map_file, raw_map_dir, raw_map_name):
    """解析地图路径：兼容旧 map_file，同时支持 map_dir + map_name。"""
    map_file = (raw_map_file or "").strip()
    map_dir = (raw_map_dir or "").strip()
    map_name = (raw_map_name or "").strip()

    # 兼容旧参数：直接传 map_file（可绝对/相对）
    if map_file:
        expanded_file = os.path.expandvars(os.path.expanduser(map_file))
        if os.path.isabs(expanded_file):
            return expanded_file
        if map_dir:
            expanded_dir = os.path.expandvars(os.path.expanduser(map_dir))
            return os.path.abspath(os.path.join(expanded_dir, expanded_file))
        return os.path.abspath(expanded_file)

    # 新参数：map_dir + map_name
    if map_name:
        if not map_dir:
            map_dir = os.environ.get("ROBOT_MAP_DIR", "")
        if not map_dir:
            map_dir = os.path.join(get_package_share_directory("robot_bringup"), "pcd")
        expanded_dir = os.path.expandvars(os.path.expanduser(map_dir))
        return os.path.abspath(os.path.join(expanded_dir, map_name))

    # 都为空时，沿用配置文件中的 map_filename
    return ""


def _launch_setup(context):
    """OpaqueFunction 回调：解析 LaunchConfiguration 后创建节点"""
    map_file = LaunchConfiguration("map_file").perform(context)
    map_dir = LaunchConfiguration("map_dir").perform(context)
    map_name = LaunchConfiguration("map_name").perform(context)
    
    base_config_path = os.path.join(
        get_package_share_directory("robot_bringup"), "config", "3d_localization", "agrobot_mid360.yaml"
    )
    eskf_cfg_path = os.path.join(
        get_package_share_directory("robot_bringup"), "config", "3d_localization", "eskf_cfg.yaml"
    )
    
    resolved_map_file = _resolve_map_path(map_file, map_dir, map_name)

    # 显式加载：都为空则使用原配置，否则生成带地图路径的配置
    location_config_path = _generate_config_with_map(base_config_path, resolved_map_file)

    return [
        # LIVOX_MID360 Location Node
        Node(
            package='indoor_location',
            executable='location_node',
            name='location_node',
            output='screen',
            respawn=True,
            parameters=[{
                "config_file": location_config_path,
                "eskf_cfg_file": eskf_cfg_path,
                'use_sim_time': False
            }],
            arguments=['--ros-args', '--log-level', 'WARN'],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        # === 地图文件路径 (显式加载，默认为空) ===
        DeclareLaunchArgument(
            'map_file',
            default_value='/home/qiaowen/rpp_ws/src/robot_bringup/pcd/map.pcd',
            description='兼容参数：PCD地图路径（支持绝对/相对）；为空时可由 map_dir+map_name 组合'
        ),
        DeclareLaunchArgument(
            'map_dir',
            default_value=os.environ.get('ROBOT_MAP_DIR', ''),
            description='地图目录（可选）；未设置且 map_name 非空时回退到 ROBOT_MAP_DIR 或包内 map 目录'
        ),
        DeclareLaunchArgument(
            'map_name',
            default_value='map.pcd',
            description='地图文件名（可选）；与 map_dir 组合成最终路径'
        ),
        DeclareLaunchArgument(
            'rviz_config_path', 
            default_value=[FindPackageShare('robot_bringup'), '/rviz/livox_rviz_config.rviz'], 
            description='RViz config path'
        ),
        
        # RViz 已禁用 - 如需启用请取消注释
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     arguments=['-d', LaunchConfiguration('rviz_config_path')],
        #     output='screen',
        #     parameters=[{'use_sim_time': False}],
        # ),
        
        # 使用 OpaqueFunction 延迟解析 map_file 参数
        OpaqueFunction(function=_launch_setup),

        # 实车模式：不播放rosbag
    ])
