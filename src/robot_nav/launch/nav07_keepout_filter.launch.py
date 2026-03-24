import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import RewrittenYaml


def generate_launch_description():

    pkg_robot_nav = get_package_share_directory('robot_nav')

    # ============================================
    # Launch Arguments
    # ============================================
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='', description='Top-level namespace')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false', description='ROS simulation clock')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution(
            [pkg_robot_nav, 'config', 'navigation', 'Nav_DWB_simple.yaml']
        ),
        description='Parameters file'
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true', description='Autostart lifecycle nodes')

    # ============================================
    # Rewritten YAML (支持 namespace)
    # ============================================
    param_substitutions = {
        'use_sim_time': use_sim_time
    }

    configured_params = RewrittenYaml(
        source_file=params_file,
        param_rewrites=param_substitutions,
        root_key=namespace,       # 自动支持多 namespace
        convert_types=True
    )

    # ============================================
    # Lifecycle Nodes
    # ============================================
    lifecycle_nodes = ['costmap_filter_info_server']

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_costmap_filters',
        namespace=namespace,
        output='screen',
        emulate_tty=True,
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': lifecycle_nodes
        }]
    )

    # Publish /costmap_filter_info
    costmap_filter_info_server = Node(
        package='nav2_map_server',
        executable='costmap_filter_info_server',
        name='costmap_filter_info_server',
        namespace=namespace,
        output='screen',
        parameters=[configured_params]
    )

    # ============================================
    # Keepout Mask Publisher
    # ============================================
    keepout_mask_publisher = Node(
        package='robot_nav',
        executable='keepout_mask_publisher',
        name='keepout_mask_publisher',
        namespace=namespace,
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'map_topic': 'map',   # cartographer map topic
            'mask_topic': '/keepout_filter_mask',
            'enable_keepout': True,
            'keepout_zones_file': PathJoinSubstitution([
                pkg_robot_nav, 'config/keepout', 'keepout_mask.json'
            ]),
        }]
    )

    # ============================================
    # Launch Description
    # ============================================
    ld = LaunchDescription()

    # Declare arguments
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)

    # Add nodes
    ld.add_action(lifecycle_manager)
    ld.add_action(costmap_filter_info_server)
    ld.add_action(keepout_mask_publisher)

    return ld
