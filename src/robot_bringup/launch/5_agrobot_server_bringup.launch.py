from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_robot_bringup = get_package_share_directory('robot_bringup')
    roompose_file = LaunchConfiguration('roompose_file')
    launch_mode = LaunchConfiguration("default_mode")
    default_launch_package = LaunchConfiguration("default_launch_package")
    common_launches = LaunchConfiguration("common_launches")
    navigation_launches = LaunchConfiguration("navigation_launches")
    mapping_launches = LaunchConfiguration("mapping_launches")

    declare_roompose_arg = DeclareLaunchArgument(
        'roompose_file',
        default_value=PathJoinSubstitution([pkg_robot_bringup, 'roompose', 'roompose.json']),
        description='房间位姿 JSON 文件路径'
    )
    launch_mode_arg = DeclareLaunchArgument(
        "default_mode",
        default_value="navigation",
        description="launch_manager default mode: navigation or mapping",
    )
    default_launch_package_arg = DeclareLaunchArgument(
        "default_launch_package",
        default_value="robot_bringup",
        description="default package for launch entries without package prefix",
    )
    common_launches_arg = DeclareLaunchArgument(
        "common_launches",
        default_value="1_agrobot_base_bringup.launch.py",
        description="comma separated launch list, format launch.py or pkg:launch.py",
    )
    navigation_launches_arg = DeclareLaunchArgument(
        "navigation_launches",
        default_value=(
            "agrobot_localization_bringup.launch.py,"
            "3_agrobot_nav_by_route_bringup.launch.py,"
            "pheno_module_controller.launch.py"
        ),
        description="comma separated launch list for navigation mode",
    )
    mapping_launches_arg = DeclareLaunchArgument(
        "mapping_launches",
        default_value="2_agrobot_mapping_bringup.launch.py",
        description="comma separated launch list for mapping mode",
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
    launch_manager_node = Node(
        package="robot_ctrl",
        executable="launch_manager.py",
        name="launch_manager",
        output="screen",
        parameters=[{
            "default_mode": launch_mode,
            "default_launch_package": default_launch_package,
            "common_launches": common_launches,
            "navigation_launches": navigation_launches,
            "mapping_launches": mapping_launches,
        }],
    )

    return LaunchDescription([
        declare_roompose_arg,
        launch_mode_arg,
        default_launch_package_arg,
        common_launches_arg,
        navigation_launches_arg,
        mapping_launches_arg,
        recorder_node,
        loader_node,
        launch_manager_node
    ])
