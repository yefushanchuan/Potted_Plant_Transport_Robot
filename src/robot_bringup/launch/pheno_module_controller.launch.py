from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 支持通过 launch 参数覆盖配置文件路径
    config_file = LaunchConfiguration("config_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("robot_bringup"), "config", "pheno_module_controller", "pheno_module_controller.yaml"]
                ),
                description="Path to pheno_module_controller parameter file",
            ),
            # 启动 pheno_module_controller 节点
            Node(
                package="pheno_module_controller",
                executable="pheno_module_controller_node",
                name="pheno_module_controller",
                output="screen",
                parameters=[config_file],
            ),
        ]
    )
