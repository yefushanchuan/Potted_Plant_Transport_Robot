from launch import LaunchDescription
from launch_ros.actions import Node

from launch.substitutions import Command, PathJoinSubstitution, FindExecutable
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    robot_description = ParameterValue(
        Command([
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([
                FindPackageShare("robot_base"),
                "urdf",
                "agrobot.xacro",
            ]),
        ]),
        value_type=str
    )

    # RViz配置文件路径
    rviz_config = PathJoinSubstitution([
        FindPackageShare("robot_base"),  # 你的包名
        "rviz",                          # 配置文件存放目录
        "bringup.rviz",                   # 配置文件名
    ])

    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
        ),

        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", rviz_config],
        ),
    ])
