from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.descriptions import ParameterFile
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_robot_bringup = get_package_share_directory("robot_bringup")

    roompose_file = LaunchConfiguration("roompose_file")
    keepout_mask_file = LaunchConfiguration("keepout_mask_file")
    route_yaml_path = LaunchConfiguration("route_yaml_path")
    map_yaml_path = LaunchConfiguration("map_yaml_path")
    localization_pbstream_file_path = LaunchConfiguration("localization_pbstream_file_path")
    localization_pcd_file_path = LaunchConfiguration("localization_pcd_file_path")
    mqtt_params_file = LaunchConfiguration("mqtt_params_file")

    # 文件路径参数
    declare_roompose_arg = DeclareLaunchArgument(
        "roompose_file",
        default_value=PathJoinSubstitution([pkg_robot_bringup, "roompose", "roompose.json"]),
        description="房间位姿 JSON 文件路径",
    )
    declare_keepout_arg = DeclareLaunchArgument(
        "keepout_mask_file",
        default_value=PathJoinSubstitution(
            [pkg_robot_bringup, "keepout", "keepout_mask.json"]
        ),
        description="Keepout 区域配置",
    )
    declare_route_yaml_arg = DeclareLaunchArgument(
        "route_yaml_path",
        default_value=PathJoinSubstitution([pkg_robot_bringup, "route", "test_route.yaml"]),
        description="路网 YAML 文件路径",
    )
    declare_map_yaml_path_arg = DeclareLaunchArgument(
        "map_yaml_path",
        default_value=PathJoinSubstitution([pkg_robot_bringup, "map", "map.yaml"]),
        description="set_map 下发后保存地图 YAML 文件路径（PGM 自动同名）",
    )
    declare_localization_pbstream_path_arg = DeclareLaunchArgument(
        "localization_pbstream_file_path",
        default_value=PathJoinSubstitution([pkg_robot_bringup, "map", "map.pbstream"]),
        description="localization_file_upload 上传 pbstream 时使用的本地文件路径",
    )
    declare_localization_pcd_path_arg = DeclareLaunchArgument(
        "localization_pcd_file_path",
        default_value=PathJoinSubstitution([pkg_robot_bringup, "pcd", "map.pcd"]),
        description="localization_file_upload 上传 pcd 时使用的本地文件路径",
    )
    declare_mqtt_params_file_arg = DeclareLaunchArgument(
        "mqtt_params_file",
        default_value=PathJoinSubstitution(
            [pkg_robot_bringup, "config", "mqtt", "mqtt_bridge_params.yaml"]
        ),
        description="MQTT 节点参数 YAML 文件路径（非文件路径类参数）",
    )

    mqtt_node = Node(
        package="robot_mqtt_bridge",
        executable="mqtt_node",
        name="mqtt_node",
        output="screen",
        parameters=[
            ParameterFile(mqtt_params_file, allow_substs=True),
            {"roompose_file_path": roompose_file},
            {"keepout_mask_file": keepout_mask_file},
            {"route_yaml_path": route_yaml_path},
            {"map_yaml_path": map_yaml_path},
            {"localization_pbstream_file_path": localization_pbstream_file_path},
            {"localization_pcd_file_path": localization_pcd_file_path},
        ]
    )

    return LaunchDescription([
        declare_roompose_arg,
        declare_keepout_arg,
        declare_route_yaml_arg,
        declare_map_yaml_path_arg,
        declare_localization_pbstream_path_arg,
        declare_localization_pcd_path_arg,
        declare_mqtt_params_file_arg,
        mqtt_node,
    ])
