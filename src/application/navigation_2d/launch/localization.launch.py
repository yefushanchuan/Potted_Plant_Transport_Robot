from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    location_config_path = os.path.join(get_package_share_directory("navigation_2d"), "config", "localication_qr100_mid360.yaml")
    eskf_cfg_path = os.path.join(get_package_share_directory("navigation_2d"), "config", "eskf_cfg.yaml")

    return LaunchDescription(
        [
            Node(
                package="indoor_location",
                executable="location_node",
                name="location_node",
                output="screen",
                respawn=True,
                parameters=[
                    {
                        "config_file": location_config_path,
                        "eskf_cfg_file": eskf_cfg_path,
                    }
                ],
                #arguments=["--ros-args", "--log-level", "INFO"],
                arguments=['--ros-args', '--log-level', 'WARN'],#终端仅仅输出WARN调试日志
            ),
            Node(
                package="nav2_map_server",
                executable="map_server",
                parameters=[
                    {"yaml_filename": "/home/rpp/rpp_data/map/map.yaml"},
                    {"topic_name": "map"},
                    {"frame_id": "map"},
                ],
                output="screen",
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_2d_map_server",
                output="screen",
                parameters=[{"autostart": True}, {"node_names": ["map_server"]}],
            ),
        ]
    )
