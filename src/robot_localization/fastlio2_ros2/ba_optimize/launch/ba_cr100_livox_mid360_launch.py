import launch
import launch_ros.actions
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    rviz_cfg = PathJoinSubstitution(
        [FindPackageShare("ba_optimize"), "rviz", "BA.rviz"]
    )
    ba_config_path = PathJoinSubstitution(
        [FindPackageShare("ba_optimize"), "config", "ba_cr100_livox_mid360.yaml"] 
    )

    lio_config_path = PathJoinSubstitution(
        [FindPackageShare("fastlio2"), "config", "cr100_livox_mid360.yaml"]     # cr100参数
    )

    #bag_file_path = "/home/wz/ros2_test_ws/src/fastlio2_ros2/bag/LIVOX_Points/cr100/rosbag2_2025_10_15-11_16_44/rosbag2_2025_10_15-11_16_44_0.db3"  # ea200办公室数据集
    bag_file_path = "/home/wz/ros2_test_ws/src/fastlio2_ros2/bag/LIVOX_Points/cr100/rosbag2_2025_10_20-16_25_09/rosbag2_2025_10_20-16_25_09_0.db3"  # ea100办公室数据集
    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                package="fastlio2",
                namespace="fastlio2",
                executable="fastlio2_node",
                name="fastlio2_node",
                output="screen",
                parameters=[{
                    "config_path": lio_config_path.perform(launch.LaunchContext()),
                    "use_sim_time": True}]
            ),
            launch_ros.actions.Node(
                package="ba_optimize",
                namespace="ba_optimize",
                executable="ba_node",
                name="ba_node",
                output="screen",
                parameters=[{"config_path": ba_config_path.perform(launch.LaunchContext()),
                            "use_sim_time": True}],
                arguments=['--ros-args', '--log-level', 'WARN'],#终端仅仅输出WARN调试日志
            ),
            launch_ros.actions.Node(
                package="rviz2",
                namespace="ba_optimize",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_cfg.perform(launch.LaunchContext())],

            ),
            # 播放 rosbag（带仿真时间）
            launch.actions.ExecuteProcess(
                cmd=["ros2", "bag", "play", bag_file_path, "--clock", "--rate", "30.0"],
                output="screen"
            ),
        ]
    )
