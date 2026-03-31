import launch
import launch_ros.actions
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    rviz_cfg = PathJoinSubstitution(
        [FindPackageShare("ba_optimize"), "rviz", "BA.rviz"]
    )
    ba_config_path = PathJoinSubstitution(
        [FindPackageShare("ba_optimize"), "config", "ba_qr100_livox_mid360.yaml"]
    )

    lio_config_path = PathJoinSubstitution(
        [FindPackageShare("fastlio2"), "config", "qr100_livox_mid360.yaml"]    # qr100参数
    )
    
    # bag_file_path = "/home/wz/ros2_test_ws/src/fastlio2_ros2/bag/LIVOX_Points/qr100/rosbag2_2025_11_11-14_55_07/rosbag2_2025_11_11-14_55_07_0.db3"  # qr100办公室较长数据集，玻璃门场景、厕所、展厅长时间放置
    bag_file_path = "/home/wz/ros2_test_ws/src/fastlio2_ros2/bag/LIVOX_Points/qr100/rosbag2_2025_12_04-10_37_22/rosbag2_2025_12_04-10_37_22_0.db3"  # qr100完整办公室较长数据集，覆盖所有房间场景

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
                parameters=[{
                    "config_path": ba_config_path.perform(launch.LaunchContext()),
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
