from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    # 声明启动参数
    use_filter = LaunchConfiguration("use_filter")
    source_topic = LaunchConfiguration("source_topic")
    pub_topic = LaunchConfiguration("pub_topic")
    outlier_threshold = LaunchConfiguration("outlier_threshold")

    # 定义组件节点
    lidar_filter_component = ComposableNode(
        package="robot_base",
        plugin="robot_base_utils::LidarFilter2D", # 对应 CMakeLists 里的 PLUGIN 名称
        name="lidar_filter",
        parameters=[
            {"source_topic": source_topic},
            {"pub_topic": pub_topic},
            {"outlier_threshold": outlier_threshold},
        ],
        # 开启进程内通信以实现零拷贝
        extra_arguments=[{'use_intra_process_comms': True}]
    )

    # 定义组件容器 (Container)
    # 如果你的雷达驱动也是 Component，把它们放在同一个 Container 里，零拷贝才会生效！
    container = ComposableNodeContainer(
        name='lidar_filter_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container', # 或者 component_container_mt (多线程)
        composable_node_descriptions=[lidar_filter_component],
        output='screen',
        condition=IfCondition(use_filter)
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_filter", default_value="true"),
            DeclareLaunchArgument("source_topic", default_value="/scan"),
            DeclareLaunchArgument("pub_topic", default_value="/scan_filtered"),
            DeclareLaunchArgument("outlier_threshold", default_value="0.1"),
            container
        ]
    )
