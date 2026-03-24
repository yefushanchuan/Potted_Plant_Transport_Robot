from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('robot_ctrl')

    # === 1. 默认行为树文件路径 ===
    default_bt_xml = PathJoinSubstitution([
        pkg_share, 'config', 'behavior_trees', 'main_tree', 'bdh_tree.xml'
    ])

    # === 2. 创建 LaunchDescription 容器 ===
    ld = LaunchDescription()

    # === 3. 声明参数 ===
    param_specs = [
        # 文件路径类参数
        ('bt_xml_file', default_bt_xml, 'Behavior Tree XML 文件路径'),
        ('error_log_file', 'src/robot_ctrl/logs/error_log.txt', 'LogErrorCondition 错误日志输出路径'),
        # 运行状态参数
        ('task_locked', 'false', '任务锁状态(true 表示当前任务执行中）'),
        ('target_pot_num', '0', '目标盆栽编号'),
        ('error_message', 'IDLE', '初始错误信息'),
        ('robot_state', 'IDLE', '机器人初始状态'),
        ('bt_loop_duration', '1000', 'BT Tick 周期（毫秒）'),
        ('server_timeout', '1000', '服务调用超时（毫秒）'),
        ('wait_for_service_timeout', '500', '等待服务超时（毫秒）'),
        ('patrol_min_interval_sec', '0', '定时巡检最小间隔（秒）'),
        ('patrol_trigger_hour', '-1', '定时巡检触发小时[0-23]，-1 表示忽略小时'),
        ('patrol_trigger_minute', '0', '定时巡检触发分钟[0-59]'),
        ('patrol_window_seconds', '5', '定时巡检触发窗口（秒）'),
        ('taskfile_retry_queue_file', 'src/robot_ctrl/logs/taskfile_retry_queue.txt', '任务文件上传失败重试队列持久化文件'),
    ]

    # 将参数声明添加到 LaunchDescription
    for name, default, desc in param_specs:
        ld.add_action(DeclareLaunchArgument(name, default_value=default, description=desc))

    # === 4. 组装参数表（Node 运行时使用） ===
    params = {name: LaunchConfiguration(name) for name, _, _ in param_specs}

    # === 5. 启动 BT 主节点 ===
    bt_node = Node(
        package='robot_ctrl',
        executable='bt_main',
        name='bt_runner',
        output='screen',
        parameters=[params],
    )

    ld.add_action(bt_node)
    return ld
