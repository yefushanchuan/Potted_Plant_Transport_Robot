#!/usr/bin/env python3
"""
双 IMU 外参标定脚本
===================
利用外置 IMU (/imu) 和 Livox Mid-360 内置 IMU (/livox/imu) 的角速度数据，
通过 SVD (Kabsch) 算法估计两者之间的旋转外参。

原理：
  两个 IMU 装在同一刚体上，体坐标系角速度相同：
    ω_body = R_ext · ω_ext = R_lidar · ω_lidar
  因此 ω_lidar = (R_lidar^T · R_ext) · ω_ext = R_ext_to_lidar · ω_ext
  收集 N 对角速度，用 SVD 求解 R_ext_to_lidar。

Mid-360 内置 IMU 坐标系 = LiDAR 坐标系 (R=I)，
所以 R_ext_to_lidar = R_ext_to_lidar_imu。

用法：
  ros2 run robot_base dual_imu_calib.py
  ros2 run robot_base dual_imu_calib.py --ros-args -p duration:=90
  ros2 run robot_base dual_imu_calib.py --ros-args -p min_omega:=0.3
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu
import message_filters


def rotation_matrix_to_euler_deg(R):
    """旋转矩阵 -> 欧拉角 (roll, pitch, yaw)，单位：度，ZYX 次序。"""
    sy = math.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    singular = sy < 1e-6
    if not singular:
        roll = math.atan2(R[2, 1], R[2, 2])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:
        roll = math.atan2(-R[1, 2], R[1, 1])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = 0.0
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def skew_symmetric(v):
    """3D 向量 -> 反对称矩阵。"""
    return np.array([
        [0, -v[2], v[1]],
        [v[2], 0, -v[0]],
        [-v[1], v[0], 0]
    ])


def estimate_rotation_svd(A, B):
    """
    Kabsch 算法：求 R 使得 B ≈ R · A。
    A, B: (3, N) 矩阵，每列是一个角速度样本。
    返回: R (3x3), 残差 (标量)
    """
    assert A.shape == B.shape and A.shape[0] == 3
    N = A.shape[1]
    if N < 3:
        raise ValueError(f"样本数不足: {N} < 3")

    H = B @ A.T
    U, S, Vt = np.linalg.svd(H)

    d = np.linalg.det(U @ Vt)
    diag = np.diag([1.0, 1.0, np.sign(d)])
    R = U @ diag @ Vt

    B_pred = R @ A
    residuals = np.linalg.norm(B - B_pred, axis=0)
    mean_residual = float(np.mean(residuals))

    return R, mean_residual


def estimate_rotation_from_gravity(g_ext, g_lidar):
    """
    用重力方向估计 roll/pitch 旋转（不含 yaw）。
    g_ext, g_lidar: 静止时的平均加速度 (3,)。
    返回: R_gravity (3x3)
    """
    g_ext_n = g_ext / np.linalg.norm(g_ext)
    g_lidar_n = g_lidar / np.linalg.norm(g_lidar)

    v = np.cross(g_ext_n, g_lidar_n)
    s = np.linalg.norm(v)
    c = np.dot(g_ext_n, g_lidar_n)

    if s < 1e-10:
        return np.eye(3)

    vx = skew_symmetric(v)
    R = np.eye(3) + vx + vx @ vx * (1 - c) / (s * s)
    return R


class DualIMUCalibrator(Node):
    def __init__(self):
        super().__init__('dual_imu_calibrator')

        # ---- 参数 ----
        self.declare_parameter('duration', 60.0)
        self.declare_parameter('min_omega', 0.1)
        self.declare_parameter('static_threshold', 0.05)
        self.declare_parameter('ext_imu_topic', '/imu')
        self.declare_parameter('lidar_imu_topic', '/livox/imu')

        self.duration = self.get_parameter('duration').value
        self.min_omega = self.get_parameter('min_omega').value
        self.static_threshold = self.get_parameter('static_threshold').value
        ext_topic = self.get_parameter('ext_imu_topic').value
        lidar_topic = self.get_parameter('lidar_imu_topic').value

        # ---- 状态标志 ----
        self.finished = False

        # ---- QoS ----
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ---- 订阅 & 同步 ----
        self.sub_ext = message_filters.Subscriber(
            self, Imu, ext_topic, qos_profile=qos)
        self.sub_lidar = message_filters.Subscriber(
            self, Imu, lidar_topic, qos_profile=qos)

        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.sub_ext, self.sub_lidar],
            queue_size=200,
            slop=0.05
        )
        self.sync.registerCallback(self.imu_callback)

        # ---- 数据存储 ----
        # 运动帧（角速度足够大）用于 SVD
        self.omega_ext_list = []
        self.omega_lidar_list = []
        # 所有帧的加速度 + 对应角速度范数，用于重力段提取
        self.all_accel_ext = []
        self.all_accel_lidar = []
        self.all_omega_norm = []

        # ---- 统计 ----
        self.total_pairs = 0
        self.valid_pairs = 0
        self.start_time = None
        self.last_print_time = 0.0

        # ---- 实时角速度窗口（状态显示用）----
        self.recent_omega_ext = []
        self.recent_omega_lidar = []

        # ---- 定时器 ----
        self.timer = self.create_timer(1.0, self.timer_callback)

        self.get_logger().info('=' * 60)
        self.get_logger().info('双 IMU 外参标定器已启动')
        self.get_logger().info(f'  外置 IMU  : {ext_topic}')
        self.get_logger().info(f'  雷达 IMU  : {lidar_topic}')
        self.get_logger().info(f'  采集时长  : {self.duration:.0f} s')
        self.get_logger().info(f'  角速度阈值: {self.min_omega} rad/s')
        self.get_logger().info(f'  同步容差  : 50 ms')
        self.get_logger().info('=' * 60)
        self.get_logger().info('>>> 请遥控机器人做旋转运动（原地正转/反转） <<<')
        self.get_logger().info('>>> 角速度越大，标定精度越高              <<<')

    # -----------------------------------------------------------------
    # 回调
    # -----------------------------------------------------------------
    def imu_callback(self, msg_ext, msg_lidar):
        if self.finished:
            return

        now = self.get_clock().now().nanoseconds / 1e9
        if self.start_time is None:
            self.start_time = now
        elapsed = now - self.start_time
        self.total_pairs += 1

        # 角速度
        w_ext = np.array([
            msg_ext.angular_velocity.x,
            msg_ext.angular_velocity.y,
            msg_ext.angular_velocity.z
        ])
        w_lidar = np.array([
            msg_lidar.angular_velocity.x,
            msg_lidar.angular_velocity.y,
            msg_lidar.angular_velocity.z
        ])

        # 线性加速度
        a_ext = np.array([
            msg_ext.linear_acceleration.x,
            msg_ext.linear_acceleration.y,
            msg_ext.linear_acceleration.z
        ])
        a_lidar = np.array([
            msg_lidar.linear_acceleration.x,
            msg_lidar.linear_acceleration.y,
            msg_lidar.linear_acceleration.z
        ])

        # NaN/Inf 检查
        if not (np.all(np.isfinite(w_ext)) and np.all(np.isfinite(w_lidar))):
            self.get_logger().warn('NaN/Inf 数据，跳过', throttle_duration_sec=5.0)
            return

        omega_norm_ext = float(np.linalg.norm(w_ext))
        omega_norm_lidar = float(np.linalg.norm(w_lidar))

        # 实时窗口（最近 200 帧）
        self.recent_omega_ext.append(omega_norm_ext)
        self.recent_omega_lidar.append(omega_norm_lidar)
        if len(self.recent_omega_ext) > 200:
            self.recent_omega_ext.pop(0)
            self.recent_omega_lidar.pop(0)

        # 存储所有帧的加速度 + 角速度范数（重力段提取用）
        self.all_accel_ext.append(a_ext)
        self.all_accel_lidar.append(a_lidar)
        self.all_omega_norm.append(omega_norm_ext)

        # 运动帧：角速度足够大
        if omega_norm_ext > self.min_omega and omega_norm_lidar > self.min_omega:
            self.omega_ext_list.append(w_ext)
            self.omega_lidar_list.append(w_lidar)
            self.valid_pairs += 1

        # 超时结束
        if elapsed >= self.duration:
            self.finish_calibration()

    def timer_callback(self):
        if self.finished or self.start_time is None:
            return

        now = self.get_clock().now().nanoseconds / 1e9
        elapsed = now - self.start_time

        # 每 5 秒打印
        if now - self.last_print_time < 5.0:
            return
        self.last_print_time = now

        if self.recent_omega_ext:
            avg_ext = float(np.mean(self.recent_omega_ext[-50:]))
            avg_lid = float(np.mean(self.recent_omega_lidar[-50:]))
        else:
            avg_ext = 0.0
            avg_lid = 0.0

        status = '运动中' if avg_ext >= self.static_threshold else '静止 - 请旋转!'

        self.get_logger().info(
            f'[{elapsed:.0f}s] '
            f'样本: {self.valid_pairs} | '
            f'w_ext: {avg_ext:.3f} | '
            f'w_lidar: {avg_lid:.3f} | '
            f'{status}'
        )

    # -----------------------------------------------------------------
    # 标定计算
    # -----------------------------------------------------------------
    def finish_calibration(self):
        if self.finished:
            return
        self.finished = True
        self.timer.cancel()

        self.get_logger().info('')
        self.get_logger().info('=' * 60)
        self.get_logger().info('采集结束，开始计算...')
        self.get_logger().info(f'  总同步帧数 : {self.total_pairs}')
        self.get_logger().info(f'  有效运动帧 : {self.valid_pairs}')

        if self.valid_pairs < 10:
            self.get_logger().error(
                '有效样本不足 (<10)，无法标定！请重新运行并多做旋转运动。')
            rclpy.shutdown()
            return

        # ---- 1. SVD 旋转标定 ----
        A = np.column_stack(self.omega_ext_list)
        B = np.column_stack(self.omega_lidar_list)

        try:
            R_ext_to_lidar, residual = estimate_rotation_svd(A, B)
        except Exception as e:
            self.get_logger().error(f'SVD 计算失败: {e}')
            rclpy.shutdown()
            return

        # ---- 2. 奇异值 / 条件数 ----
        H = B @ A.T
        _, S, _ = np.linalg.svd(H)
        cond_number = float(S[0] / (S[2] + 1e-10))

        # ---- 3. 重力验证 ----
        R_gravity = None
        static_count = 0
        if len(self.all_omega_norm) > 0:
            static_mask = np.array(self.all_omega_norm) < self.static_threshold
            static_count = int(np.sum(static_mask))
            if static_count > 50:
                accel_ext_arr = np.array(self.all_accel_ext)
                accel_lidar_arr = np.array(self.all_accel_lidar)
                g_ext_mean = np.mean(accel_ext_arr[static_mask], axis=0)
                g_lidar_mean = np.mean(accel_lidar_arr[static_mask], axis=0)
                R_gravity = estimate_rotation_from_gravity(g_ext_mean, g_lidar_mean)

        # ---- 4. 欧拉角 ----
        roll_svd, pitch_svd, yaw_svd = rotation_matrix_to_euler_deg(R_ext_to_lidar)

        # ---- 5. 输出 ----
        log = self.get_logger().info

        log('')
        log('=' * 60)
        log('       双 IMU 外参标定结果')
        log('=' * 60)

        log('')
        log('R_ext_to_lidar (外置IMU -> LiDAR 旋转矩阵):')
        for row in R_ext_to_lidar:
            log(f'  [{row[0]:+10.6f}, {row[1]:+10.6f}, {row[2]:+10.6f}]')

        log('')
        log('欧拉角 (ZYX, roll/pitch/yaw):')
        log(f'  roll  = {roll_svd:+8.2f} deg')
        log(f'  pitch = {pitch_svd:+8.2f} deg')
        log(f'  yaw   = {yaw_svd:+8.2f} deg')

        if R_gravity is not None:
            roll_g, pitch_g, yaw_g = rotation_matrix_to_euler_deg(R_gravity)
            log('')
            log(f'重力验证 (静止帧 {static_count}, 仅 roll/pitch 有效):')
            log(f'  roll  = {roll_g:+8.2f} deg')
            log(f'  pitch = {pitch_g:+8.2f} deg')
            log(f'  yaw   = {yaw_g:+8.2f} deg (重力无法估计 yaw)')
            log(f'  Droll  = {abs(roll_svd - roll_g):.2f} deg')
            log(f'  Dpitch = {abs(pitch_svd - pitch_g):.2f} deg')
        else:
            log('')
            log(f'重力验证: 静止帧不足 ({static_count}), 跳过')

        log('')
        log('URDF 理论值 (LiDAR pitch 安装角 45 deg):')
        log(f'  理论 pitch = +45.00 deg')
        log(f'  标定 pitch = {pitch_svd:+8.2f} deg')
        log(f'  偏差 Dpitch = {pitch_svd - 45.0:+.2f} deg')

        t_ext_to_lidar = np.array([0.577, -0.241, 0.090])
        log('')
        log('平移 (URDF 理论值, 双 IMU 无法精确估计):')
        log(f'  t_ext_to_lidar = [{t_ext_to_lidar[0]:.3f}, '
            f'{t_ext_to_lidar[1]:.3f}, {t_ext_to_lidar[2]:.3f}] m')

        # FAST-LIO2 兼容格式
        R_il = R_ext_to_lidar.T
        r_il_flat = R_il.flatten()
        log('')
        log('--- FAST-LIO2 兼容格式 (可直接粘贴) ---')
        log(f'r_il: [{r_il_flat[0]:.6f}, {r_il_flat[1]:.6f}, {r_il_flat[2]:.6f}, '
            f'{r_il_flat[3]:.6f}, {r_il_flat[4]:.6f}, {r_il_flat[5]:.6f}, '
            f'{r_il_flat[6]:.6f}, {r_il_flat[7]:.6f}, {r_il_flat[8]:.6f}]')
        log(f't_il: [{t_ext_to_lidar[0]:.6f}, '
            f'{t_ext_to_lidar[1]:.6f}, {t_ext_to_lidar[2]:.6f}]')

        # 质量评估
        log('')
        log(f'SVD 平均残差 : {residual:.6f} rad/s')
        if residual < 0.05:
            log('  残差评估: 优秀 (< 0.05)')
        elif residual < 0.1:
            log('  残差评估: 良好 (< 0.1)')
        elif residual < 0.2:
            log('  残差评估: 一般 (< 0.2)，建议重新标定')
        else:
            self.get_logger().warn('  残差评估: 较差 (>= 0.2)，请重新标定')

        log(f'奇异值       : [{S[0]:.2f}, {S[1]:.2f}, {S[2]:.2f}]')
        log(f'条件数       : {cond_number:.1f}')
        if cond_number < 5:
            log('  可观性: 优秀 (< 5)')
        elif cond_number < 10:
            log('  可观性: 良好 (< 10)')
        elif cond_number < 30:
            log('  可观性: 一般 (< 30)，建议增加运动多样性')
        else:
            self.get_logger().warn('  可观性: 差 (>= 30)，运动激励不足')

        log('')
        log('=' * 60)
        log('标定完成。')
        log('=' * 60)

        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = DualIMUCalibrator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('用户中断 (Ctrl+C)')
        if not node.finished and node.valid_pairs >= 10:
            node.finish_calibration()
        elif not rclpy.ok():
            pass
        else:
            rclpy.shutdown()


if __name__ == '__main__':
    main()
