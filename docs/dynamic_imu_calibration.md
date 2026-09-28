# 不依赖静止段的双 IMU 动态标定

适用于外部 IMU 与 MID360 内部 IMU 刚性固定、可抱起整个底盘或整个刚性传感器组件做三轴运动的情况。输入是两路 IMU，不需要点云、里程计、静止重力方向或静止零偏。不能分别拿起两个传感器运动。

脚本 `src/robot_base/scripts/dynamic_imu_calibration.py` 可以直接离线运行，也注册到了 robot_base 安装入口。旧的平面底盘标定脚本保持独立。

## 求解内容

动态陀螺先初始化相对旋转 R、时间差 dt 和陀螺差偏置；动态加速度差初始化 xyz 和加速度差偏置，然后联合优化全部 16 个参数：R 的 3 个自由度、xyz、dt、两颗陀螺各自的 3 轴零偏、3 轴加速度差偏置。仅初始化时参考陀螺零偏置零，最终并不固定它。没有静止均值约束或零偏正则项。

原始加速度保留重力，单位统一到 m/s²。时间和坐标正确对齐后，共同的重力项在差分中抵消，因此不需要单独估计 g。相对加速度差偏置不等于任一传感器的物理加速度零偏，**不要把它直接填进 LIO 的单颗 IMU 加速度零偏**。

在活动筛选时，使用角速度局部变化量，不使用“静止时均值为零”的假设。低活动区不参与求解；短暂停顿允许存在，但无需专门录制静止段。

## 录制

1. 保持两颗 IMU 刚性安装、线缆固定，传感器工作温度稳定。录原始两路 IMU，在在线时间修正节点之前取数据；录制期间不要重启传感器或切换时间源。
2. 抱起底盘，分别绕车体前后、左右、上下三个方向来回转动，再加入混合转动。建议每轴约 ±20–45°，不断改变转速和转向；不要只绕竖直轴转，也不要只做长时间匀速旋转。动作平顺，避免碰撞、松动和数据饱和。
3. 起步建议每条录制 60–90 秒，三轴都覆盖多次，并包含不同快慢的动作。这是采集建议，最终看激励和验证检查，而不是只看时长。
4. 同次上电再录一条独立的 60–90 秒验证记录，改变动作次序和快慢，保持安装不变。第二条用来独立标定和冻结参数预测，不能复制第一条。

项目默认原始话题为 `/imu` 和 `/livox/imu`。如实际话题不同，同时修改录制命令、QoS 文件和标定参数。下面命令在仓库根目录执行，两次分别录制，动作结束后 Ctrl-C：

```bash
ros2 bag record -o imu_train \
  --qos-profile-overrides-path src/robot_base/config/dynamic_imu_recording_qos.yaml \
  /imu /livox/imu
```

```bash
ros2 bag record -o imu_check \
  --qos-profile-overrides-path src/robot_base/config/dynamic_imu_recording_qos.yaml \
  /imu /livox/imu
```

使用消息 **header.stamp**，不使用 rosbag 写入时间或 IMU orientation 字段。拒绝重复/倒退时间戳。只估计常量 dt；持续时钟漂移、在线动态改戳或重启造成的时间跳变不属于该模型。

## 安装与运行

Python 3.10+，离线运行无需 ROS 环境；读 bag 使用 rosbags。已在 NumPy 2.5.3、SciPy 1.18.1、rosbags 0.11.5 的研究环境验证。新环境：

```bash
python3 -m venv .venv_imu_calib
.venv_imu_calib/bin/python -m pip install -r src/robot_base/config/dynamic_imu_requirements.txt
```

仓库原有配置 `imu_chain_calibration.yaml` 对外部 IMU 使用 SI 单位，对原始 MID360 加速度使用 g→m/s² 换算。因此当前接线/驱动的示例命令是：

```bash
OPENBLAS_NUM_THREADS=1 .venv_imu_calib/bin/python \
  src/robot_base/scripts/dynamic_imu_calibration.py \
  --input imu_train --holdout imu_check \
  --imu0 /imu --imu1 /livox/imu \
  --accel-scale0 1 --accel-scale1 9.80665 \
  --output calibration/dynamic_imu_pair.json
```

如果 MID360 的上游节点已经转成 m/s²，把 `--accel-scale1` 改成 `1`；禁止重复换算。两个角速度默认均为 rad/s，若输入为 °/s，显式设置对应 `--gyro-scale0` 或 `--gyro-scale1` 为 `0.017453292519943295`。脚本检查比力模长以捕获明显单位错误，不会猜测并自动修正单位。

支持 ROS1 `.bag`、ROS2 bag 目录以及 `.npz`。NPZ 的两个 key 由 `--imu0/--imu1` 指定，各自为 N×7 数组，列顺序 `[timestamp_s, ax, ay, az, gx, gy, gz]`，无需预先同步或统一采样率。默认 dt 搜索 ±0.1 s；已知可能更大时可在录制后显式设置 `--dt-bound 0.3`，并重新检查结果。

输出文件必须不存在，避免覆盖上一轮结果。单条记录可以运行，但状态仅为 `fit_passed_needs_independent_validation`，应继续录第二条。`rejected` 时返回退出码 2；不要因为 JSON 中有数值就使用该结果。

## 输出方向与验收

`imu0` 是外部 IMU、`imu1` 是 MID360 内部 IMU 时：

- `R`：把 IMU1 的向量转到 IMU0；四元数顺序为 xyzw。
- `t_m`：IMU1 感测原点在 IMU0 坐标系中的位置，xyz 均估计，单位 m。
- `td_s`：同一事件 `timestamp0 = timestamp1 + td_s`。在 IMU0 时刻 `t`，应读取 IMU1 的 `t - td_s`。这不是自动推断的点云时间偏移。
- `gyro_bias0_rad_s`、`gyro_bias1_rad_s`：各自在本 IMU 坐标系中的陀螺零偏；只在记录检查通过后考虑使用。
- `accel_difference_bias_m_s2`：转到 IMU0 后的加速度常量差偏置；不是单颗加速度计独立内参。

有独立验证记录时，只有状态 `passed_recording_checks_not_absolute_accuracy_certified` 表示通过当前记录检查。检查包括：

- 足够的多轴动态激励，优化收敛，参数矩阵无明显退化，dt 不顶搜索边界。
- 动态陀螺/加速度差逐分量 RMSE 默认不超过 0.03 rad/s、0.25 m/s²。
- 各自陀螺零偏模长不超过 0.1 rad/s；跨记录各自零偏变化不超过 0.02 rad/s。
- 两次独立标定旋转差不超过 0.5°，三维平移差不超过 15 mm，dt 差不超过 3 ms。
- 两个方向的跨记录预测都通过；预测时 R、xyz、dt 和所有偏置均冻结。

这些是用于拒绝明显异常结果的工程门限，**不是 15 mm 或其他绝对精度保证**。需求更严时，应查看具体重复差、杆臂实测值和实际 LIO 效果；不要仅放宽门限让失败结果通过。比例因子、轴间非正交和时变偏置未完整标定。

自由估计零偏会放大弱激励或模型误差的影响。HRMC 实测就出现了外参看似稳定、各自陀螺零偏却跨记录不一致的情况，脚本会拒绝这类结果。若失败，先改善三轴变速激励、延长记录并检查时间/单位/刚性安装；仍不通过时需补充传感器内参约束，不能承诺无静止流程适合所有硬件。

## 组合到底盘轮轴与点云

抱起来做运动只能求两颗 IMU 的相对外参，不能识别车体前向或轮轴中点。需要提供一个已知锚点 `T_body_imu0`（外部 IMU 到车体/轮轴坐标系的 R 和 t），以及实际型号确认过的 `T_imu1_lidar`（点云坐标系到内部 IMU 的 R 和 t），才能得到整个空间链路。

可选 `--mounts measured_mounts.json`，字段结构如下；下面 null 必须替换为实测数值，R 是 3×3 矩阵，t 是长度 3 的米制向量：

```json
{
  "T_body_imu0": {"R": [[null,null,null],[null,null,null],[null,null,null]], "t_m": [null,null,null]},
  "T_imu1_lidar": {"R": [[null,null,null],[null,null,null],[null,null,null]], "t_m": [null,null,null]}
}
```

两项也可以只提供一项。脚本输出可组合的 `T_body_imu1`、`T_imu0_lidar`、`T_body_lidar`，统一遵守 `T_目标_源` 把源坐标映射到目标坐标的约定。不会覆盖 TF/LIO 配置，也不会把加速度差偏置写成某颗 IMU 的绝对零偏。测量锚点误差和厂家内部几何误差会保留在组合结果里。

## 已完成的验证

五条真实双 IMU 记录均从动态数据独立初始化并联合求解，未输入静止零偏或重力。mix-cal 两条通过当前验收：xyz 重复差约 0.49 mm、xy 约 0.38 mm、旋转约 0.056°，三维平移与作者机械参考相差约 3.2–3.3 mm。冻结全部参数的双向动态加速度差 RMSE 约 0.075–0.080 m/s²。重复性不等于参考精度。

HRMC 三条的 xyz 重复差约 0.95–1.68 mm，但各自陀螺零偏不稳定，全部记录对未通过验收；因此没有把五条数据都标成成功。对现有数据各次录制是否同次上电没有独立确认。

独立解析信号测试覆盖非零各自陀螺零偏、非单位旋转、三维杆臂、37 ms 时间差、不同采样率、无静止初值恢复、含噪独立验证、错误时间符号、纯 yaw 拒绝、同点 IMU 的零偏退化拒绝及测量锚点组合。运行：

```bash
OPENBLAS_NUM_THREADS=1 .venv_imu_calib/bin/python -m unittest discover \
  -s src/robot_base/test -p test_dynamic_imu_calibration.py -v
```

已验证真实 ROS1 mix-cal bag 的完整命令行训练/验证通过，以及原始 ROS2 HRMC bag 的异常解被拒绝并返回退出码 2。当前研究主机未安装 ROS，未进行 colcon 构建或现场 ros2 bag record 测试；离线 bag 读取使用 rosbags，已实际执行。
