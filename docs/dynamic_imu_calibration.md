# 静止约束与三轴动态联合的双 IMU 标定

适用于外部 IMU 与 MID360 内部 IMU 刚性固定、可抱起整个底盘或整个刚性传感器组件做三轴运动的情况。输入是两路原始 IMU。默认流程为：**同次录制的静止零偏约束 → 三轴动态初始化 → 联合精修 → 独立记录验证**。

脚本 `src/robot_base/scripts/dynamic_imu_calibration.py` 可离线运行，也注册到 robot_base 安装入口。文件名保留兼容，默认方法已由无约束纯动态改为 `joint_static_soft`，JSON schema 升为 2。旧的平面底盘标定脚本独立保留。

## 算法流程

1. **检查输入。** 使用两路消息的 header 时间，拒绝重复/倒退时间戳、明显错误单位和过低采样率；保留加速度中的重力。时间不预先强行对齐，dt 是待估参数。
2. **建立静止陀螺零偏约束。** 在同一份记录里寻找两颗 IMU 都满足条件的低运动窗口，分别取三轴陀螺均值，得到各自零偏初值。软约束让它们在联合优化中小幅调整。不会拿另一轮上电或长静止数据的均值直接代替当前零偏。
3. **动态陀螺初始化 R 和 dt。** 搜索时间差，把两个三轴角速度序列配准，通过去均值的旋转配准求相对 R；去均值避免常量差偏置主导旋转估计。随后将静止零偏带入后续模型。三轴转速必须变化，只有纯 yaw 不足以完成本流程。
4. **动态加速度差初始化 xyz。** 把两路比力旋转、移时到同一坐标系和时刻。共同的平移加速度与重力抵消，剩余的切向、向心加速度由角运动和相对杆臂决定。通过 0.16 秒积分模型降低直接求角加速度的噪声，鲁棒线性求解三维平移和三轴常量加速度差偏置。z 与 x/y 一起估计。
5. **联合精修。** 同时优化 R、xyz、dt、两颗陀螺的三轴零偏、三轴加速度差偏置，共 16 个自由度。动态陀螺与加速度差残差使用 soft-L1；静止陀螺零偏使用单独的高斯软约束，按积分窗口重叠修正动态残差权重。
6. **独立验证并保留分步对照。** 第二份记录独立估计参数，再把第一份全部参数冻结后预测第二份，并反向检验。同时计算初始化/分步结果的冻结预测和参数差。不会只因训练残差下降而自动换方法；如要采用分步结果，可显式用 `--method staged` 重跑同一验证流程。

单次静止只能得到带加速度零偏影响的比力方向。脚本将其保存为检查信息，**不把它当作已知完整 R，不从中求 yaw，也不据此宣称估出了独立加速度零偏**。两路原始比力在正确对齐后共同重力抵消，所以动态差分无需预先去重力。

`accel_difference_bias_m_s2` 是两颗传感器旋转到同一坐标系后的等效常量差，**不能直接填进 LIO 的单颗 IMU 加速度零偏**。比例因子、轴间非正交和时变偏置未完整建模。

默认静止筛选：1 秒窗口，两颗各至少 80 点、最大相邻时间缺口 <40 ms、陀螺标准差模长 <0.01 rad/s、均值模长 <0.03 rad/s、加速度标准差模长 <0.15 m/s²，取首个合格窗口。软约束每轴尺度为标准差除以 sqrt(6)，并设 0.001 rad/s 下限；它是沿用比较实验的工程尺度，不是严格统计置信区间。没有窗口时默认报错，**不会自动使用零偏为零的假设或自由零偏优化**。

## 录制

1. 保持两颗 IMU 刚性安装、线缆固定，传感器工作温度稳定。录原始两路 IMU，在在线时间修正节点之前取数据；录制期间不要重启传感器或切换时间源。
2. 每条记录开始先放稳静止约 10 秒，再抱起底盘，分别绕车体前后、左右、上下三个方向来回转动，再加入混合转动。建议每轴约 ±20–45°，不断改变转速和转向；不要只绕竖直轴转，也不要只做长时间匀速旋转。动作平顺，避免碰撞、松动和数据饱和。结束后再放稳静止约 5–10 秒。静止不要求车体水平，但必须真正放稳。
3. 起步建议动态运动录制 60–90 秒，另加前后的静止段，三轴都覆盖多次，并包含不同快慢的动作。这是采集建议，最终看激励和验证检查，而不是只看时长。
4. 同次上电再录一条带前后静止段的独立验证记录，改变动作次序和快慢，保持安装不变。第二条用来独立标定和冻结参数预测，不能复制第一条。

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
  --method joint_static_soft \
  --imu0 /imu --imu1 /livox/imu \
  --accel-scale0 1 --accel-scale1 9.80665 \
  --output calibration/dynamic_imu_pair.json
```

如果 MID360 的上游节点已经转成 m/s²，把 `--accel-scale1` 改成 `1`；禁止重复换算。两个角速度默认均为 rad/s，若输入为 °/s，显式设置对应 `--gyro-scale0` 或 `--gyro-scale1` 为 `0.017453292519943295`。脚本检查比力模长以捕获明显单位错误，不会猜测并自动修正单位。

支持 ROS1 `.bag`、ROS2 bag 目录以及 `.npz`。NPZ 的两个 key 由 `--imu0/--imu1` 指定，各自为 N×7 数组，列顺序 `[timestamp_s, ax, ay, az, gx, gy, gz]`，无需预先同步或统一采样率。默认 dt 搜索 ±0.3 s，可用 `--dt-bound` 显式调整。只估计常量偏移，不能代替持续时钟漂移模型。

输出文件必须不存在，避免覆盖上一轮结果。单条记录可以运行，但状态仅为 `fit_passed_needs_independent_validation`，应继续录第二条。`rejected` 时返回退出码 2；不要因为 JSON 中有数值就使用该结果。

## 方法选择

| `--method` | 静止段 | 零偏处理 | 用途 |
|---|---|---|---|
| `joint_static_soft` | 必需 | 两颗陀螺零偏围绕静止均值软约束 | 默认 |
| `joint_fixed_bg` | 必需 | 两颗陀螺零偏固定为静止均值 | 比较固定与软约束 |
| `staged` | 必需 | 固定静止均值，保留分步解，不联合精修 | 独立验证后的分步选择 |
| `joint_reference_fixed` | 不使用 | 明确假设 IMU0 陀螺零偏为 0，估另一颗等效零偏 | 无静止数据的有条件试验 |
| `joint_free` | 不使用 | 两颗零偏完全自由 | 实验选项，实测有不稳定风险 |

默认输出 `estimate` 为所选方法结果，`staged_baseline.estimate` 为对应分步结果。`diagnostics.static_info` 保存选中的窗口、零偏均值、软约束尺度和比力方向。`validation.staged_baseline` 保存双向冻结预测、参数差和“所选方法减去分步”的残差变化，正值表示所选方法更差；没有自动切换或替用户选择参数。

## 输出方向与验收

`imu0` 是外部 IMU、`imu1` 是 MID360 内部 IMU 时：

- `R`：把 IMU1 的向量转到 IMU0；四元数顺序为 xyzw。
- `t_m`：IMU1 感测原点在 IMU0 坐标系中的位置，xyz 均估计，单位 m。
- `td_s`：同一事件 `timestamp0 = timestamp1 + td_s`。在 IMU0 时刻 `t`，应读取 IMU1 的 `t - td_s`。这不是自动推断的点云时间偏移。
- `gyro_bias0_rad_s`、`gyro_bias1_rad_s`：各自在本 IMU 坐标系中的陀螺零偏；只在记录检查通过后考虑使用。
- `accel_difference_bias_m_s2`：转到 IMU0 后的加速度常量差偏置；不是单颗加速度计独立内参。

有独立验证记录时，只有状态 `passed_recording_checks_not_absolute_accuracy_certified` 表示通过当前记录检查。检查包括：

- 默认模式有合格静止窗口，联合零偏未偏离静止均值超过 5 倍软约束尺度。
- 足够的多轴动态激励，优化收敛，参数矩阵无明显退化，dt 不顶搜索边界。
- 动态陀螺/加速度差逐分量 RMSE 默认不超过 0.03 rad/s、0.25 m/s²。
- 各自陀螺零偏模长不超过 0.1 rad/s；跨记录各自零偏变化不超过 0.02 rad/s。
- 两次独立标定旋转差不超过 0.5°，三维平移差不超过 15 mm，dt 差不超过 3 ms。
- 两个方向的跨记录预测都通过；预测时 R、xyz、dt 和所有偏置均冻结。

这些是用于拒绝明显异常结果的工程门限，**不是 15 mm 或其他绝对精度保证**。需求更严时，应查看具体重复差、杆臂实测值和实际 LIO 效果；不要仅放宽门限让失败结果通过。比例因子、轴间非正交和时变偏置未完整标定。

自由估计零偏在 HRMC、MILUV、Hilti 的试验中均出现不稳定现象，因此不作为默认模式。默认静止软约束的雅可比条件数包含先验影响，不能把它解释成所有参数仅由动态数据独立观测得到。若失败，先检查静止条件、三轴变速激励、时间戳、单位和刚性安装；不要仅放宽门限。

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

## 数据集证据与边界

- HRMC 三条训练记录及另三条冻结测试支持“静止零偏约束 + 动态精修”作为默认候选。静止固定与软约束的新增记录加速度差 RMSE 均约 0.07830 m/s²，分步约 0.08099；软约束与固定几乎打平，不能声称软约束绝对最优。相对作者平移参考仍约差 14 mm。
- MILUV 完整标定记录的前后分段试验中，无静止的参考零偏固定联合法把留出加速度差 RMSE 从约 0.122 降到 0.112 m/s²，但平移参考差仍约 9.28 mm，旋转分段一致性变差。Hilti 2021 的分步与同类联合法留出残差均约 0.040 m/s²，平移参考差约 7.1 mm。两者没有通过原静止窗口筛选，因此不能作为静止默认模式的完整验证。
- FusionPortable 受下载配额限制，仅完成约 8 秒原始前缀探索；不是完整数据集验收。作者外参均为标定参考，不是独立毫米真值。
- mix-cal 无合格静止段，旧纯动态实验的约 0.49 mm 平移重复差不能当作默认静止流程已经通过的证据。

本次固化的是带静止约束、分步对照和拒绝条件的可复现流程，不是对所有传感器、噪声条件或安装场景的最优精度保证。

独立解析信号测试覆盖有/无静止、非零零偏、非单位 R、三维杆臂、37 ms 时差、不同采样率、含噪独立验证、错误时间符号、纯 yaw 拒绝、同点 IMU 的零偏退化及测量锚点组合。运行：

```bash
OPENBLAS_NUM_THREADS=1 .venv_imu_calib/bin/python -m unittest discover \
  -s src/robot_base/test -p test_dynamic_imu_calibration.py -v
```

离线读取使用 rosbags，无需 ROS。当前研究主机未安装 ROS；没有进行 colcon 构建或现场 ros2 bag record 测试。
