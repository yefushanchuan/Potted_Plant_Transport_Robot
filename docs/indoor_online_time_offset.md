# indoor_location 在线时间偏移估计

## 范围与约定

分支 `feat/indoor-online-time-offset` 基于 `demo` 的
`bd8346e8e4aeecfaa187326b16d114d51c539450` 创建。

本次修改 `indoor_location` 的定位滤波器，并将同一时间估计核心接入 FAST-LIO2
增量建图（详见 `fastlio2_online_time_offset.md`）。HIPNUC 驱动继续使用原来的 ROS
接收时间戳；不读取 HIPNUC 设备时钟，不覆盖输入消息时间戳。回环优化、PCD 保存
和描述子建库算法没有修改。`dual_descriptor.launch.py` 仅透传开关。

统一使用秒，定义：

```text
t_imu_ros = t_lidar + td
```

例如给 IMU header 额外加 20 ms，估计的 td 应相应增加约 20 ms。
这里估计的是两路观测在运动模型下的有效时间偏移，含接收延迟的低频部分，
不能从软件滤波中恢复每一条消息独立、不可预测的网络/串口排队延迟。

## 实现

`agrobot_time_sync` 包的 `time_offset_filter.*` 和 `time_offset_motion.*` 是纯 Eigen C++ 实现，
与 ROS 通信分离。误差状态为 19 维：

```text
x = [p, v, rotation_error, gyro_bias, accel_bias, gravity, td]
td_next = td + random_walk
P_next = F P F^T + Q
```

保留 td 与其他状态的完整交叉协方差。正常跟踪直接使用带逐点时间的原始观测，
不把同一帧 ICP 位姿再次当作独立观测更新滤波器。

```text
t_point = lidar_header + point_offset + td
world_point = R(t_point) * base_point + p(t_point)
residual = plane_normal · world_point + plane_offset

H_td = plane_normal · [v(t_point) + R(t_point) * ((gyro-bg) × base_point)]

objective = prior_error^T * inverse(P_prior) * prior_error
          + sum(Huber(point_to_plane_residual / point_std))
```

代码按以米为单位的 `huber` 阈值进行 IRLS 加权。点到先验地图局部平面的关联复用地图
KD-tree，局部平面通过 PCA 检查。每次迭代都从同一个先验构建正规方程；一帧结束后
只更新一次协方差，并变换到最终旋转切空间。平面对应关系与地图本身视为固定参考，
当前协方差不包含建图误差与外参误差。

滤波状态锚定在固定 IMU 时间。积分/插值计算观测时刻的轨迹和完整状态敏感度，
改变 td 不移动该状态锚点。观测可向锚点前方回推运动，协方差不反向传播。
插值要求实际 IMU 样本夹住查询时刻，禁止缺数据时外推。

输入处理同时做了以下调整：

- 在线路径目前支持本工程 Livox `PointCloud2` 格式：`timestamp` 为逐点**绝对纳秒**，
  `header` 为帧时间原点。ROS1 `CustomMsg` 需先转换，不能直接订阅。
- 每个体素保留一个真实采样点及其时间，避免把跨时刻点的质心作为真实观测。
- 在线路径按 `sensor_msgs/Imu` 的 SI 单位读取加速度和角速度，通过 TF 转到 base，
  做杆臂的向心和角加速度补偿；不使用旧路径依赖加速度 z 方向的单位/轴向猜测。
- 同步按当前 td 及 `max_td_step` 留出两侧 IMU 余量，保留前一状态锚点覆盖。
- 发布位姿、去畸变点云及 odom 查询使用 `scan_end + td`，统一到 IMU ROS 时间参考。
  在线路径直接发布 map→base 时使用该观测时间；map→odom 保持原有以当前时间发布校正量的策略。
- IMU 缓存保留 2 秒，LiDAR 队列最多 10 帧；消息时钟回退清空队列和时间标定。
- 位姿重定位保留 td 均值及方差，清除与重置位姿的交叉项；命令引起的状态重置
  交给处理线程执行。

原 ICP 用于初始化/重定位。关闭开关时恢复原 ICP 位姿观测加 18 维 ESKF 跟踪路径。

## 可观性与异常处理

滑动 IMU 激励统计结合地图几何信息门限决定是否更新 td。时间信息检查会消去六维
瞬时位姿方向，避免把匀速下可被位姿吸收的时间雅可比误当成充分约束。
激励不足或几何退化时采用考虑 td 不确定性的冻结更新，不收缩 td 方差。

IMU 缺口、覆盖不足、非单调时间、匹配不足、非有限值和非正定协方差会阻止对应更新。
默认 td 范围为 ±100 ms，单帧变化不超过 5 ms。现在时间修正超限时，在边界上
重新求其余 18 个状态的增量，而不是丢弃整帧，或只裁剪 td 后照搬其他状态增量。
一帧内所有迭代共用相对该帧先验的同一个边界，不能靠多次迭代累计突破上限。

```text
lower = max(-max_abs_td, td_prior - max_td_step)
upper = min(+max_abs_td, td_prior + max_td_step)
delta_td = clamp(td_current + delta_td_proposed, lower, upper) - td_current
A_oo * delta_other = rhs_other - A_ot * delta_td
```

最终处于边界时，采用其余状态的局部约束增益和完整 Joseph 协方差更新；td 增益
为零，保留 td 方差及其对其他状态的不确定度影响。数值边界不作为时间观测。
这是局部活动约束近似，并非截断高斯分布的精确后验。边界接受帧诊断为
`tracking_td_limited`；`td_limited` 表示本次迭代中曾触及限幅。
匹配、位姿增量、IMU 覆盖等检查仍然生效。没有新位姿输出时在线状态不再报告正常跟踪。
这些边界用于局部在线估计，不能把任意设备时钟纪元或从零开始的 500 ms 偏差
视为已经验证可自动收敛的场景。

## 启动与参数

原启动命令可继续使用；以下两个 launch 增加 `online_time_offset` 参数，默认 true：

```bash
ros2 launch indoor_location run_argbot_mid360.launch.py \
  map_file:=/absolute/path/map.pcd online_time_offset:=true

ros2 launch dual_descriptor_relocalization dual_descriptor.launch.py \
  online_time_offset:=true
```

地图等原有参数按原使用方式传入。回退：`online_time_offset:=false`。
详细参数位于 `config/argbot_mid360.yaml` 的 `time_offset` 节；launch 开关优先于 YAML。
该节由应用直接读取 YAML，不是支持动态修改的 ROS 参数集合。

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| initial_td / initial_td_std | 0 / 0.02 s | 初始偏移及标准差 |
| td_noise | 0.0005 s/sqrt(s) | 时间偏移随机游走密度 |
| max_abs_td / max_td_step | 0.1 / 0.005 s | 总范围 / 单帧接受变化上限 |
| max_imu_gap / max_scan_duration | 0.05 / 0.25 s | 最大 IMU 间隔 / 扫描时长 |
| point_std / huber | 0.08 / 0.15 m | 点面噪声 / 鲁棒阈值 |
| min_fitness / max_rms | 0.6 / 0.2 m | 有效对应比例 / 残差门限 |
| max_points / iterations | 1200 / 4 | 每帧最多观测点 / 迭代次数 |

诊断：

```bash
ros2 topic echo /indoor_location/time_offset
```

消息类型 `std_msgs/msg/String`，内容为 JSON。字段包括 `lidar_time_sec`、`td_ms`、`td_std_ms`、
`accepted`、`td_updated`、`td_limited`、`reason`、`fitness`、`rms_m`、`matches` 和激励统计。
`td_std_ms` 是当前模型内部的不确定度，不能当作实际标定误差保证。

## 验证环境与已有结果

验证目录按要求放在：

```text
/home/descfly/Downloads/Potted_Plant_Transport_Robot-demo/docker/time_offset_humble
```

Humble/Jammy Docker 编译 `agrobot_time_sync`、完整 `indoor_location` 和 `fastlio2`，源码只读挂载，build/install/log 在
容器 `/tmp/time-validation`，测试日志和脚本在上述验证目录。运行 `validate.sh`
执行编译、GTest 与 ROS 模拟回放；不启动真实传感器。

13 项 GTest 已通过：19 列雅可比数值校验、正/负时差收敛、缓慢漂移、静止及匀速
冻结、插值边界、IMU 缺口/重复时间、失败更新事务性、重定位、去畸变和非法配置；另覆盖大偏差多帧限幅收敛、绝对边界与协方差保守性。
实际 ROS 节点的理想合成回放已覆盖 ±20 ms、时钟回退和关闭开关的旧路径。
这些理想合成误差不能用于声称真机精度。

限幅后的实采三组原始/±20 ms 注入均有 355/356 帧位姿输出（首帧初始化）。
注入量响应误差分别约 0.106 / 0.169 ms；这些是差分一致性指标，不能作为绝对标定真值误差。
默认 5 ms 单帧上限也独立回放验证。源码提取为共享包后重新运行核心与 ROS 回归。

实采数据及逐项对照结果记录在验证目录的 `PUBLIC_DATA_REPORT.md`，原始诊断和位姿
保存在 `results/public_*.json`。该报告明确区分原始文件、恢复片段、先验地图来源、
测试配置与默认配置，以及没有外部真值的限制。

## 参考来源

- [LI-Init 官方项目与公开标定数据入口](https://github.com/hku-mars/LiDAR_IMU_Init)：
  LiDAR–IMU 时间偏移和外参标定问题、运动激励、Livox IMU 单位说明。
- [LI-Init Temporal Calibration 数据目录](https://drive.google.com/drive/folders/1QsyZcT2f0u8fohVj9HjA62n7626khWF5)：
  本次公开数据来源。本实现使用先验地图中的在线滤波更新，并非移植 LI-Init 批量初始化算法。
