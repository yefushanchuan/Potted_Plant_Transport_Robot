# 差速底盘、外置 IMU、MID360 标定

工具：`src/robot_base/scripts/imu_chain_calibration.py`。
配置模板：`src/robot_base/config/imu_chain_calibration.yaml`。
脚本支持 ROS 2 原始数据采集和不依赖 ROS 的离线求解，不发布速度、不修改运行配置/TF。
旧 `dual_imu_calib.py` 的单轴陀螺 SVD 不能确定 yaw，底盘平面运动请使用本工具。

## 本工具求什么

1. 手测一个外置 IMU 感测原点到车体参考点的 x/y/z，作为固定锚点。
2. 静止重力求两路传感器相对水平车体的倾角；直线加减速求 yaw。
   机械安装旋转先验用于消除前/后方向的 180° 歧义。
3. 变速旋转的两路陀螺求时间差；有可靠先验时可以直接固化，包括 0。
4. 固定 R、dt 和两 IMU 的车体竖直高度差，利用旋转时两路加速度之差，
   联合求相对 x/y 和三个常值加速度差偏置。
5. 接上手测锚点和 MID360 内部变换，输出外置 IMU、内置 IMU、点云原点相对车体的变换。

**不使用“轮轴中心加速度为零”的假设来求轴心位置。** 两路差分消去共同的平动和重力，
要求传感器刚性固定、时间与方向对齐。即使中心轻微晃动，模型仍适用。
拟合使用完整三维刚体旋转项，保留 roll/pitch 角运动和固定高度的贡献：

```text
delta_f_B = [alpha_B × t_B] + [w_B × (w_B × t_B)] + bias_B
t_B = 从外置 IMU 指向内置 IMU 的向量，以车体 B 坐标表示
```

相同的积分窗口作用于加速度差和运动学项；角加速度的窗口均值由角速度端点差计算。
先计算并平均向心项，不能用“平均角速度的平方”代替。
通过 Huber IRLS 抑制离群点。前半段拟合后冻结参数，在后半段预测；另独立拟合后半段作一致性检查。
常值偏置消除后的设计矩阵用于可观性检查：仅一档匀速旋转可能与偏置混淆，必须改变转速。

## 坐标、时间、单位

所有变换 `T_A_B` 表示 `p_A = R_A_B * p_B + t_A_B`。
R 按行排列，t 是 B 原点在 A 中的位置；距离 m、角速度 rad/s、加速度 m/s²。

| 代号 | 含义 |
| --- | --- |
| B / body | 配置中的车体坐标系，x 前、y 左、z 上 |
| E / external | 外置 IMU 的实际感测原点和消息轴向 |
| M / mid360 | MID360 内置 IMU 感测原点和消息轴向 |
| L / lidar | MID360 点云原点 |

当前工程在线 LIO 使用 `base_footprint`。它与轮轴中点的 x/y 相同，z 原点在地面；
当前 URDF 的 `base_link` 又比物理轮轴中心高 0.1 m，不能把三个参考点的 z 混用。
模板注释中的 URDF 名义位置不是实测值，不会自动填入结果。

```text
同一物理事件：stamp_external = stamp_mid360 + time_offset_s
对齐方式：external(t) 对应 mid360(t - time_offset_s)
```

例如外置 IMU 消息的时间戳比内置 IMU 晚 20 ms，参数填 `+0.020`。
这与 onlineoffset 的 `t_imu = t_lidar + td` 符号一致，但只有确认点云与内置 IMU 使用同一时间基准后，
才能把本结果作为 LiDAR–外置 IMU 的时间先验；工具不会自动设置 LIO 的 `initial_td`。
连续通电也可能有时钟漂移/接收延迟变化，工具会比较旋转前后两段的独立 dt 估计。

两路 CSV 保存驱动原值，不根据话题名称自动判断单位。
当前常用 HIPNUC 输出为 SI；原始 Livox 驱动加速度为 g，配置乘 9.80665。
如果你的中间节点已转换为 SI，改成 1.0。静止模长检查会拒绝明显错误单位或去重力后的数据。

## 安装和准备

ROS 环境安装包声明的运行依赖，并编译：

```bash
sudo apt-get install python3-numpy python3-scipy python3-yaml
colcon build --packages-select robot_base --symlink-install
source install/setup.bash
ros2 run robot_base imu_chain_calibration.py --help
```

纯离线求解只需要 Python 3、NumPy、SciPy、PyYAML，可以直接运行源码脚本。
不需要把 ROS 包装进新的 `robot_base.python` 目录。

启动两路 IMU 驱动，在一次连续通电会话内采集。若重启设备或改变时间同步配置，重新采集整套数据。
使用最终给 LIO 的外置 IMU 数据话题；消息中的加速度必须保留重力，且轴向不能在采集中切换。
采集不使用 ApproximateTimeSynchronizer，保留每条消息的 header 时间；原始时间回退、重复时间、
非有限数据或 frame_id 改变会终止采集并记录错误。已有文件不会覆盖。

## 采集三段数据

每个命令在指定时间内采集，然后退出。人工遥控完成动作；脚本不驱动底盘。
默认话题 `/imu` 与 `/livox/imu`，可用 `--external-topic` / `--mid360-topic` 修改。

```bash
mkdir -p ~/imu_chain_session
cp src/robot_base/config/imu_chain_calibration.yaml ~/imu_chain_session/config.yaml

# 1. 静止 10 秒；水平地面，车体基准面水平，不仅仅是“车不动”
ros2 run robot_base imu_chain_calibration.py record \
  --phase static --duration 10 --directory ~/imu_chain_session

# 2. 同一直线上多次前进/后退、加速/减速，避免转向和地面坡度变化
ros2 run robot_base imu_chain_calibration.py record \
  --phase straight --duration 60 --directory ~/imu_chain_session

# 3. 多档转速、加减速、正反转和停顿；前后半段都应包含充分变化
ros2 run robot_base imu_chain_calibration.py record \
  --phase rotate --duration 90 --directory ~/imu_chain_session
```

每段得到 `PHASE_external.csv`、`PHASE_mid360.csv` 和采集元数据。
若在线采集在动作开始前就启动，两路驱动应已经持续输出，避免空等消耗采集时长。
也可以通过 ROS 2 bag 回放采集；分段选择静止/直线/旋转，避免把整段混合运动误标为直线。
自行从 bag 导出时，两路必须使用原始 header 时间，保留顺序，不以接收时间替代：

```text
t,ax,ay,az,gx,gy,gz
1700000000.000000000,0,0,9.80665,0,0,0
...
```

## 填入测量值并求解

编辑会话中的 `config.yaml`：

- `t_body_external_m`：实测外置 IMU 原点位置；大致正后方不代表 y 恰好为零。
- `external_to_mid360_z_in_body_m`：内置 IMU 与外置 IMU 的高度差，以 B 的 z 为准，
  不是相对变换在倾斜外置 IMU 中的 tz，也不是雷达壳体高度差。
- `R_body_external` / `R_body_mid360`：机械安装旋转先验，确保与真实姿态相差小于 45°。
  单轴旋转不能自动补出 yaw；不能把不明确的轴向随意填成单位矩阵。
- 核对单位比例以及实际 MID360 型号对应的 `R_mid360_lidar` / `t_mid360_lidar_m`，
  再将 `factory_transform_confirmed` 设为 true。模板的平移是点云原点在内置 IMU 中的位置。

`rotation_mode: estimate` 估计两路 R；`rotation_mode: fixed` 完全固定两路 R，无需直线文件。
固定一个 IMU 的轴心外参、只标另一台的方向时，可分别设置：

```yaml
rotation_mode:
  external: fixed
  mid360: estimate
time_offset_s: 0.004  # 举例：替换为自己的已知值；null 表示重新标定
```

即使固定 R/dt，仍采集静止段估计陀螺偏置、检查单位，旋转段用于求 x/y。
倾角估计需要水平基准，单次静止不能分离加速度零偏与倾角；已有更可信旋转时选 fixed。

```bash
ros2 run robot_base imu_chain_calibration.py solve \
  --config ~/imu_chain_session/config.yaml \
  --directory ~/imu_chain_session \
  --output ~/imu_chain_session/result.yaml

# 不依赖 ROS 的同一入口
python3 src/robot_base/scripts/imu_chain_calibration.py solve \
  --config ~/imu_chain_session/config.yaml \
  --directory ~/imu_chain_session \
  --output ~/imu_chain_session/result_offline.yaml
```

退出码 0 表示通过配置的质量门限；2 表示输入无效、激励不足或一致性检查未通过。
后两项质量检查失败仍输出诊断结果，但不会附带供滤波器使用的参数块。
默认门限：dt 两段差 3 ms、xy 两段差 20 mm、留出段加速度每分量总体 RMSE 0.1 m/s²。
这些门限只是起始筛选条件，不是宣称可达到的误差。检查原始数据和运动条件后再调整。

## 使用结果

`transforms` 给出 B←E、B←M、B←L、E←M、E←L，附带 URDF 顺序的 roll/pitch/yaw（rad）。
输出使用前半段拟合的 x/y；后半段只用于验证，没有再把验证集并入拟合。

当前分支与标准 FAST-LIO 有区别：在线模式将 IMU 加速度换算到车体中心，因此 B←E 的平移会进入
杆臂补偿。结果的 `fastlio2_online_yaml_patch` 包含当前在线模式的两组 sensor-to-body 参数和
`online_use_tf_extrinsics: false`。它是合并到现有 fastlio2 YAML 顶层的参数片段，不是完整配置文件，
不是 ROS `--params-file` 格式；同时确认 `online_imu_topic` 使用本次外置 IMU、`imu_accel_scale` 正确。
在线模式忽略旧 `r_il/t_il`；单独修改这两个旧参数不会生效。

若继续使用默认 `online_use_tf_extrinsics: true` 或 indoor_location，需要把结果转换到 URDF
各关节的实际父坐标系后更新固定 TF。当前传感器父级是 `chassis_link`，
不能把输出的 `base_footprint` 下绝对位置直接写进 `chassis_link` 下的 joint origin。
不要同时为已有父子关系发布第二组固定 TF。

`external_imu_lidar_extrinsic` 则表示点云→外置 IMU 的 r_il/t_il，仅供实际使用外置 IMU 且采用
这种外参约定的 LIO 配置；当前旧模式默认用 `/livox/imu`，不能直接拿外置 IMU 的外参替换。
工具不会更新 URDF 或启动滤波器，避免未测的模板值成为运行参数。

## 验证及限制

离线回归：

```bash
python3 -m unittest discover -s src/robot_base/test -p 'test_imu_chain_calibration.py' -v

# ROS 2 环境中同时运行采集测试
colcon test --packages-select robot_base
colcon test-result --verbose
```

合成测试由独立的叉乘刚体模型生成，包含正/负/零时差、400/200 Hz 不同采样率、倾斜安装、
近正后方的锚点、共同中心加速度、三维旋转、偏置/离群点、不可观匀速旋转和缺数据。
它检验实现与符号，不代表真机精度。

Humble/Jammy 容器中已完成整个 `robot_base` 包编译、14 项离线测试与 1 项真实 ROS 消息采集测试，
并检查已安装的 `ros2 run robot_base imu_chain_calibration.py --help` 入口。
采集测试使用合成消息，验证不同频率、原始时间戳、frame_id 和单位保持，不连接真实底盘。

2026-09-24 用已有 IILABS `livox_calib_square_ccw_rot` 实采提取数据复跑：
固定之前独立求得的 R、dt，固定相对高度 0.21712 m，头 3 秒估计静止陀螺偏置，
整段双 IMU 差分数据按时间分半（包含直行/停顿/转动，未假设中心静止）。

| 检查 | 结果 |
| --- | --- |
| 第一半相对 x/y | [9.388, 22.334] mm |
| 第二半独立 x/y | [9.374, 22.548] mm |
| 两段 x/y 距离 | 0.215 mm |
| 留出段加速度 RMSE | 0.03888 m/s²（仅常值偏置基线 0.10941） |
| 若重新估计 dt，两段结果 | -4.075 / -3.955 ms |

这是当前脚本在该数据上的重复性/预测验证，选择的数据范围和旧实验不同，数字不能直接当作同一实验的改进。
CAD 名义相对 x/y 约 [11, 23.29] mm，未提供独立高精度芯片原点真值。
数据集不能证明差速轮轴中点的绝对精度，也未验证本车机械测量、实际 ROS 接收延迟或完整 LIO 轨迹。
数据来源：[IILABS 官方数据集](https://jorgedfr.github.io/3d_lidar_slam_benchmark_at_iilab/)。
