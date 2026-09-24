# FAST-LIO2 在线时间偏移估计

## 范围

源码工程 `/home/descfly/Potted_Plant_Transport_Robot`，分支 `feat/indoor-online-time-offset`。
在当前 FAST-LIO2 增量 ikd-Tree 建图链路中加入在线时间估计；回环 BA、地图保存服务、
`save_pcd.sh`、`pcd2pgm.sh`、描述子建库算法不变。HIPNUC 保持 ROS header 时间来源。

`agrobot_time_sync` 是新增的纯 Eigen C++ 共享滤波库，供 indoor_location 和 FAST-LIO2
共同使用，避免维护两套时间模型和限幅算法。它不是另起一个 ROS 滤波节点。

当前仓库原 FAST-LIO2 是自定义 21 维滤波实现，包含 6 维外参、固定重力。
**在线模式使用固定外参的 19 维联合滤波：p、v、姿态误差、陀螺偏置、加计偏置、重力、td。**
并非简单将旧 21 维矩阵扩为 22 维。旧 IESKF State 在在线模式仅作发布/地图管理的数据适配，
不对同一帧再次更新。`esti_il=true` 与在线模式组合会明确报错。
关闭在线开关仍走原 21 维流程；其他型号的原配置默认关闭在线估计。

固定外参的采集、标定和配置方法见 [双 IMU 与底盘标定](imu_chain_calibration.md)。
该工具输出 sensor-to-body 参数；在线模式使用这些参数或 TF，不使用旧 `r_il/t_il`。

## 数据和时序

约定 `t_imu_ros = t_lidar + td`。在线模式消费本工程 Livox PointCloud2：
逐点 `timestamp` 是 FLOAT64 绝对纳秒，header 是扫描时间原点；curvature 内部保存相对毫秒。
默认 4 线，`lidar_lines` 可配置。ROS1 CustomMsg 需要转换；不能将相对秒字段冒充绝对纳秒。

argbot 在线默认 IMU `/imu`（HIPNUC/互补滤波输出），`imu_accel_scale=1.0`，加速度 m/s²、角速度 rad/s。
订阅采用 SensorDataQoS；不猜测加速度单位、不读设备时钟、不改写输入时间戳。
旧模式仍取原 `/livox/imu` 和原单位转换。在线 ROS 参数 `imu_topic` 可显式覆盖数据源。

默认从固定 URDF TF 查询 `base_footprint <- imu_link` 和 `base_footprint <- front_laser_link`。
等待外参齐全后开始收数据；旋转两路数据到 base，IMU 加速度补偿杆臂向心项和角加速度项。
点云已在 base 后，不再叠加旧 `r_il/t_il`。`online_use_tf_extrinsics=false` 时使用 YAML 中
两组 sensor-to-base 矩阵，适用于已知外参的数据集测试。

流程：

1. 累积默认 1 秒静止 IMU，检查激励和单位范围，初始化重力和陀螺零偏。
2. 首个有效扫描去畸变后建立 ikd-Tree 局部地图，不加载先验 PCD。
3. 按当前 td 和每帧限幅范围收集两侧 IMU，覆盖上一状态锚点；禁止缺数据时外推。
4. 输入体素选择真实采样点及时间，点到局部地图平面残差联合更新状态与 td。
5. 时间修正过大时，在 td 边界上重新解其他状态；低激励/退化时冻结 td 均值。
6. 成功后在修正时间 `scan_end + td` 计算位姿、全帧去畸变，再体素化写入局部地图。
   匹配失败的扫描不发布位姿/点云、不写入地图。匹配使用插入当前帧之前的地图。
7. body_cloud、world_cloud、lio_odom、lio_path 和 odom→base TF 共用修正时间。
   BA 现有消息配对接口保持一致。

每帧边界、联合求解及协方差处理见 `indoor_online_time_offset.md`。默认每帧 5 ms，
总范围 ±100 ms。该机制解决可观测的缓变偏移，不能恢复逐条独立随机接收延迟。

IMU 缓冲 2 秒，点云最多 10 帧。时钟回退清队列、时间状态、本节点局部地图和路径。
若积分遇到无法跨越的 IMU 缺口，停止更新并报告 `imu_gap_restart_required`，需要重启该建图会话；
不会把新原点自动合入旧地图。已经运行的外部 BA 会话不由本节点复位，跨时钟纪元应重启整条建图链路。
匹配拒绝可在后续良好观测上恢复，但本次没有新增全局重定位模块。

## 启动

```bash
ros2 launch ba_optimize ba_argbot_mid360.launch.py online_time_offset:=true
# 或单独启动 FAST-LIO2（需要既有传感器及固定 TF）
ros2 launch fastlio2 argbot_mid360.launch.py online_time_offset:=true
# 回退原算法与原 IMU 数据源
ros2 launch ba_optimize ba_argbot_mid360.launch.py online_time_offset:=false
ros2 topic echo /fastlio2/time_offset
```

两个 argbot launch 均默认 true。算法参数位于 fastlio2/config/argbot_mid360.yaml 的
`time_offset` 节，与共享库 Options 一致；YAML 不是运行中可动态修改的 ROS 参数。
`online_init_duration` 控制静止初始化时长。

诊断 JSON 包括 td、标准差、接受/限幅标志、原因、匹配比例和残差、map_points、map_inserted、处理耗时。
内部标准差没有计入地图关联/外参不确定度，不能视为真实标定精度保证。

## 验证

验证工具、Docker 和数据在用户指定的 Downloads 工程 docker/time_offset_humble。
`validate.sh` 编译两个节点和共享库，运行 13 项核心 GTest、两个节点各自的正负时差/旧路径 ROS 回放。
`RUN_PUBLIC_DATA=1 bash validate.sh` 追加相同公开实采片段的原始、±20 ms 时间注入、冻结 dt 对照。

FAST-LIO2 实采测试从前段静止观测初始化并独立增量建图，不读取定位测试的冻结 PCD。
`fast_smoke.py` 用已知真值检测 dt 收敛、每帧限幅、BA 输入时间戳配对、坏扫描不污染地图、时钟回退。
`fast_public_replay.py` 使用保留原始时序的实采数据，检查实际节点输出和地图增长。
结果与限制见验证目录 `FASTLIO2_DATA_REPORT.md`、`results/fast-*.json`、`results/fast_public_*.json`。

实采 bag 有零填充坏尾，仅恢复完整前缀约 37.7 秒；无轨迹真值、独立 td 真值或完整外参。
文件名虽含 mid360，实际为 6 线点云，硬件身份未核实。时间注入响应只能证明差分一致性，
自建地图点面残差包含地图适配效应，不能直接当作绝对轨迹或地图精度。没有进行真机验证。
