# Nav2 Keepout Filter 配置指南

## 概述

`keepout_filter` 允许您在地图上手动标记禁行区域，机器人导航时会避开这些区域。

## 当前配置状态

✅ 配置文件 `Nav_RotationShim_DWB.yaml` 中已经配置了：
- `keepout_filter` 插件（在 `local_costmap` 和 `global_costmap` 中）
- `costmap_filter_info_server` 配置
- `mask_topic`: `/keepout_filter_mask`
- `filter_info_topic`: `/costmap_filter_info`

## 需要启动的节点

### 1. costmap_filter_info_server 节点

这个节点需要单独启动。修改 `nav03_navigation.launch.py` 添加以下节点：

```python
Node(
    package='nav2_costmap_filters',
    executable='costmap_filter_info_server',
    name='costmap_filter_info_server',
    output='screen',
    parameters=[nav2_configured_params],
)
```

### 2. Mask 发布节点

需要发布 `/keepout_filter_mask` topic（类型：`nav_msgs::msg::OccupancyGrid`）。

**方法 A：使用地图服务器发布静态掩码**

创建一个 YAML 配置文件指向您的 keepout mask 地图文件。

**方法 B：使用自定义节点动态发布掩码**

创建一个节点订阅地图并发布修改后的 keepout mask。

> **提示**：如果使用 `keepout_mask_publisher`，请改用 `keepout_mask.json`（JSON 结构）。本文档描述的是借助 `nav2_map_server` 的传统 YAML/PGM 方式，仅在需要基于静态图像生成 mask 时参考。

## 步骤 1：准备 Keepout Mask 地图

1. **复制地图文件**：
   ```bash
   cp /path/to/your/map.pgm /path/to/your/keepout_mask.pgm
   cp /path/to/your/map.yaml /path/to/your/keepout_mask.yaml
   ```

2. **编辑 keepout_mask.pgm**：
   - 使用图像编辑工具（GIMP、Photoshop等）打开
   - 在禁行区域绘制**白色**（255）表示可通行
   - 在禁行区域绘制**黑色**（0）表示禁行

3. **编辑 keepout_mask.yaml**：
   ```yaml
   image: keepout_mask.pgm
   resolution: 0.03
   origin: [-10.0, -10.0, 0.0]
   negate: 0
   occupied_thresh: 0.65
   free_thresh: 0.196
   ```

## 步骤 2：修改 Launch 文件

在 `nav03_navigation.launch.py` 中添加：

```python
# costmap_filter_info_server
costmap_filter_info_server_node = TimerAction(
    period=9.0,  # 在 navigation launch 之后启动
    actions=[
        Node(
            package='nav2_costmap_filters',
            executable='costmap_filter_info_server',
            name='costmap_filter_info_server',
            output='screen',
            parameters=[nav2_configured_params],
        )
    ]
)

# Keepout mask publisher (使用 map_server)
keepout_mask_server = TimerAction(
    period=9.0,
    actions=[
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='keepout_mask_server',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'yaml_filename': '/path/to/keepout_mask.yaml'  # 修改为实际路径
            }],
            remappings=[
                ('/map', '/keepout_filter_mask'),
                ('/map_metadata', '/keepout_filter_mask_metadata'),
            ],
        )
    ]
)
```

然后在 `LaunchDescription` 的返回列表中添加这两个节点。

## 步骤 3：验证配置

启动导航后，检查以下 topics：

```bash
# 检查 filter_info 是否发布
ros2 topic echo /costmap_filter_info

# 检查 mask 是否发布
ros2 topic echo /keepout_filter_mask

# 检查 costmap 是否应用了 filter
ros2 topic echo /global_costmap/costmap
ros2 topic echo /local_costmap/costmap
```

## 注意事项

1. **Mask 地图尺寸必须与主地图一致**（分辨率、原点、尺寸）
2. **Mask 值**：
   - 0（黑色）= 禁行区域（lethal）
   - 255（白色）= 可通行区域
   - -1 = 未知区域
3. **配置中的参数**：
   - `override_lethal_cost: True` - 禁行区域会被设置为 lethal cost
   - `lethal_override_cost: 200` - 设置的 cost 值
4. **启动顺序**：确保 costmap_filter_info_server 在导航节点之后启动

## 故障排查

- **Mask 未生效**：检查 `enabled: True` 是否设置
- **Topic 未发布**：检查节点是否启动，查看日志
- **尺寸不匹配**：确保 mask 地图与主地图分辨率、原点一致
