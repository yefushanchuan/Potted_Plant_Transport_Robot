
-- 继承建图配置作为基础配置
include "cartographer_mapping.lua"

-- ========== 纯定位模式配置 ==========
-- === 启用纯定位模式 ===
-- 冻结地图构建，只进行定位，不创建新的子地图
TRAJECTORY_BUILDER.pure_localization_trimmer = {
    max_submaps_to_keep = 3,              -- 保留的子地图数量，减少内存占用
}

-- === 禁用新子地图创建 ===
-- 强制使用预加载的地图，防止在定位过程中创建新子地图
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 999999    -- 设置极大值，确保不会创建新子地图
 
return options  -- 返回完整配置选项

