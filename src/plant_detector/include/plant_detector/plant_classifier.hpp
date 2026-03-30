#pragma once
#include "plant_detector/euclidean_cluster.hpp"

namespace plant_detector
{

struct ClassifierParams
{
  // --- 1. 基础尺寸过滤 (米) 硬限制 ---
  float min_height;
  float max_height;
  float min_width;
  float max_width;
  float min_depth;
  float max_depth;

  // --- 2. 空间位置过滤 ---
  float max_bottom_z;         // 悬空最大高度 (过滤天花板/悬空噪点)

  // --- 3. 形状比例过滤 (正面宽高比 = 高度 / 宽度) ---
  float min_aspect_ratio;     // 最小正面宽高比 (矮胖型下限)
  float max_aspect_ratio;     // 最大正面宽高比 (瘦高型上限)

  // --- 4. 软评分打分机制 (Soft Scoring) ---
  float ideal_height;         // 理想高度 (m)
  float height_tolerance;     // 高度评分容差 (高斯分布分母)
  float ideal_aspect_ratio;   // 理想正面宽高比
  float aspect_tolerance;     // 宽高比评分容差 (高斯分布分母)
  float max_points_for_score; // 达到满分(密度分)所需的点云数量

  // --- 5. 权重分布 ---
  float weight_height;        // 高度得分权重
  float weight_aspect;        // 比例得分权重
  float weight_density;       // 密度/点数得分权重

  // --- 6. 最终输出阈值 ---
  float confidence_threshold; // 发布检测结果的最低置信度
};

class PlantClassifier
{
public:
  explicit PlantClassifier(const ClassifierParams & params = {});

  /**
   * Evaluate a single cluster.
   * @returns confidence score in [0, 1].  0 means "definitely not a plant".
   */
  float classify(const ClusterResult & cluster) const;

  bool isPlant(const ClusterResult & cluster) const
  {
    return classify(cluster) >= params_.confidence_threshold;
  }

  void setParams(const ClassifierParams & p) { params_ = p; }

private:
  ClassifierParams params_;
};

}  // namespace plant_detector
