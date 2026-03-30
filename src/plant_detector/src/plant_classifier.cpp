#include "plant_detector/plant_classifier.hpp"
#include <algorithm>
#include <cmath>

namespace plant_detector
{

PlantClassifier::PlantClassifier(const ClassifierParams & params)
: params_(params) {}

float PlantClassifier::classify(const ClusterResult & c) const
{
  const float h = c.height();
  const float w = c.width();
  const float d = c.depth();

  float actual_w = std::max(w, d); 
  float actual_d = std::min(w, d);

  // ── 0. 悬空与天花板噪点过滤 (新增) ──────────────────────────────────
  // 假设 base_link 的 Z=0 是地面。允许 RANSAC 误差，花盆的底部最高不能超过地面 15cm
  // 如果你的 base_link 在雷达中心处(离地23cm)，那么地面Z是 -0.23，这里就改为 c.min_z > -0.05f
  if (c.min_z > 0.15f) return 0.0f; 
  
  // ── 1. 基础尺寸过滤 (硬拒绝) ──────────────────────────────────────────
  if (h < params_.min_height || h > params_.max_height) return 0.0f;
  if (w < params_.min_width  || w > params_.max_width)  return 0.0f;
  // 注意：不再对厚度(d)做过于严苛的限制，只要大于极其微小的值即可，因为单面扫描可能极薄
  if (d < params_.min_depth  || d > params_.max_depth)  return 0.0f;

  // ── 2. 单面视角特征提取 (2.5D 逻辑) ──────────────────────────────────
  // 因为只能扫到一个面，点云的形状通常是一道“弧面”。
  // 我们不再要求 w 和 d 接近，只看正面的“宽高比” (Height vs Width)
  
  // 对于大部分室内盆栽，高度和宽度的比例通常在 0.5 (矮胖) 到 3.0 (瘦高) 之间
  float front_aspect = h / std::max(w, 0.001f);
  if (front_aspect < 0.3f || front_aspect > 4.0f) return 0.0f;

  // ── 3. 评分系统 (Soft Scoring 0~1) ───────────────────────────────────

  // A. 高度得分：基于你的盆栽，最理想的高度设为 0.25m 左右
  float ideal_h = 0.25f;
  float h_score = std::exp(-std::pow((h - ideal_h) / 0.3f, 2.0f));

  // B. 正面比例得分：最理想的盆栽正面宽高比大概是 1.0 (正方形) 到 1.5 (稍微偏高)
  float ideal_aspect = 1.0f; 
  float asp_score = std::exp(-std::pow((front_aspect - ideal_aspect) / 1.0f, 2.0f));

  // C. 点云数量得分：距离 40cm 的雷达应该能扫到非常密集的点，点数越多越确信
  // 假设 50 个点以上就有一定可信度，150 个点满分
  float pts = static_cast<float>(c.cloud->size());
  float density_score = std::min(pts / 150.0f, 1.0f);

  // ── 4. 权重组合 ──────────────────────────────────────────────────────
  // 减弱了对厚度特征的依赖，重点看高度、正面比例和点云密集度
  float score = 0.40f * h_score
              + 0.30f * asp_score
              + 0.30f * density_score;

  return std::clamp(score, 0.0f, 1.0f);
}

}  // namespace plant_detector
