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

  // ── 0. 悬空与天花板噪点过滤 ──────────────────────────────────
  if (c.min_z > params_.max_bottom_z) return 0.0f; 
  
  // ── 1. 基础尺寸过滤 (硬拒绝) ──────────────────────────────────────────
  if (h < params_.min_height || h > params_.max_height) return 0.0f;
  if (actual_w < params_.min_width || actual_w > params_.max_width) return 0.0f;
  if (actual_d < params_.min_depth || actual_d > params_.max_depth) return 0.0f;  

  // ── 2. 单面视角特征提取 (2.5D 逻辑) ──────────────────────────────────
  float front_aspect = h / std::max(actual_w, 0.001f); // 数学安全保护
  if (front_aspect < params_.min_aspect_ratio || front_aspect > params_.max_aspect_ratio) return 0.0f;

  // ── 3. 评分系统 (Soft Scoring 0~1) ───────────────────────────────────

  // A. 高度得分
  float h_score = std::exp(-std::pow((h - params_.ideal_height) / params_.height_tolerance, 2.0f));

  // B. 正面比例得分
  float asp_score = std::exp(-std::pow((front_aspect - params_.ideal_aspect_ratio) / params_.aspect_tolerance, 2.0f));

  // C. 点云数量得分
  float pts = static_cast<float>(c.cloud->size());
  float density_score = std::min(pts / params_.max_points_for_score, 1.0f);

  // ── 4. 权重组合 ──────────────────────────────────────────────────────
  float score = params_.weight_height * h_score
              + params_.weight_aspect * asp_score
              + params_.weight_density * density_score;

  score = 0.8f + 0.2f * std::sqrt(score);

  return std::clamp(score, 0.0f, 1.0f);
}

}  // namespace plant_detector
