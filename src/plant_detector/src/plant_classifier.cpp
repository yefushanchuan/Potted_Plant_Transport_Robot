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

  // ── Hard rejection filters ────────────────────────────────────────────
  if (h < params_.min_height || h > params_.max_height) return 0.0f;
  if (w < params_.min_width  || w > params_.max_width)  return 0.0f;
  if (d < params_.min_depth  || d > params_.max_depth)  return 0.0f;

  // ── Elongation check (footprint should be roughly circular) ──────────
  float elong = (w > d) ? (w / std::max(d, 0.001f)) : (d / std::max(w, 0.001f));
  if (elong > params_.max_elongation) return 0.0f;

  // ── Aspect ratio (h / max_footprint_dim) ─────────────────────────────
  float footprint_max = std::max(w, d);
  float aspect = h / std::max(footprint_max, 0.001f);
  if (aspect < params_.min_aspect_ratio || aspect > params_.max_aspect_ratio) return 0.0f;

  // ── Point density ─────────────────────────────────────────────────────
  float volume = w * d * h;
  float density = static_cast<float>(c.cloud->size()) / std::max(volume, 1e-4f);
  if (density < params_.min_point_density) return 0.0f;

  // ── Score components (each in [0,1]) ──────────────────────────────────

  // 1. Height score — peaked at ~0.35 m (typical small potted plant)
  float ideal_h = 0.35f;
  float h_score = std::exp(-std::pow((h - ideal_h) / 0.3f, 2.0f));

  // 2. Circularity score — w/d should be close to 1.0
  float circ_score = 1.0f - std::min((elong - 1.0f) / (params_.max_elongation - 1.0f), 1.0f);

  // 3. Aspect ratio score — peaked around 1.0
  float ideal_aspect = 1.0f;
  float asp_score = std::exp(-std::pow((aspect - ideal_aspect) / 1.0f, 2.0f));

  // 4. Density score — more points = higher confidence (up to saturation)
  float density_score = std::min(density / 500.0f, 1.0f);

  // ── Weighted combination ──────────────────────────────────────────────
  float score = 0.35f * h_score
              + 0.30f * circ_score
              + 0.20f * asp_score
              + 0.15f * density_score;

  return std::clamp(score, 0.0f, 1.0f);
}

}  // namespace plant_detector
