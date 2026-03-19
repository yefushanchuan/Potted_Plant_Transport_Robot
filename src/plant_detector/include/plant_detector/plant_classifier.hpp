#pragma once
#include "plant_detector/euclidean_cluster.hpp"

namespace plant_detector
{

struct ClassifierParams
{
  // --- Size thresholds (metres) ---
  float min_height = 0.10f;   // shortest acceptable plant
  float max_height = 1.20f;   // tallest acceptable plant (including pot)
  float min_width  = 0.05f;
  float max_width  = 0.80f;
  float min_depth  = 0.05f;
  float max_depth  = 0.80f;

  // --- Shape ratios ---
  // Height / max(width, depth)  — plants tend to be taller than wide
  float min_aspect_ratio = 0.4f;
  float max_aspect_ratio = 5.0f;

  // Width / Depth — roughly circular cross-section
  float max_elongation = 3.0f;   // if one side is > 3× the other, reject

  // --- Point density ---
  float min_point_density = 5.0f;   // points / m³  (very loose lower bound)

  // Confidence threshold to publish
  float confidence_threshold = 0.35f;
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
