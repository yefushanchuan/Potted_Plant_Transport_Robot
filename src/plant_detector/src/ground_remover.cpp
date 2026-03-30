#include "plant_detector/ground_remover.hpp"

#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>

namespace plant_detector
{

GroundRemover::GroundRemover(const GroundRemoverParams & params)
: params_(params) {}

void GroundRemover::remove(
  const pcl::PointCloud<pcl::PointXYZI>::ConstPtr & cloud_in,
  pcl::PointCloud<pcl::PointXYZI>::Ptr             & cloud_out,
  pcl::PointCloud<pcl::PointXYZI>::Ptr             * ground_out)
{
  // ── 1. Pass-through filter (range + height) ────────────────────────────
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>);

  // height (z)
  {
    pcl::PassThrough<pcl::PointXYZI> pass;
    pass.setInputCloud(cloud_in);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(
      static_cast<float>(params_.min_height),
      static_cast<float>(params_.max_height));
    pass.filter(*filtered);
  }

  // range (x)  — MID360 looks mostly forward; also filter by distance
  {
    pcl::PassThrough<pcl::PointXYZI> pass;
    pass.setInputCloud(filtered);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(
      static_cast<float>(-params_.max_range),
      static_cast<float>( params_.max_range));
    pass.filter(*filtered);
  }

  if (filtered->empty()) {
    cloud_out = filtered;
    return;
  }

  // ── 2. RANSAC ground-plane segmentation ───────────────────────────────
  pcl::ModelCoefficients::Ptr      coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr           inliers(new pcl::PointIndices);
  pcl::SACSegmentation<pcl::PointXYZI> seg;

  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(params_.ransac_distance_thresh);
  seg.setMaxIterations(params_.ransac_max_iter);

  seg.setInputCloud(filtered);
  seg.segment(*inliers, *coefficients);

  if (inliers->indices.empty()) {
    cloud_out = filtered;
    return;
  }

  // ── 3. Extract non-ground points ──────────────────────────────────────
  pcl::ExtractIndices<pcl::PointXYZI> extractor;
  extractor.setInputCloud(filtered);
  extractor.setIndices(inliers);

  cloud_out.reset(new pcl::PointCloud<pcl::PointXYZI>);
  extractor.setNegative(true);   // keep non-ground
  extractor.filter(*cloud_out);

  if (ground_out) {
    *ground_out = pcl::PointCloud<pcl::PointXYZI>::Ptr(
      new pcl::PointCloud<pcl::PointXYZI>);
    extractor.setNegative(false); // keep ground
    extractor.filter(**ground_out);
  }

  cloud_out->header = filtered->header;
}

}  // namespace plant_detector
