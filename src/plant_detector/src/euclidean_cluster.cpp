#include "plant_detector/euclidean_cluster.hpp"

#include <limits>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

namespace plant_detector
{

EuclideanCluster::EuclideanCluster(const ClusterParams & params)
: params_(params) {}

std::vector<ClusterResult> EuclideanCluster::cluster(
  const pcl::PointCloud<pcl::PointXYZI>::ConstPtr & cloud_in)
{
  std::vector<ClusterResult> results;

  if (!cloud_in || cloud_in->empty()) return results;

  // ── KD-tree search structure ───────────────────────────────────────────
  pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(
    new pcl::search::KdTree<pcl::PointXYZI>);
  tree->setInputCloud(cloud_in);

  // ── Euclidean cluster extraction ──────────────────────────────────────
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZI> ece;
  ece.setClusterTolerance(params_.cluster_tolerance);
  ece.setMinClusterSize(params_.min_cluster_size);
  ece.setMaxClusterSize(params_.max_cluster_size);
  ece.setSearchMethod(tree);
  ece.setInputCloud(cloud_in);
  ece.extract(cluster_indices);

  results.reserve(cluster_indices.size());

  for (const auto & indices : cluster_indices) {
    ClusterResult cr;
    cr.cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
    cr.cloud->reserve(indices.indices.size());

    cr.min_x = cr.min_y = cr.min_z = std::numeric_limits<float>::max();
    cr.max_x = cr.max_y = cr.max_z = std::numeric_limits<float>::lowest();

    float sx = 0, sy = 0, sz = 0;

    for (int idx : indices.indices) {
      const auto & pt = (*cloud_in)[idx];
      cr.cloud->push_back(pt);

      cr.min_x = std::min(cr.min_x, pt.x);  cr.max_x = std::max(cr.max_x, pt.x);
      cr.min_y = std::min(cr.min_y, pt.y);  cr.max_y = std::max(cr.max_y, pt.y);
      cr.min_z = std::min(cr.min_z, pt.z);  cr.max_z = std::max(cr.max_z, pt.z);

      sx += pt.x; sy += pt.y; sz += pt.z;
    }

    float n = static_cast<float>(indices.indices.size());
    cr.cx = sx / n;
    cr.cy = sy / n;
    cr.cz = sz / n;

    cr.cloud->header = cloud_in->header;
    results.push_back(std::move(cr));
  }

  return results;
}

}  // namespace plant_detector
