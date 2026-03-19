#pragma once
#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace plant_detector
{

struct ClusterParams
{
  double cluster_tolerance = 0.15;  // m — max distance between points in same cluster
  int    min_cluster_size  = 20;    // points
  int    max_cluster_size  = 5000;  // points
};

struct ClusterResult
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;

  // AABB in sensor frame
  float min_x, max_x;
  float min_y, max_y;
  float min_z, max_z;

  // Centroid
  float cx, cy, cz;

  float width()  const { return max_x - min_x; }
  float depth()  const { return max_y - min_y; }
  float height() const { return max_z - min_z; }
};

class EuclideanCluster
{
public:
  explicit EuclideanCluster(const ClusterParams & params = {});

  /**
   * Segment @cloud_in into individual clusters.
   * Returns a vector of ClusterResult, one per cluster.
   */
  std::vector<ClusterResult> cluster(
    const pcl::PointCloud<pcl::PointXYZI>::ConstPtr & cloud_in);

  void setParams(const ClusterParams & p) { params_ = p; }

private:
  ClusterParams params_;
};

}  // namespace plant_detector
