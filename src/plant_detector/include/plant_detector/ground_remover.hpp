#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace plant_detector
{

struct GroundRemoverParams
{
  // RANSAC plane fitting
  double  ransac_distance_thresh = 0.05;  // m — points within this dist belong to ground
  int     ransac_max_iter        = 100;
  double  min_ground_z           = -0.5;  // sensor frame z, ignore points below
  double  max_ground_z           =  0.3;  // rough upper bound for ground seed selection

  // Pass-through filter applied before ground removal
  double  min_range  =  0.3;   // x
  double  max_range  = 15.0;   // x
  double  min_width  = -3.0;   // y
  double  max_width  =  3.0;   // y
  double  min_height = -2.0;   // z
  double  max_height =  3.0;   // z
};

class GroundRemover
{
public:
  explicit GroundRemover(const GroundRemoverParams & params = {});

  /**
   * Remove ground plane from @cloud_in.
   * @cloud_out  – obstacle points (ground removed)
   * @ground_out – extracted ground points (optional, may be nullptr)
   */
  void remove(
    const pcl::PointCloud<pcl::PointXYZI>::ConstPtr & cloud_in,
    pcl::PointCloud<pcl::PointXYZI>::Ptr             & cloud_out,
    pcl::PointCloud<pcl::PointXYZI>::Ptr             * ground_out = nullptr);

  void setParams(const GroundRemoverParams & p) { params_ = p; }

private:
  GroundRemoverParams params_;
};

}  // namespace plant_detector
