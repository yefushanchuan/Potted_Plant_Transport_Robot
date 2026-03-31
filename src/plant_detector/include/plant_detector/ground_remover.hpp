#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace plant_detector
{

struct GroundRemoverParams
{
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
    pcl::PointCloud<pcl::PointXYZI>::Ptr             & cloud_out);

  void setParams(const GroundRemoverParams & p) { params_ = p; }

private:
  GroundRemoverParams params_;
};

}  // namespace plant_detector
