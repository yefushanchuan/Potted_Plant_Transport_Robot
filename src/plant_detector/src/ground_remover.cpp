#include "plant_detector/ground_remover.hpp"

#include <pcl/filters/passthrough.h>

namespace plant_detector
{

GroundRemover::GroundRemover(const GroundRemoverParams & params)
: params_(params) {}

void GroundRemover::remove(
  const pcl::PointCloud<pcl::PointXYZI>::ConstPtr & cloud_in,
  pcl::PointCloud<pcl::PointXYZI>::Ptr             & cloud_out)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>);

  // 1 remove ground by z
  pcl::PassThrough<pcl::PointXYZI> pass;
  pass.setInputCloud(cloud_in);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(
    static_cast<float>(params_.min_height),
    static_cast<float>(params_.max_height));
  pass.filter(*filtered);

  // 2 range filter
  pass.setInputCloud(filtered);
  pass.setFilterFieldName("x");
  pass.setFilterLimits(
    static_cast<float>(params_.min_range),
    static_cast<float>(params_.max_range));
  pass.filter(*filtered);

  // 3 width filter
  pass.setInputCloud(filtered);
  pass.setFilterFieldName("y");
  pass.setFilterLimits(
    static_cast<float>(params_.min_width),
    static_cast<float>(params_.max_width));
  pass.filter(*filtered);

  cloud_out = filtered;
  cloud_out->header = filtered->header;
}

}  // namespace plant_detector
