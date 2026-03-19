#include <memory>
#include <string>
#include <vector>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/point.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "plant_detector/ground_remover.hpp"
#include "plant_detector/euclidean_cluster.hpp"
#include "plant_detector/plant_classifier.hpp"
#include "plant_detector/msg/plant_cluster.hpp"
#include "plant_detector/msg/plant_cluster_array.hpp"

using namespace plant_detector;
using std::placeholders::_1;

// ─────────────────────────────────────────────────────────────────────────────
class PlantDetectorNode : public rclcpp::Node
{
public:
  PlantDetectorNode()
  : Node("plant_detector_node")
  {
    declare_and_load_params();

    // ── Ground remover ───────────────────────────────────────────────────
    GroundRemoverParams gp;
    gp.ransac_distance_thresh = get_parameter("ground.ransac_distance_thresh").as_double();
    gp.ransac_max_iter        = get_parameter("ground.ransac_max_iter").as_int();
    gp.min_range              = get_parameter("ground.min_range").as_double();
    gp.max_range              = get_parameter("ground.max_range").as_double();
    gp.min_height             = get_parameter("ground.min_height").as_double();
    gp.max_height             = get_parameter("ground.max_height").as_double();
    ground_remover_ = std::make_unique<GroundRemover>(gp);

    // ── Euclidean cluster ────────────────────────────────────────────────
    ClusterParams cp;
    cp.cluster_tolerance = get_parameter("cluster.tolerance").as_double();
    cp.min_cluster_size  = get_parameter("cluster.min_size").as_int();
    cp.max_cluster_size  = get_parameter("cluster.max_size").as_int();
    clusterer_ = std::make_unique<EuclideanCluster>(cp);

    // ── Plant classifier ─────────────────────────────────────────────────
    ClassifierParams clp;
    clp.min_height           = static_cast<float>(get_parameter("classifier.min_height").as_double());
    clp.max_height           = static_cast<float>(get_parameter("classifier.max_height").as_double());
    clp.min_width            = static_cast<float>(get_parameter("classifier.min_width").as_double());
    clp.max_width            = static_cast<float>(get_parameter("classifier.max_width").as_double());
    clp.min_depth            = static_cast<float>(get_parameter("classifier.min_depth").as_double());
    clp.max_depth            = static_cast<float>(get_parameter("classifier.max_depth").as_double());
    clp.confidence_threshold = static_cast<float>(get_parameter("classifier.confidence_threshold").as_double());
    classifier_ = std::make_unique<PlantClassifier>(clp);

    // ── Publishers ───────────────────────────────────────────────────────
    pub_clusters_ = create_publisher<plant_detector::msg::PlantClusterArray>(
      "plant_detector/clusters", 10);
    pub_markers_  = create_publisher<visualization_msgs::msg::MarkerArray>(
      "plant_detector/markers", 10);
    pub_filtered_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "plant_detector/filtered_cloud", 10);

    // ── Subscriber ───────────────────────────────────────────────────────
    std::string lidar_topic = get_parameter("lidar_topic").as_string();
    sub_cloud_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      lidar_topic, rclcpp::SensorDataQoS(),
      std::bind(&PlantDetectorNode::cloudCallback, this, _1));

    RCLCPP_INFO(get_logger(),
      "PlantDetectorNode started. Listening on: %s", lidar_topic.c_str());
  }

private:
  // ── Parameter declaration ──────────────────────────────────────────────
  void declare_and_load_params()
  {
    declare_parameter("lidar_topic",              "/livox/lidar");

    // Ground remover
    declare_parameter("ground.ransac_distance_thresh", 0.05);
    declare_parameter("ground.ransac_max_iter",         100);
    declare_parameter("ground.min_range",               0.3);
    declare_parameter("ground.max_range",              15.0);
    declare_parameter("ground.min_height",             -2.0);
    declare_parameter("ground.max_height",              3.0);

    // Cluster
    declare_parameter("cluster.tolerance",  0.15);
    declare_parameter("cluster.min_size",    20);
    declare_parameter("cluster.max_size",  5000);

    // Classifier
    declare_parameter("classifier.min_height",           0.10);
    declare_parameter("classifier.max_height",           1.20);
    declare_parameter("classifier.min_width",            0.05);
    declare_parameter("classifier.max_width",            0.80);
    declare_parameter("classifier.min_depth",            0.05);
    declare_parameter("classifier.max_depth",            0.80);
    declare_parameter("classifier.confidence_threshold", 0.35);
  }

  // ── Main callback ──────────────────────────────────────────────────────
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    auto t0 = std::chrono::steady_clock::now();

    // Convert ROS → PCL
    pcl::PointCloud<pcl::PointXYZI>::Ptr raw(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg, *raw);

    if (raw->empty()) return;

    // ── Ground removal ───────────────────────────────────────────────────
    pcl::PointCloud<pcl::PointXYZI>::Ptr obstacle_cloud;
    ground_remover_->remove(raw, obstacle_cloud);

    // Publish filtered cloud (for visualisation / debugging)
    if (pub_filtered_->get_subscription_count() > 0) {
      sensor_msgs::msg::PointCloud2 filtered_msg;
      pcl::toROSMsg(*obstacle_cloud, filtered_msg);
      filtered_msg.header = msg->header;
      pub_filtered_->publish(filtered_msg);
    }

    // ── Clustering ────────────────────────────────────────────────────────
    auto clusters = clusterer_->cluster(obstacle_cloud);

    // ── Classification & publish ─────────────────────────────────────────
    plant_detector::msg::PlantClusterArray cluster_array;
    cluster_array.header = msg->header;

    visualization_msgs::msg::MarkerArray marker_array;
    // Delete old markers first
    {
      visualization_msgs::msg::Marker del;
      del.action = visualization_msgs::msg::Marker::DELETEALL;
      marker_array.markers.push_back(del);
    }

    int plant_id = 0;
    for (const auto & cr : clusters) {
      float conf = classifier_->classify(cr);
      if (conf < get_parameter("classifier.confidence_threshold").as_double()) continue;

      // ── Fill message ───────────────────────────────────────────────────
      plant_detector::msg::PlantCluster cluster_msg;
      cluster_msg.header      = msg->header;
      cluster_msg.id          = plant_id;
      cluster_msg.position.x  = cr.cx;
      cluster_msg.position.y  = cr.cy;
      cluster_msg.position.z  = cr.cz;
      cluster_msg.width       = cr.width();
      cluster_msg.depth       = cr.depth();
      cluster_msg.height      = cr.height();
      cluster_msg.point_count = static_cast<int>(cr.cloud->size());
      cluster_msg.confidence  = conf;
      cluster_array.clusters.push_back(cluster_msg);

      RCLCPP_INFO(get_logger(),
        "[Plant #%d]  pos=(%.2f, %.2f, %.2f) m  "
        "size=(w=%.2f d=%.2f h=%.2f)  conf=%.2f  pts=%d",
        plant_id,
        cr.cx, cr.cy, cr.cz,
        cr.width(), cr.depth(), cr.height(),
        conf,
        static_cast<int>(cr.cloud->size()));

      // ── Bounding-box marker ────────────────────────────────────────────
      {
        visualization_msgs::msg::Marker bbox;
        bbox.header    = msg->header;
        bbox.ns        = "plant_bbox";
        bbox.id        = plant_id;
        bbox.type      = visualization_msgs::msg::Marker::CUBE;
        bbox.action    = visualization_msgs::msg::Marker::ADD;
        bbox.pose.position.x = cr.cx;
        bbox.pose.position.y = cr.cy;
        bbox.pose.position.z = cr.cz;
        bbox.pose.orientation.w = 1.0;
        bbox.scale.x = cr.width();
        bbox.scale.y = cr.depth();
        bbox.scale.z = cr.height();
        // Green, semi-transparent; shade by confidence
        bbox.color.r = 0.0f;
        bbox.color.g = conf;
        bbox.color.b = 0.2f;
        bbox.color.a = 0.35f;
        bbox.lifetime = rclcpp::Duration::from_seconds(0.3);
        marker_array.markers.push_back(bbox);
      }

      // ── Text label ────────────────────────────────────────────────────
      {
        visualization_msgs::msg::Marker txt;
        txt.header    = msg->header;
        txt.ns        = "plant_label";
        txt.id        = plant_id;
        txt.type      = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        txt.action    = visualization_msgs::msg::Marker::ADD;
        txt.pose.position.x = cr.cx;
        txt.pose.position.y = cr.cy;
        txt.pose.position.z = cr.max_z + 0.15f;
        txt.pose.orientation.w = 1.0;
        txt.scale.z = 0.12f;
        txt.color.r = 1.0f; txt.color.g = 1.0f; txt.color.b = 1.0f; txt.color.a = 1.0f;
        // Format: "Plant#0  (x, y) m"
        char buf[128];
        snprintf(buf, sizeof(buf),
          "Plant#%d\n(%.2f, %.2f, %.2f) m\nconf:%.0f%%",
          plant_id, cr.cx, cr.cy, cr.cz, conf * 100.0f);
        txt.text = buf;
        txt.lifetime = rclcpp::Duration::from_seconds(0.3);
        marker_array.markers.push_back(txt);
      }

      ++plant_id;
    }

    pub_clusters_->publish(cluster_array);
    pub_markers_->publish(marker_array);

    auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - t0).count();
    RCLCPP_DEBUG(get_logger(),
      "Pipeline: %zu raw pts → %zu clusters → %d plants  [%ld ms]",
      raw->size(), clusters.size(), plant_id, dt);
  }

  // ── Members ────────────────────────────────────────────────────────────
  std::unique_ptr<GroundRemover>    ground_remover_;
  std::unique_ptr<EuclideanCluster> clusterer_;
  std::unique_ptr<PlantClassifier>  classifier_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr   sub_cloud_;
  rclcpp::Publisher<plant_detector::msg::PlantClusterArray>::SharedPtr pub_clusters_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr          pub_markers_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr                 pub_filtered_;
};

// ── main ─────────────────────────────────────────────────────────────────────
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlantDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
