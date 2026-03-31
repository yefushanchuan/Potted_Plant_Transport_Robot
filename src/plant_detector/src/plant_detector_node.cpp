#include <memory>
#include <string>
#include <vector>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp" 

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/point.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include "plant_detector/ground_remover.hpp"
#include "plant_detector/euclidean_cluster.hpp"
#include "plant_detector/plant_classifier.hpp"
#include "plant_detector/msg/plant_cluster.hpp"
#include "plant_detector/msg/plant_cluster_array.hpp"

using namespace plant_detector;
using std::placeholders::_1;

namespace plant_detector
{
// ─────────────────────────────────────────────────────────────────────────────
class PlantDetectorNode : public rclcpp::Node
{
public:
  PlantDetectorNode(const rclcpp::NodeOptions & options)
  : Node("plant_detector_node", options)
  {
    declare_and_load_params();

    // ── Ground remover ───────────────────────────────────────────────────
    GroundRemoverParams gp;
    gp.min_range              = get_parameter("ground.min_range").as_double();
    gp.max_range              = get_parameter("ground.max_range").as_double();
    gp.min_height             = get_parameter("ground.min_height").as_double();
    gp.max_height             = get_parameter("ground.max_height").as_double();
    gp.min_width              = get_parameter("ground.min_width").as_double();
    gp.max_width              = get_parameter("ground.max_width").as_double();
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
    clp.max_bottom_z         = static_cast<float>(get_parameter("classifier.max_bottom_z").as_double());
    clp.min_aspect_ratio     = static_cast<float>(get_parameter("classifier.min_aspect_ratio").as_double());
    clp.max_aspect_ratio     = static_cast<float>(get_parameter("classifier.max_aspect_ratio").as_double());
    clp.ideal_height         = static_cast<float>(get_parameter("classifier.ideal_height").as_double());
    clp.height_tolerance     = static_cast<float>(get_parameter("classifier.height_tolerance").as_double());
    clp.ideal_aspect_ratio   = static_cast<float>(get_parameter("classifier.ideal_aspect_ratio").as_double());
    clp.aspect_tolerance     = static_cast<float>(get_parameter("classifier.aspect_tolerance").as_double());
    clp.max_points_for_score = static_cast<float>(get_parameter("classifier.max_points_for_score").as_double());
    clp.weight_height        = static_cast<float>(get_parameter("classifier.weight_height").as_double());
    clp.weight_aspect        = static_cast<float>(get_parameter("classifier.weight_aspect").as_double());
    clp.weight_density       = static_cast<float>(get_parameter("classifier.weight_density").as_double());
    classifier_ = std::make_unique<PlantClassifier>(clp);

    // ── TF listener ───────────────────────────────────────────────────────
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

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
    declare_parameter("ground.min_range",               0.3);
    declare_parameter("ground.max_range",              15.0);
    declare_parameter("ground.min_height",             -2.0);
    declare_parameter("ground.max_height",              3.0);
    declare_parameter("ground.min_width",              -3.0);
    declare_parameter("ground.max_width",               3.0);

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
    declare_parameter("classifier.max_bottom_z",         0.15);
    declare_parameter("classifier.min_aspect_ratio",     0.3);
    declare_parameter("classifier.max_aspect_ratio",     4.0);
    declare_parameter("classifier.ideal_height",         0.25);
    declare_parameter("classifier.height_tolerance",     0.3);
    declare_parameter("classifier.ideal_aspect_ratio",   1.0);
    declare_parameter("classifier.aspect_tolerance",     1.0);
    declare_parameter("classifier.max_points_for_score", 150.0);
    declare_parameter("classifier.weight_height",        0.40);
    declare_parameter("classifier.weight_aspect",        0.30);
    declare_parameter("classifier.weight_density",       0.30);
  }

  // ── Main callback ──────────────────────────────────────────────────────
  void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
  {
    auto t0 = std::chrono::steady_clock::now();

    // TF transform to target frame
    std::string target_frame = "base_footprint";
    sensor_msgs::msg::PointCloud2 cloud_transformed;

    try {
      // 查找从 lidar 坐标系到目标坐标系的变换 (带 50ms 超时等待)
      geometry_msgs::msg::TransformStamped tf_stamped = tf_buffer_->lookupTransform(
        target_frame, 
        msg->header.frame_id,
        msg->header.stamp, // 使用点云的时间戳
        rclcpp::Duration::from_seconds(0.05) 
      );

      // 执行点云坐标转换
      tf2::doTransform(*msg, cloud_transformed, tf_stamped);
    } 
    catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000, 
        "等待 TF 转换失败, 忽略此帧: %s", ex.what()
      );
      return; // 如果 TF 还没准备好，安全退出，不至于闪退
    }

    // Convert ROS → PCL
    pcl::PointCloud<pcl::PointXYZI>::Ptr raw(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(cloud_transformed, *raw);

    if (raw->empty()) return;

    // ── Ground removal ───────────────────────────────────────────────────
    pcl::PointCloud<pcl::PointXYZI>::Ptr obstacle_cloud;
    ground_remover_->remove(raw, obstacle_cloud);

    // Publish filtered cloud (for visualisation / debugging)
    if (pub_filtered_->get_subscription_count() > 0) {
      auto filtered_msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
      pcl::toROSMsg(*obstacle_cloud, *filtered_msg);
      filtered_msg->header = cloud_transformed.header;
      pub_filtered_->publish(std::move(filtered_msg)); // handle ownership transfer with std::move
    }

    // ── Clustering ────────────────────────────────────────────────────────
    auto clusters = clusterer_->cluster(obstacle_cloud);

    // ── Classification & publish ─────────────────────────────────────────
    plant_detector::msg::PlantClusterArray cluster_array;
    cluster_array.header = cloud_transformed.header;

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
      cluster_msg.header      = cloud_transformed.header;
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
        bbox.header    = cloud_transformed.header;
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
        txt.header    = cloud_transformed.header;
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
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr   sub_cloud_;
  rclcpp::Publisher<plant_detector::msg::PlantClusterArray>::SharedPtr pub_clusters_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr          pub_markers_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr                 pub_filtered_;
};

} // namespace plant_detector

RCLCPP_COMPONENTS_REGISTER_NODE(plant_detector::PlantDetectorNode)
