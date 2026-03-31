#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Transform.h>
#include <cstring>

namespace robot_base_utils {

class CropBoxComponent : public rclcpp::Node {
public:
    explicit CropBoxComponent(const rclcpp::NodeOptions & options) 
    : Node("cropbox_node", options) {
        x_min_ = this->declare_parameter("min_x", 0.0);
        x_max_ = this->declare_parameter("max_x", 0.2);
        y_min_ = this->declare_parameter("min_y", -0.5);
        y_max_ = this->declare_parameter("max_y", 0.0);
        z_min_ = this->declare_parameter("min_z", 0.0);
        z_max_ = this->declare_parameter("max_z", 0.3);
        negative_ = this->declare_parameter("negative", true);
        target_frame_ = this->declare_parameter("target_frame", "base_link");

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "input", rclcpp::SensorDataQoS(),
            std::bind(&CropBoxComponent::callback, this, std::placeholders::_1));
            
        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "output", rclcpp::SensorDataQoS());
            
        RCLCPP_INFO(this->get_logger(), "Custom TF-Aware CropBox Component Started!");
    }

private:
    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        auto output = std::make_unique<sensor_msgs::msg::PointCloud2>();
        output->header = msg->header;
        output->height = 1;
        output->fields = msg->fields;
        output->is_bigendian = msg->is_bigendian;
        output->point_step = msg->point_step;
        output->is_dense = msg->is_dense;
        output->data.resize(msg->data.size()); 

        int x_offset = -1, y_offset = -1, z_offset = -1;
        for (const auto& field : msg->fields) {
            if (field.name == "x") x_offset = field.offset;
            else if (field.name == "y") y_offset = field.offset;
            else if (field.name == "z") z_offset = field.offset;
        }

        if (x_offset == -1 || y_offset == -1 || z_offset == -1) return;

        tf2::Transform tf2_trans;
        bool use_transform = false;
        
        if (target_frame_ != msg->header.frame_id) {
            try {
                auto transform_stamped = tf_buffer_->lookupTransform(
                    target_frame_, msg->header.frame_id, tf2::TimePointZero);
                tf2::fromMsg(transform_stamped.transform, tf2_trans);
                use_transform = true;
            } catch (const tf2::TransformException & ex) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                    "Waiting for TF from %s to %s...",
                    msg->header.frame_id.c_str(), target_frame_.c_str());
            }
        }

        size_t valid_points = 0;
        const size_t point_step = msg->point_step;
        const size_t num_points = msg->width * msg->height;
        const uint8_t* in_data = msg->data.data();
        uint8_t* out_data = output->data.data();

        for (size_t i = 0; i < num_points; ++i) {
            float x, y, z;
            std::memcpy(&x, in_data + i * point_step + x_offset, sizeof(float));
            std::memcpy(&y, in_data + i * point_step + y_offset, sizeof(float));
            std::memcpy(&z, in_data + i * point_step + z_offset, sizeof(float));

            float check_x = x, check_y = y, check_z = z;

            if (use_transform) {
                tf2::Vector3 pt(x, y, z);
                tf2::Vector3 pt_transformed = tf2_trans * pt;
                check_x = pt_transformed.x();
                check_y = pt_transformed.y();
                check_z = pt_transformed.z();
            }

            bool in_box = (check_x >= x_min_ && check_x <= x_max_ &&
                           check_y >= y_min_ && check_y <= y_max_ &&
                           check_z >= z_min_ && check_z <= z_max_);

            bool keep = negative_ ? !in_box : in_box;

            if (keep) {
                std::memcpy(out_data + valid_points * point_step, 
                            in_data + i * point_step, point_step);
                valid_points++;
            }
        }

        output->width = valid_points;
        output->row_step = valid_points * point_step;
        output->data.resize(output->row_step);
        pub_->publish(std::move(output));
    }

    double x_min_, x_max_, y_min_, y_max_, z_min_, z_max_;
    bool negative_;
    std::string target_frame_;
    
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

} // namespace robot_base_utils

RCLCPP_COMPONENTS_REGISTER_NODE(robot_base_utils::CropBoxComponent)
