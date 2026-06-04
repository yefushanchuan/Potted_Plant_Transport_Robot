#include "dual_descriptor_relocalization/dual_descriptor_node.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<dual_descriptor_relocalization::DualDescriptorNode>(
        rclcpp::NodeOptions());

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
