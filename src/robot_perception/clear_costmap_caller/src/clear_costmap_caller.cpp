#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <nav2_msgs/srv/clear_entire_costmap.hpp>

using namespace std::chrono_literals;

class ClearCostmapCaller final : public rclcpp::Node
{
public:
  ClearCostmapCaller()
  : Node("clear_costmap_caller")
  {
    // 声明可配置参数
    this->declare_parameter<double>("clear_interval_sec", 3.0);
    this->declare_parameter<bool>("clear_global_costmap", true);
    this->declare_parameter<bool>("clear_local_costmap", true);
    this->declare_parameter<double>("service_wait_timeout_sec", 1.0);

    // 获取参数
    clear_interval_sec_ = this->get_parameter("clear_interval_sec").as_double();
    clear_global_ = this->get_parameter("clear_global_costmap").as_bool();
    clear_local_ = this->get_parameter("clear_local_costmap").as_bool();
    service_wait_timeout_sec_ = this->get_parameter("service_wait_timeout_sec").as_double();

    RCLCPP_INFO(this->get_logger(), 
      "Clear costmap caller started: interval=%.1fs, global=%s, local=%s",
      clear_interval_sec_,
      clear_global_ ? "true" : "false",
      clear_local_ ? "true" : "false");

    // 创建服务客户端
    if (clear_global_) {
      global_client_ = this->create_client<nav2_msgs::srv::ClearEntireCostmap>(
        "/global_costmap/clear_entirely_global_costmap");
    }
    if (clear_local_) {
      local_client_ = this->create_client<nav2_msgs::srv::ClearEntireCostmap>(
        "/local_costmap/clear_entirely_local_costmap");
    }

    // 等待服务可用
    auto timeout = std::chrono::duration<double>(service_wait_timeout_sec_);
    bool services_ready = true;
    
    if (clear_global_ && !global_client_->wait_for_service(std::chrono::duration_cast<std::chrono::seconds>(timeout))) {
      RCLCPP_WARN(this->get_logger(), "Global costmap service not available");
      services_ready = false;
    }
    if (clear_local_ && !local_client_->wait_for_service(std::chrono::duration_cast<std::chrono::seconds>(timeout))) {
      RCLCPP_WARN(this->get_logger(), "Local costmap service not available");
      services_ready = false;
    }

    if (!services_ready) {
      RCLCPP_WARN(this->get_logger(), "Some services not available, will retry on timer...");
    }

    // 创建定时器
    auto interval_ms = std::chrono::milliseconds(static_cast<int>(clear_interval_sec_ * 1000));
    timer_ = this->create_wall_timer(interval_ms, std::bind(&ClearCostmapCaller::call_services, this));
  }

private:
  void call_services()
  {
    auto request = std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>();

    // 异步调用全局成本地图清理服务
    if (clear_global_ && global_client_ && global_client_->service_is_ready()) {
      global_client_->async_send_request(
        request, [this](const rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedFuture future) {
          this->service_response_callback("Global Costmap", future);
        });
    }

    // 异步调用局部成本地图清理服务
    if (clear_local_ && local_client_ && local_client_->service_is_ready()) {
      local_client_->async_send_request(
        request, [this](const rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedFuture future) {
          this->service_response_callback("Local Costmap", future);
        });
    }
  }

  void service_response_callback(
    const std::string & costmap_type,
    const rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedFuture & future)
  {
    try {
      auto response = future.get();
      // 成功时静默处理
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to clear %s: %s", costmap_type.c_str(), e.what());
    }
  }

  // 参数
  double clear_interval_sec_;
  bool clear_global_;
  bool clear_local_;
  double service_wait_timeout_sec_;

  // 客户端和定时器
  rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr global_client_;
  rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr local_client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  const auto node = std::make_shared<ClearCostmapCaller>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
