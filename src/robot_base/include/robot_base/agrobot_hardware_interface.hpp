#ifndef ROBOTCAR_HARDWARE_INTERFACE_HPP
#define ROBOTCAR_HARDWARE_INTERFACE_HPP

#include <string>
#include <vector>
#include <atomic>
#include <cstdint>
#include <memory>
#include <thread>
#include <mutex>

#include <libserial/SerialPort.h>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "robot_base/visibility_control.h"
#include "robot_base/msg/agrobot_info.hpp"
#include "robot_base/srv/set_control_mode.hpp"
#include "robot_base/action/operate_pot.hpp"

#include "sensor_msgs/msg/battery_state.hpp"

namespace robot_base
{

class AgrobotHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(AgrobotHardwareInterface)

  // ================= 生命周期接口 =================
  ROBOT_BASE_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  ROBOT_BASE_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  ROBOT_BASE_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  // ================= 数据接口 =================
  ROBOT_BASE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  ROBOT_BASE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  ROBOT_BASE_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  ROBOT_BASE_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  // ================= 析构函数 =================
  ROBOT_BASE_PUBLIC
  virtual ~AgrobotHardwareInterface();

private:

  // ================= 硬件通信 =================
  LibSerial::SerialPort serial_port_;
  std::vector<uint8_t> rx_buffer_;

  // ================= ROS接口 =================
  rclcpp::Node::SharedPtr non_realtime_node_;
  rclcpp::Publisher<robot_base::msg::AgrobotInfo>::SharedPtr agrobot_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
  rclcpp::Service<robot_base::srv::SetControlMode>::SharedPtr mode_service_;
  rclcpp_action::Server<robot_base::action::OperatePot>::SharedPtr action_server_;

  // ================= 线程管理 =================
  std::thread node_thread_;
  std::thread action_thread_;
  std::atomic<bool> shutting_down_{false};
  std::mutex action_thread_mutex_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;

  // ================= 状态变量 =================
  double hw_position_left_;
  double hw_velocity_left_;
  double hw_position_right_;
  double hw_velocity_right_;

  // ================= 控制命令 =================
  double hw_command_velocity_left_;
  double hw_command_velocity_right_;

  // ================= 控制标志 =================
  std::atomic<bool> current_action_running_flag_{false};
  std::atomic<bool> is_action_busy_{false};

  std::atomic<uint16_t> hw_mode1_;
  std::atomic<uint16_t> hw_mode2_;
  std::atomic<int8_t> rack_index_cmd_;

  // ================= 协议缓存 =================
  uint8_t tx_seq_id_{0};
  uint16_t last_status_word_{0};
  uint16_t last_health_word_{0};
  uint16_t last_alarm_info_{0};
  uint16_t last_battery_soc_x100_{0};

  double last_robot_vx_{0.0};
  double last_robot_vth_{0.0};

  struct LastTxInfo
  {
    std::string frame_hex;
    int32_t v_linear_mm_s{0};
    int32_t w_angular_mrad_s{0};
    int8_t rack_index;
    uint16_t status_mask{0};
    uint16_t status_value{0};
    uint8_t seq{0};
  };

  struct LastAckInfo
  {
    uint8_t seq{0};
    uint8_t result{0};
  };

  std::mutex protocol_debug_mutex_;
  LastTxInfo last_tx_info_;
  LastAckInfo last_ack_info_;

  // ================= 参数 =================
  std::string serial_port_name_;
  int serial_baud_rate_;

  double wheel_radius_;
  double wheel_separation_;

  // ================= 工具函数 =================
  void resetCommandParams();
};

} // namespace robot_base

#endif
