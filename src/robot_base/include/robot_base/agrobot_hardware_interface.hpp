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
#include "robot_base/visibility_control.h"
#include "robot_base/msg/agrobot_info.hpp"
#include "robot_base/srv/set_control_mode.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/battery_state.hpp"

namespace robot_base
{

class AgrobotHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(AgrobotHardwareInterface)

  // 初始化硬件接口：解析参数、准备通信节点与初始状态
  ROBOT_BASE_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  // 导出底盘左右轮的状态接口（位置与速度）
  ROBOT_BASE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  // 导出底盘左右轮的速度指令接口
  ROBOT_BASE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // 激活阶段：打开串口并复位状态
  ROBOT_BASE_PUBLIC
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state)
  override;

  // 关闭阶段：停止线程、关闭串口
  ROBOT_BASE_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state)
  override;

  // 从硬件读取一帧数据并更新状态/发布消息
  ROBOT_BASE_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  // 将控制指令封装成协议帧写入硬件
  ROBOT_BASE_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  // Communication with hardware
  LibSerial::SerialPort serial_port_;

  // Node for publishing and services
  rclcpp::Node::SharedPtr non_realtime_node_;
  rclcpp::Publisher<robot_base::msg::AgrobotInfo>::SharedPtr agrobot_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
  rclcpp::Service<robot_base::srv::SetControlMode>::SharedPtr mode_service_;

  // --- New Members for Magnetometer ---
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;
  std::vector<double> hw_magnetic_field_;
  // ------------------------------------
  std::vector<uint8_t> rx_buffer_;

  std::thread node_thread_;

  // Hardware parameters
  std::string serial_port_name_;
  int serial_baud_rate_;

  // Wheel joint states
  double hw_position_left_;
  double hw_velocity_left_;
  double hw_position_right_;
  double hw_velocity_right_;

  // Wheel joint commands
  double hw_command_velocity_left_;
  double hw_command_velocity_right_;

  // Mode commands
  std::atomic<uint16_t> hw_mode1_;
  std::atomic<uint16_t> hw_mode2_;
  std::atomic<uint16_t> z_lift_on_;
  std::atomic<int32_t> z_lift_offset_;
  std::atomic<bool> has_mode_update_{false};

  // 一次性下发浇水量（单位 mL，协议为 uint16）
  std::atomic<uint16_t> pending_spray_volume_ml_{0};
  std::atomic<bool> has_pending_spray_volume_{false};

  // 协议运行态缓存
  uint8_t tx_seq_id_{0};
  uint16_t last_status_word_{0};
  uint16_t last_health_word_{0};
  uint16_t last_alarm_info_{0};
  uint16_t last_battery_soc_x100_{0};
  uint16_t last_bucket_volume_ml_{0};
  double last_robot_vx_{0.0};
  double last_robot_vth_{0.0};
  double last_lift_z_{0.0};

  // 协议联调信息：write() 产生、read() 发布（避免在 write() 中直接发布话题）
  struct LastTxInfo
  {
    std::string frame_hex;                      // 最近一次下发 Command 帧（十六进制字符串）
    int32_t v_linear_mm_s{0};                   // TLV 0x01
    int32_t w_angular_mrad_s{0};                // TLV 0x02
    int32_t z_lift_mm{0};                       // TLV 0x03（1mm 定标，未下发时为 0）
    uint16_t status_mask{0};                    // TLV 0x11（未下发时为 0）
    uint16_t status_value{0};                   // TLV 0x12（未下发时为 0）
    uint16_t spray_volume_ml{0};                // 浇水量（未下发时为 0）
    uint8_t seq{0};                            // Command 帧 SEQ
  };

  struct LastAckInfo
  {
    uint8_t seq{0};                    // ACK 帧 SEQ（通常对应被应答的 Command SEQ）
    uint8_t result{0};                 // TLV 0x30：0=OK，其它=错误码（未收到则为 0）
  };

  std::mutex protocol_debug_mutex_;
  LastTxInfo last_tx_info_;
  LastAckInfo last_ack_info_;

  // Wheel parameters from URDF
  double wheel_radius_;
  double wheel_separation_;

  // 控制类一次性参数初始化/复位（服务触发的开关、浇水量、滑台高度等）
  void resetCommandParams();
};

}

#endif
