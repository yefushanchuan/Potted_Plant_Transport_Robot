#include "robot_base/agrobot_hardware_interface.hpp"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <algorithm>
#include <array>
#include <limits>
#include <memory>
#include <string>
#include <vector>
#include <stdexcept>
#include <numeric>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "robot_base/uart_protocol.hpp"

namespace robot_base
{

// 控制类一次性参数初始化/复位（服务触发的开关、浇水量、滑台高度等）
void AgrobotHardwareInterface::resetCommandParams()
{
  hw_mode1_.store(0);
  hw_mode2_.store(0);
  z_lift_on_.store(0);
  z_lift_offset_.store(0);
  has_mode_update_.store(false);
  pending_spray_volume_ml_.store(0);
  has_pending_spray_volume_.store(false);
}

namespace
{
// 协议常量
constexpr uint8_t FRAME_HEAD_1 = 0xA0;  // 帧头
constexpr uint8_t FRAME_HEAD_2 = 0x0A;  // 帧头
constexpr uint8_t PROTOCOL_VER = 0x01;  // 协议版本

constexpr uint8_t UART_TYPE_TELEMETRY = 0x01;  // 帧类型：数据
constexpr uint8_t UART_TYPE_COMMAND = 0x81;    // 帧类型：命令
constexpr uint8_t UART_TYPE_ACK = 0x02;        // 帧类型：ACK

constexpr uint8_t TAG_V_LINEAR_MM_S = 0x01;     // int32: 线速度 mm/s
constexpr uint8_t TAG_W_ANGULAR_MRAD_S = 0x02;  // int32: 角速度 mrad/s
constexpr uint8_t TAG_Z_LIFT_MM = 0x03;         // int32: 滑台高度 1mm
constexpr uint8_t TAG_BUCKET_VOLUME_ML = 0x04;  // u16: 水箱余量 mL
constexpr uint8_t TAG_STATUS_MASK = 0x11;       // u16
constexpr uint8_t TAG_STATUS_VALUE = 0x12;      // u16
constexpr uint8_t TAG_STATUS_WORD = 0x13;       // u16
constexpr uint8_t TAG_HEALTH_WORD = 0x14;       // u16
constexpr uint8_t TAG_ALARM_INFO = 0x15;        // u16
constexpr uint8_t TAG_BATT_SOC_X100 = 0x21;     // u16: 电池百分比*100
constexpr uint8_t TAG_SPRAY_VOLUME_ML = 0x22;   // u16: 浇水量 mL（下发；部分固件可能也用于上报桶余量）
constexpr uint8_t TAG_ACK_RESULT = 0x30;        // u8: 0=OK，其它=错误码
constexpr uint8_t TAG_ACK_REQUEST = 0x40;       // u8: 请求 ACK

constexpr size_t FRAME_FIXED_HEADER_LEN = 7; // H1 H2 VER SEQ TYPE LEN_L LEN_H
constexpr size_t FRAME_MIN_SIZE = FRAME_FIXED_HEADER_LEN + 2; // + CRC，payload 为 0 时的最小长度
constexpr uint16_t MAX_PAYLOAD_LEN = 512; // 单帧最大负载长度（容错上限，防止错误 LEN 导致缓存膨胀）

// 将字节流编码为大写十六进制字符串（字节之间用空格分隔）
std::string bytesToHexString(const uint8_t * data, size_t len)
{
  static constexpr char HEX[] = "0123456789ABCDEF";
  std::string out;
  if (data == nullptr || len == 0) {
    return out;
  }
  out.reserve(len * 3 - 1);
  for (size_t i = 0; i < len; i++) {
    const uint8_t b = data[i];
    out.push_back(HEX[(b >> 4) & 0x0F]);
    out.push_back(HEX[(b >> 0) & 0x0F]);
    if (i + 1 < len) {out.push_back(' ');}
  }
  return out;
}

// alarm_info：电机报警码（4bit）→中文描述（0 表示无报警）
const char * motorAlarmCodeToText(uint8_t code)
{
  switch (code) {
    case 0x01: return "编码器故障 ABZ 报警";
    case 0x02: return "编码器故障 UVW 报警";
    case 0x03: return "位置超差";
    case 0x04: return "失速";
    case 0x05: return "电流采样故障";
    case 0x06: return "过载";
    case 0x07: return "欠压";
    case 0x08: return "过压";
    case 0x09: return "过流";
    case 0x0A: return "放电瞬时功率过大";
    case 0x0B: return "放电平均功率大";
    case 0x0C: return "参数读写异常";
    case 0x0D: return "输入口功能定义重复";
    case 0x0E: return "通讯看门狗触发";
    case 0x0F: return "电机过温报警";
    default:   return "未知电机报警码";
  }
}

} // namespace

// 初始化硬件接口：读取参数、创建节点与通信对象
hardware_interface::CallbackReturn AgrobotHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  // 调用父类的初始化方法
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // 从硬件参数中读取串口名称和波特率
  serial_port_name_ = info_.hardware_parameters["serial_port_name"];
  serial_baud_rate_ = std::stoi(info_.hardware_parameters["serial_baud_rate"]);
  wheel_radius_ = std::stod(info_.hardware_parameters["wheel_radius"]);
  wheel_separation_ = std::stod(info_.hardware_parameters["wheel_separation"]);

  // 初始化硬件状态变量
  hw_position_left_ = 0.0;
  hw_velocity_left_ = 0.0;
  hw_position_right_ = 0.0;
  hw_velocity_right_ = 0.0;
  hw_command_velocity_left_ = 0.0;
  hw_command_velocity_right_ = 0.0;
  resetCommandParams();
  tx_seq_id_ = 0;
  last_health_word_ = 0;
  last_alarm_info_ = 0;
  last_battery_soc_x100_ = 0;
  last_bucket_volume_ml_ = 0;
  last_robot_vx_ = 0.0;
  last_robot_vth_ = 0.0;
  last_lift_z_ = 0.0;
  // resetCommandParams() 已清理控制类一次性标记，这里仅保留 Telemetry 缓存初始化

  // 创建非实时节点
  non_realtime_node_ = rclcpp::Node::make_shared(info_.name + "_non_realtime_node");
  // 创建车轮信息发布者
  agrobot_info_pub_ = non_realtime_node_->create_publisher<robot_base::msg::AgrobotInfo>(
    "/agrobot_base_info", rclcpp::SystemDefaultsQoS());
//   // 创建磁力计数据发布者
//   mag_pub_ = non_realtime_node_->create_publisher<sensor_msgs::msg::MagneticField>(
//     "/wit/mag",
//     rclcpp::SystemDefaultsQoS());

  battery_pub_ = non_realtime_node_->create_publisher<sensor_msgs::msg::BatteryState>(
    "/battery_state", rclcpp::SystemDefaultsQoS());

  // 创建设置控制模式的服务回调
  auto service_callback =
    [this](const std::shared_ptr<robot_base::srv::SetControlMode::Request> request,
      std::shared_ptr<robot_base::srv::SetControlMode::Response> response)
    {
      // 根据协议位定义将控制命令转换为 status_mask/status_value（mask 指定需要修改的位）
      uint16_t status_mask = 0;
      uint16_t status_value = 0;

      constexpr uint16_t RIGHT_WHEEL_ENABLE_SHIFT = 0;           // bit0-bit1
      constexpr uint16_t RIGHT_WHEEL_RESET_BIT = 2;              // bit2
      constexpr uint16_t LEFT_WHEEL_ENABLE_SHIFT = 3;            // bit3-bit4
      constexpr uint16_t LEFT_WHEEL_RESET_BIT = 5;               // bit5
      constexpr uint16_t SLIDE_TABLE_ENABLE_SHIFT = 6;           // bit6-bit7（2bit：0默认 1使能 2失能）
      constexpr uint16_t SPRAY_ON_BIT = 12;                      // bit12
      constexpr uint16_t DRAIN_ON_BIT = 13;                      // bit13
      constexpr uint16_t CHARGING_ON_BIT = 14;                   // bit14
      constexpr uint16_t GRIPPER_CLOSED_BIT = 15;                // bit15

      auto applyTriStateBit = [&](uint16_t bit, uint8_t cmd, const char * name) -> bool {
          if (cmd == 0) {
            return true;
          }
          if (cmd != 1 && cmd != 2) {
            RCLCPP_ERROR(
              rclcpp::get_logger("AgrobotHardwareInterface"),
              "%s_cmd 取值非法：%u（期望 0/1/2）",
              name,
              static_cast<unsigned>(cmd));
            return false;
          }
          status_mask |= static_cast<uint16_t>(1u << bit);
          if (cmd == 1) {
            status_value |= static_cast<uint16_t>(1u << bit);
          }
          return true;
        };

      auto applyEnable2Bits = [&](uint16_t shift, uint8_t cmd, const char * name) -> bool {
          if (cmd == 0) {
            return true;
          }
          if (cmd != 1 && cmd != 2) {
            RCLCPP_ERROR(
              rclcpp::get_logger("AgrobotHardwareInterface"),
              "%s_cmd 取值非法：%u（期望 0/1/2）",
              name,
              static_cast<unsigned>(cmd));
            return false;
          }
          status_mask |= static_cast<uint16_t>(0x03u << shift);
          status_value |= static_cast<uint16_t>(
            (static_cast<uint16_t>(cmd) & 0x03u) << shift);
          return true;
        };

      auto applyResetBit = [&](uint16_t bit, uint8_t cmd, const char * name) -> bool {
          if (cmd == 0) {
            return true;
          }
          if (cmd != 1) {
            RCLCPP_ERROR(
              rclcpp::get_logger("AgrobotHardwareInterface"),
              "%s_cmd 取值非法：%u（期望 0/1）",
              name,
              static_cast<unsigned>(cmd));
            return false;
          }
          status_mask |= static_cast<uint16_t>(1u << bit);
          status_value |= static_cast<uint16_t>(1u << bit);
          return true;
        };

      // 电机控制
      if (!applyEnable2Bits(
          RIGHT_WHEEL_ENABLE_SHIFT, request->right_wheel_enable_cmd, "right_wheel_enable"))
      {
        this->resetCommandParams();
        response->success = false;
        return;
      }
      if (!applyEnable2Bits(
          LEFT_WHEEL_ENABLE_SHIFT, request->left_wheel_enable_cmd, "left_wheel_enable"))
      {
        this->resetCommandParams();
        response->success = false;
        return;
      }
      if (!applyResetBit(
          RIGHT_WHEEL_RESET_BIT, request->right_wheel_reset_cmd, "right_wheel_reset"))
      {
        this->resetCommandParams();
        response->success = false;
        return;
      }
      if (!applyResetBit(LEFT_WHEEL_RESET_BIT, request->left_wheel_reset_cmd, "left_wheel_reset")) {
        this->resetCommandParams();
        response->success = false;
        return;
      }

      // 喷水/排水/充电/夹爪（按需修改）
      if (!applyTriStateBit(SPRAY_ON_BIT, request->spray_on_cmd, "spray_on")) {
        this->resetCommandParams();
        response->success = false;
        return;
      }
      if (!applyTriStateBit(DRAIN_ON_BIT, request->drain_on_cmd, "drain_on")) {
        this->resetCommandParams();
        response->success = false;
        return;
      }
      if (!applyTriStateBit(CHARGING_ON_BIT, request->charging_on_cmd, "charging_on")) {
        this->resetCommandParams();
        response->success = false;
        return;
      }
      if (!applyTriStateBit(GRIPPER_CLOSED_BIT, request->gripper_closed_cmd, "gripper_closed")) {
        this->resetCommandParams();
        response->success = false;
        return;
      }

      // 一次性下发浇水量（允许为 0，因此使用 spray_volume_set 控制是否下发）
      if (request->spray_volume_set != 0) {
        this->pending_spray_volume_ml_.store(request->spray_volume_ml);
        this->has_pending_spray_volume_.store(true);
      }

      // 一次性下发滑台高度（单位 mm）
      if (request->z_lift_set != 0) {
        this->z_lift_on_.store(1);
        this->z_lift_offset_.store(request->z_lift_mm);
        if (z_lift_offset_.load() < 0 || z_lift_offset_.load() > 230) {
          RCLCPP_ERROR(rclcpp::get_logger("AgrobotHardwareInterface"), "z_lift_mm out of range");
          this->resetCommandParams();
          response->success = false;
          return;
        }

        // 按协议建议：下发滑台高度时，确保滑台电机使能（bit6-bit7 = 1）
        status_mask |= static_cast<uint16_t>(0x03u << SLIDE_TABLE_ENABLE_SHIFT);
        status_value |= static_cast<uint16_t>(0x01u << SLIDE_TABLE_ENABLE_SHIFT);
      } else {
        this->z_lift_on_.store(0);
        this->z_lift_offset_.store(0);
      }

      // 更新状态开关（一次性下发，发送后清空标记）
      this->hw_mode1_.store(status_mask);
      this->hw_mode2_.store(status_value);

      // 只要有需要一次性下发的控制内容，就置位发送标记
      this->has_mode_update_.store(
        (status_mask != 0) ||
        (this->z_lift_on_.load() != 0) ||
        (this->has_pending_spray_volume_.load()));

      RCLCPP_INFO(
        rclcpp::get_logger("AgrobotHardwareInterface"),
        "SetControlMode: status_mask=0x%04X status_value=0x%04X z_lift_set=%u z_lift_mm=%d spray_volume_set=%u spray_volume_ml=%u",
        status_mask,
        status_value,
        static_cast<unsigned>(request->z_lift_set),
        request->z_lift_mm,
        static_cast<unsigned>(request->spray_volume_set),
        static_cast<unsigned>(request->spray_volume_ml));

      response->success = true;
    };
  // 创建设置控制模式的服务
  mode_service_ = non_realtime_node_->create_service<robot_base::srv::SetControlMode>(
    "/set_agrobotbase_ctrlmode", service_callback);

  // 创建并启动节点线程
  node_thread_ = std::thread([this]() {rclcpp::spin(this->non_realtime_node_);});

  // 打印初始化成功信息
  RCLCPP_INFO(rclcpp::get_logger("AgrobotHardwareInterface"), "on_init 成功");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// 暴露左右轮位置、速度状态接口给控制器
std::vector<hardware_interface::StateInterface> AgrobotHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // 导出左轮的状态接口
  state_interfaces.emplace_back(
    hardware_interface::StateInterface(
      info_.joints[0].name, "position",
      &hw_position_left_));
  state_interfaces.emplace_back(
    hardware_interface::StateInterface(
      info_.joints[0].name, "velocity",
      &hw_velocity_left_));

  // 导出右轮的状态接口
  state_interfaces.emplace_back(
    hardware_interface::StateInterface(
      info_.joints[1].name, "position",
      &hw_position_right_));
  state_interfaces.emplace_back(
    hardware_interface::StateInterface(
      info_.joints[1].name, "velocity",
      &hw_velocity_right_));

  return state_interfaces;
}

// 暴露左右轮速度指令接口给控制器
std::vector<hardware_interface::CommandInterface> AgrobotHardwareInterface::
export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // 导出左轮的命令接口
  command_interfaces.emplace_back(
    hardware_interface::CommandInterface(
      info_.joints[0].name,
      "velocity", &hw_command_velocity_left_));
  // 导出右轮的命令接口
  command_interfaces.emplace_back(
    hardware_interface::CommandInterface(
      info_.joints[1].name,
      "velocity", &hw_command_velocity_right_));
  return command_interfaces;
}
// 激活阶段：打开串口并准备通信
hardware_interface::CallbackReturn AgrobotHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  hw_position_left_ = 0.0;
  hw_position_right_ = 0.0;
  // 打印激活信息
  RCLCPP_INFO(rclcpp::get_logger("AgrobotHardwareInterface"), "Activating ...please wait...");
  try {
    // 打开主串口
    serial_port_.Open(serial_port_name_);
    LibSerial::BaudRate baud_rate;
    // 根据波特率设置串口参数
    switch (serial_baud_rate_) {
      case 9600: baud_rate = LibSerial::BaudRate::BAUD_9600; break;
      case 19200: baud_rate = LibSerial::BaudRate::BAUD_19200; break;
      case 38400: baud_rate = LibSerial::BaudRate::BAUD_38400; break;
      case 57600: baud_rate = LibSerial::BaudRate::BAUD_57600; break;
      case 115200: baud_rate = LibSerial::BaudRate::BAUD_115200; break;
      case 230400: baud_rate = LibSerial::BaudRate::BAUD_230400; break;
      default:
        // 如果波特率不支持，则打印错误信息并使用默认值
        RCLCPP_ERROR(
          rclcpp::get_logger("AgrobotHardwareInterface"),
          "Unsupported baud rate: %d. Using 115200 as default.", serial_baud_rate_);
        baud_rate = LibSerial::BaudRate::BAUD_115200;
        break;
    }
    // 设置串口参数
    serial_port_.SetBaudRate(baud_rate);
    serial_port_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
    serial_port_.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
    serial_port_.SetParity(LibSerial::Parity::PARITY_NONE);
    serial_port_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
    serial_port_.FlushIOBuffers();
    rx_buffer_.clear();

    // 打印串口打开成功信息
    RCLCPP_INFO(
      rclcpp::get_logger(
        "AgrobotHardwareInterface"), "✅ Main serial port opened successfully: %s",
      serial_port_name_.c_str());
  } catch (const LibSerial::OpenFailed & e) {
    // 如果打开串口失败，则打印错误信息并返回错误
    RCLCPP_FATAL(
      rclcpp::get_logger(
        "AgrobotHardwareInterface"), "❌ Failed to open serial port %s: %s",
      serial_port_name_.c_str(), e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // 打印激活成功的信息
  RCLCPP_INFO(rclcpp::get_logger("AgrobotHardwareInterface"), "Successfully activated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// 停用阶段：停止线程、关闭串口，释放资源
hardware_interface::CallbackReturn AgrobotHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  hw_position_left_ = 0.0;
  hw_position_right_ = 0.0;
  RCLCPP_INFO(rclcpp::get_logger("AgrobotHardwareInterface"), "Deactivating ...please wait...");

  // Close serial ports
  if (serial_port_.IsOpen()) {
    serial_port_.Close();
    RCLCPP_INFO(rclcpp::get_logger("AgrobotHardwareInterface"), "Successfully close stm32_serial!");
  }
  if (node_thread_.joinable()) {
    node_thread_.join();
  }
  RCLCPP_INFO(rclcpp::get_logger("AgrobotHardwareInterface"), "Successfully deactivated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// 读取底盘反馈：解析串口缓存、更新里程并发布状态
hardware_interface::return_type AgrobotHardwareInterface::read(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  if (serial_port_.IsDataAvailable()) {
    std::vector<uint8_t> read_buffer;
    try {
      // 一次多读一些数据，避免残包
      size_t available = serial_port_.GetNumberOfBytesAvailable();
      if (available > 0) {
        serial_port_.Read(read_buffer, available);
      }

      if (!read_buffer.empty()) {
        // 将新数据加入缓存
        rx_buffer_.insert(rx_buffer_.end(), read_buffer.begin(), read_buffer.end());

        // 循环解析缓存
        while (rx_buffer_.size() >= FRAME_MIN_SIZE) {
          // 帧头对齐
          if (rx_buffer_[0] != FRAME_HEAD_1 || rx_buffer_[1] != FRAME_HEAD_2) {
            rx_buffer_.erase(rx_buffer_.begin());
            RCLCPP_WARN(
              rclcpp::get_logger("AgrobotHardwareInterface"),
              "帧头对齐失败，丢弃一个字节"
            );
            continue;
          }
          RCLCPP_DEBUG(
            rclcpp::get_logger("AgrobotHardwareInterface"),
            "帧头对齐成功"
          );

          // 固定头长度不足，等待更多数据
          if (rx_buffer_.size() < FRAME_FIXED_HEADER_LEN) {
            RCLCPP_WARN(
              rclcpp::get_logger("AgrobotHardwareInterface"),
              "固定帧头长度不足，等待更多数据"
            );
            break;
          }
          RCLCPP_DEBUG(
            rclcpp::get_logger("AgrobotHardwareInterface"),
            "固定帧头长度足够，继续解析"
          );

          uint8_t version = rx_buffer_[2];
          uint8_t seq_id = rx_buffer_[3];
          uint8_t type = rx_buffer_[4];
          uint16_t payload_len = static_cast<uint16_t>(rx_buffer_[5]) |
            (static_cast<uint16_t>(rx_buffer_[6]) << 8);
          RCLCPP_DEBUG(
            rclcpp::get_logger("AgrobotHardwareInterface"),
            "解析到数据帧: version=%u seq_id=%u type=0x%02X payload_len=%u",
            static_cast<unsigned>(version),
            static_cast<unsigned>(seq_id),
            static_cast<unsigned>(type),
            static_cast<unsigned>(payload_len)
          );
          if (payload_len > MAX_PAYLOAD_LEN) {
            RCLCPP_WARN(
              rclcpp::get_logger("AgrobotHardwareInterface"),
              "PAYLOAD 长度异常：%u（上限 %u），丢弃帧头重新同步",
              static_cast<unsigned>(payload_len),
              static_cast<unsigned>(MAX_PAYLOAD_LEN));
            rx_buffer_.erase(rx_buffer_.begin());
            continue;
          }
          size_t frame_len = FRAME_FIXED_HEADER_LEN + payload_len + 2;           // + CRC
          RCLCPP_DEBUG(rclcpp::get_logger("AgrobotHardwareInterface"),
                       "收到 %u 长度的数据帧, 负载长度为 %u",
                       static_cast<unsigned>(frame_len),
                       static_cast<unsigned>(payload_len));

          // 数据未收齐，等待下一轮
          if (rx_buffer_.size() < frame_len) {
            break;
          }

          RCLCPP_DEBUG(rclcpp::get_logger("AgrobotHardwareInterface"),
                       "已找到%ld字节的数据，开始匹配协议版本和CRC校验", rx_buffer_.size());
          // const uint8_t *frame_data = rx_buffer_.data();
          // print_hex(frame_data, frame_len);

          if (version != PROTOCOL_VER) {
            RCLCPP_WARN(
              rclcpp::get_logger("AgrobotHardwareInterface"),
              "协议版本不匹配，收到 %u", static_cast<unsigned>(version));
            rx_buffer_.erase(rx_buffer_.begin());
            continue;
          }

          // CRC 按小端存储，计算范围从 VER 开始（不含帧头）
          uint16_t crc_in_frame =
            static_cast<uint16_t>(rx_buffer_[FRAME_FIXED_HEADER_LEN + payload_len]) |
            (static_cast<uint16_t>(rx_buffer_[FRAME_FIXED_HEADER_LEN + payload_len + 1]) << 8);
          uint16_t crc_calc = uart::crc16_modbus(
            rx_buffer_.data() + 2,
            static_cast<uint16_t>((FRAME_FIXED_HEADER_LEN - 2) + payload_len));
          if (crc_calc != crc_in_frame) {
            RCLCPP_WARN(
              rclcpp::get_logger("AgrobotHardwareInterface"),
              "CRC 校验失败，收到 0x%04X 实得 0x%04X, frame_id:0x%02X", crc_in_frame, crc_calc, seq_id);
            RCLCPP_WARN(
              rclcpp::get_logger("AgrobotHardwareInterface"),
              "错误数据帧内容: %s",
              bytesToHexString(rx_buffer_.data(), frame_len).c_str());
            rx_buffer_.erase(rx_buffer_.begin());
            continue;
          }
          RCLCPP_DEBUG(rclcpp::get_logger("AgrobotHardwareInterface"),
                       "CRC 校验成功，开始解析数据帧...");

          const uint8_t * payload = rx_buffer_.data() + FRAME_FIXED_HEADER_LEN;           // 负载数据
          const std::string rx_frame_hex = bytesToHexString(rx_buffer_.data(), frame_len);
          bool parsed = false;
          if (type == UART_TYPE_TELEMETRY) {
            RCLCPP_DEBUG(rclcpp::get_logger("AgrobotHardwareInterface"), "收到数据类型为 UART_TYPE_TELEMETRY");
            // 使用上一帧的数值做默认值，缺失标签时维持上一次状态
            double robot_vx = last_robot_vx_;
            double robot_vth = last_robot_vth_;
            double lift_z_ = last_lift_z_;
            uint16_t health_word = last_health_word_;
            uint16_t alarm_info = last_alarm_info_;
            uint16_t battery_soc_x100 = last_battery_soc_x100_;
            uint16_t bucket_volume = last_bucket_volume_ml_;

            size_t offset = 0;  // 负载偏移
            bool tlv_error = false;
            while (offset + 2 <= payload_len) {
              uint8_t tag = payload[offset];
              uint8_t len = payload[offset + 1];
              if (offset + 2 + len > payload_len) {
                tlv_error = true;
                RCLCPP_WARN(
                  rclcpp::get_logger("AgrobotHardwareInterface"),
                  "TLV 解析错误：标签 0x%02X 长度 %u 超出负载边界 %u",
                  static_cast<unsigned>(tag),
                  static_cast<unsigned>(len),
                  static_cast<unsigned>(payload_len));
                break;
              }
              RCLCPP_DEBUG(
                rclcpp::get_logger("AgrobotHardwareInterface"),
                "解析标签 0x%02X 长度 %u",
                static_cast<unsigned>(tag),
                static_cast<unsigned>(len));
              const uint8_t * data_ptr = payload + offset + 2;
              switch (tag) {
                case TAG_V_LINEAR_MM_S:
                  if (len != 4) {tlv_error = true; break;}
                  robot_vx = static_cast<double>(uart::readInt32LE(data_ptr)) / 1000.0;
                  break;
                case TAG_W_ANGULAR_MRAD_S:
                  if (len != 4) {tlv_error = true; break;}
                  robot_vth = static_cast<double>(uart::readInt32LE(data_ptr)) / 1000.0;
                  break;
                case TAG_Z_LIFT_MM:
                  if (len != 4) {tlv_error = true; break;}
                  lift_z_ = static_cast<double>(uart::readInt32LE(data_ptr));
                  break;
                case TAG_HEALTH_WORD:
                  if (len != 2) {tlv_error = true; break;}
                  health_word = uart::readUint16LE(data_ptr);
                  break;
                case TAG_ALARM_INFO:
                  if (len != 2) {tlv_error = true; break;}
                  alarm_info = uart::readUint16LE(data_ptr);
                  break;
                case TAG_BATT_SOC_X100:
                  if (len != 2) {tlv_error = true; break;}
                  battery_soc_x100 = uart::readUint16LE(data_ptr);
                  break;
                case TAG_BUCKET_VOLUME_ML:
                  if (len != 2) {tlv_error = true; break;}
                  bucket_volume = uart::readUint16LE(data_ptr);
                  break;
                case TAG_SPRAY_VOLUME_ML:
                  if (len != 2) {tlv_error = true; break;}
                  bucket_volume = uart::readUint16LE(data_ptr);
                  break;
                default:
                  // 未知标签按长度跳过，保证向后兼容
                  break;
              }
              offset += (2 + len);
            }
            if (tlv_error) {
              RCLCPP_WARN(
                rclcpp::get_logger("AgrobotHardwareInterface"),
                "TLV 解析过程中出现错误");
            }
            else {
              RCLCPP_DEBUG(rclcpp::get_logger("AgrobotHardwareInterface"),"tlv_error state: %d", tlv_error);
            }

            if (!tlv_error) {
              // 更新左右轮速度/位置
              hw_velocity_left_ = (robot_vx - robot_vth * wheel_separation_ / 2.0) / wheel_radius_;
              hw_velocity_right_ = (robot_vx + robot_vth * wheel_separation_ / 2.0) / wheel_radius_;
              hw_position_left_ += hw_velocity_left_ * period.seconds();
              hw_position_right_ += hw_velocity_right_ * period.seconds();

              // 发布底盘信息
              auto msg = std::make_unique<robot_base::msg::AgrobotInfo>();
              msg->speed_x = robot_vx;
              msg->speed_z = robot_vth;
              msg->z_lift = lift_z_;
              msg->power = static_cast<float>(battery_soc_x100) / 100.0f;
              msg->bucket_volume = bucket_volume;
              msg->flag_1 = static_cast<uint8_t>(0x00);

              // 原始协议字段
              msg->protocol_version = version;
              msg->rx_seq = seq_id;
              msg->rx_type = type;
              msg->rx_payload_len = payload_len;
              msg->health_word_raw = health_word;
              msg->alarm_info_raw = alarm_info;
              msg->battery_soc_x100 = battery_soc_x100;
              msg->rx_frame_hex = rx_frame_hex;

              // 复制最近一次下发帧/命令与 ACK 信息
              {
                std::lock_guard<std::mutex> lock(protocol_debug_mutex_);
                msg->tx_frame_hex = last_tx_info_.frame_hex;
                msg->cmd_v_linear_mm_s = last_tx_info_.v_linear_mm_s;
                msg->cmd_w_angular_mrad_s = last_tx_info_.w_angular_mrad_s;
                msg->cmd_z_lift_mm = last_tx_info_.z_lift_mm;
                msg->cmd_status_mask = last_tx_info_.status_mask;
                msg->cmd_status_value = last_tx_info_.status_value;
                msg->cmd_spray_volume_ml = last_tx_info_.spray_volume_ml;
                msg->ack_seq = last_ack_info_.seq;
                msg->ack_result = last_ack_info_.result;
              }
              // health_word 映射（按新协议位定义）
              msg->estop_state = (health_word & (1u << 0)) != 0;                       // emergency_active
              msg->right_wheel_enabled = (health_word & (1u << 1)) != 0;               // right_motor_enable
              msg->right_wheel_alarm = (health_word & (1u << 2)) != 0;                 // right_motor_alarm
              msg->left_wheel_enabled = (health_word & (1u << 3)) != 0;                // left_motor_enable
              msg->left_wheel_alarm = (health_word & (1u << 4)) != 0;                  // left_motor_alarm
              msg->slide_table_enabled = (health_word & (1u << 5)) != 0;               // slide_table_enable
              msg->slide_table_alarm = (health_word & (1u << 6)) != 0;                  // slide_table_alarm
              msg->battery_comm_fault = (health_word & (1u << 8)) != 0;                // battery_comm_fault
              msg->remote_connected = (health_word & (1u << 9)) != 0;                  // bus_connected（遥控器连接）
              msg->charging_on = (health_word & (1u << 12)) != 0;                       // bit12
              msg->spray_on = (health_word & (1u << 13)) != 0;                          // bit13
              msg->drain_on = (health_word & (1u << 14)) != 0;                          // bit14
              msg->gripper_closed = (health_word & (1u << 15)) != 0;                    // bit15
              msg->motor_alarm = msg->right_wheel_alarm || msg->left_wheel_alarm || (alarm_info != 0);

              // alarm_list：拼接为字符串列表（优先使用 alarm_info 的细分报警码）
              std::vector<std::string> alarms;
              const uint8_t right_alarm_code = static_cast<uint8_t>(alarm_info & 0x0F);
              const uint8_t left_alarm_code = static_cast<uint8_t>((alarm_info >> 4) & 0x0F);

              if (right_alarm_code != 0) {
                alarms.emplace_back(std::string("右轮电机：") + motorAlarmCodeToText(right_alarm_code));
              } else if (msg->right_wheel_alarm) {
                alarms.emplace_back("右轮电机：报警");
              }

              if (left_alarm_code != 0) {
                alarms.emplace_back(std::string("左轮电机：") + motorAlarmCodeToText(left_alarm_code));
              } else if (msg->left_wheel_alarm) {
                alarms.emplace_back("左轮电机：报警");
              }

              if (msg->estop_state) {
                alarms.emplace_back("急停触发");
              }
              if (msg->battery_comm_fault) {
                alarms.emplace_back("电池通信故障");
              }

              msg->alarm_list = std::move(alarms);

              auto battery_msg = sensor_msgs::msg::BatteryState();
              battery_msg.header.stamp = time;
              battery_msg.header.frame_id = "base_link";
              battery_msg.voltage = 0.0;
              battery_msg.current = msg->charging_on ? 1.0 : -1.0;
              battery_msg.percentage = std::clamp(
                static_cast<float>(battery_soc_x100) / 10000.0f,
                0.0f, 1.0f);
              battery_msg.present = msg->battery_comm_fault ? false : true;
              battery_msg.power_supply_health =
                sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_GOOD;
              battery_msg.power_supply_technology =
                sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;
              battery_msg.power_supply_status = msg->charging_on ?
                sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING :
                sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;

              agrobot_info_pub_->publish(std::move(msg));
              battery_pub_->publish(battery_msg);
              // RCLCPP_DEBUG(rclcpp::get_logger("AgrobotHardwareInterface"),"battery soc: %.4f", battery_msg.percentage);

              last_robot_vx_ = robot_vx;
              last_robot_vth_ = robot_vth;
              last_lift_z_ = lift_z_;
              last_health_word_ = health_word;
              last_alarm_info_ = alarm_info;
              last_battery_soc_x100_ = battery_soc_x100;
              last_bucket_volume_ml_ = bucket_volume;

              parsed = true;
            }
          } else if (type == UART_TYPE_ACK) {
            uint8_t ack_result = 0;
            size_t offset = 0;
            bool tlv_error = false;
            while (offset + 2 <= payload_len) {
              const uint8_t tag = payload[offset];
              const uint8_t len = payload[offset + 1];
              if (offset + 2 + len > payload_len) {
                tlv_error = true;
                break;
              }
              const uint8_t * data_ptr = payload + offset + 2;
              if (tag == TAG_ACK_RESULT) {
                if (len != 1) {tlv_error = true; break;}
                ack_result = data_ptr[0];
              }
              offset += (2 + len);
            }

            if (!tlv_error) {
              std::lock_guard<std::mutex> lock(protocol_debug_mutex_);
              last_ack_info_.seq = seq_id;
              last_ack_info_.result = ack_result;
              parsed = true;
            }
          }

          if (parsed) {
            rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + frame_len);
          } else {
            // 解析失败丢弃头部，避免卡住
            rx_buffer_.erase(rx_buffer_.begin());
          }

          (void)seq_id;           // 当前未使用序列号，仅保持接口兼容
        }
      }
    } catch (const LibSerial::ReadTimeout &) {
      RCLCPP_WARN(rclcpp::get_logger("AgrobotHardwareInterface"), "stm32 read timeout");
    }
  }

  return hardware_interface::return_type::OK;
}


hardware_interface::return_type AgrobotHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  double vx = (hw_command_velocity_right_ + hw_command_velocity_left_) * wheel_radius_ / 2.0;
  double vth = (hw_command_velocity_right_ - hw_command_velocity_left_) * wheel_radius_ /
    wheel_separation_;

  // 按新协议组装 Command TLV
  std::vector<uint8_t> payload;
  const int32_t v_linear_mm_s = static_cast<int32_t>(std::lround(vx * 1000.0));
  const int32_t w_angular_mrad_s = static_cast<int32_t>(std::lround(vth * 1000.0));
  uart::appendInt32TLV(payload, TAG_V_LINEAR_MM_S, v_linear_mm_s);
  uart::appendInt32TLV(payload, TAG_W_ANGULAR_MRAD_S, w_angular_mrad_s);

  // 一次性下发：滑台高度（单位 mm）
  int32_t z_lift_mm = 0;
  if (z_lift_on_.load() != 0) {
    // 协议单位为 1mm：直接下发
    z_lift_mm = static_cast<int32_t>(z_lift_offset_.load());
    uart::appendInt32TLV(payload, TAG_Z_LIFT_MM, z_lift_mm);
  }

  // 一次性下发：状态开关（仅在有新指令时下发）
  const uint16_t status_mask = hw_mode1_.load();
  const uint16_t status_value = hw_mode2_.load();
  if (status_mask != 0) {
    uart::appendUint16TLV(payload, TAG_STATUS_MASK, status_mask);
    uart::appendUint16TLV(payload, TAG_STATUS_VALUE, status_value);
  }

  // 一次性下发：浇水量（uint16）
  const uint16_t spray_volume_ml = pending_spray_volume_ml_.load();
  const bool has_spray_volume = has_pending_spray_volume_.load();
  if (has_spray_volume) {
    uart::appendUint16TLV(payload, TAG_SPRAY_VOLUME_ML, spray_volume_ml);
  }

  std::vector<uint8_t> frame;
  frame.reserve(FRAME_FIXED_HEADER_LEN + payload.size() + 2);
  frame.push_back(FRAME_HEAD_1);
  frame.push_back(FRAME_HEAD_2);
  frame.push_back(PROTOCOL_VER);
  const uint8_t tx_seq = tx_seq_id_++;
  frame.push_back(tx_seq);
  frame.push_back(UART_TYPE_COMMAND);
  uint16_t payload_len = static_cast<uint16_t>(payload.size());
  frame.push_back(static_cast<uint8_t>(payload_len & 0xFF));
  frame.push_back(static_cast<uint8_t>((payload_len >> 8) & 0xFF));
  frame.insert(frame.end(), payload.begin(), payload.end());

  // CRC 计算覆盖 VER 之后的数据，低字节在前
  uint16_t crc = uart::crc16_modbus(frame.data() + 2, static_cast<uint16_t>(frame.size() - 2));
  frame.push_back(static_cast<uint8_t>(crc & 0xFF));
  frame.push_back(static_cast<uint8_t>((crc >> 8) & 0xFF));
  const std::string tx_frame_hex = bytesToHexString(frame.data(), frame.size());
  {
    std::lock_guard<std::mutex> lock(protocol_debug_mutex_);
    last_tx_info_.frame_hex = tx_frame_hex;
    last_tx_info_.v_linear_mm_s = v_linear_mm_s;
    last_tx_info_.w_angular_mrad_s = w_angular_mrad_s;
    last_tx_info_.z_lift_mm = z_lift_mm;
    last_tx_info_.status_mask = status_mask;
    last_tx_info_.status_value = status_value;
    last_tx_info_.spray_volume_ml = has_spray_volume ? spray_volume_ml : 0;
    last_tx_info_.seq = tx_seq;
  }
  if (z_lift_on_.load() != 0 || status_mask != 0 || has_spray_volume) {
    RCLCPP_INFO(
      rclcpp::get_logger("AgrobotHardwareInterface"),
      "send command: %s", tx_frame_hex.c_str());
  } 

  if (serial_port_.IsOpen()) {
    serial_port_.Write(frame);

    // 一次性指令发送后清除标记
    if (has_mode_update_.load()) {
      has_mode_update_.store(false);
    }
    if (hw_mode1_.load() != 0) {
      hw_mode1_.store(0);
    }
    if (z_lift_on_.load() != 0) {
      z_lift_on_.store(0);
    }
    if (has_pending_spray_volume_.load()) {
      has_pending_spray_volume_.store(false);
    }

    return hardware_interface::return_type::OK;
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("AgrobotHardwareInterface"), "serial port not open");
    return hardware_interface::return_type::ERROR;
  }
}

}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  robot_base::AgrobotHardwareInterface, hardware_interface::SystemInterface)
