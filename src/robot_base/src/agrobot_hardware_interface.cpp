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
#include "sensor_msgs/msg/battery_state.hpp"
#include "robot_base/uart_protocol.hpp"

namespace robot_base
{

// 控制类一次性参数初始化/复位
void AgrobotHardwareInterface::resetCommandParams()
{
  hw_mode1_.store(0);
  hw_mode2_.store(0);
  rack_index_cmd_.store(0);
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
constexpr uint8_t TAG_RACK_INDEX = 0x05;        // int8: 花盆架索引
constexpr uint8_t TAG_STATUS_MASK = 0x11;       // u16
constexpr uint8_t TAG_STATUS_VALUE = 0x12;      // u16
constexpr uint8_t TAG_STATUS_WORD = 0x13;       // u16
constexpr uint8_t TAG_HEALTH_WORD = 0x14;       // u16
constexpr uint8_t TAG_ALARM_INFO = 0x15;        // u16
constexpr uint8_t TAG_BATT_SOC_X100 = 0x21;     // u16: 电池百分比*100
constexpr uint8_t TAG_ACK_RESULT = 0x30;        // u8: 0=OK，其它=错误码
constexpr uint8_t TAG_ACK_REQUEST = 0x40;       // u8: 请求 ACK

constexpr uint16_t RIGHT_WHEEL_ENABLE_SHIFT = 0;  // bit0-bit1
constexpr uint16_t RIGHT_WHEEL_RESET_BIT = 2;     // bit2
constexpr uint16_t LEFT_WHEEL_ENABLE_SHIFT = 3;   // bit3-bit4
constexpr uint16_t LEFT_WHEEL_RESET_BIT = 5;      // bit5
constexpr uint16_t ACTION_ENABLE_BIT = 9;         // bit9: 0:不修改(Mask=0), 1:使能(写入1), 2:失能(写入0)
constexpr uint16_t ACTION_TYPE_SHIFT = 10;        // bit10-bit11:  0:不修改(Mask=00), 1:搬运(01), 2:卸载(10), 3:slave(11)
constexpr uint16_t CHARGING_ON_BIT = 14;          // bit14: 充电控制位

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
  last_robot_vx_ = 0.0;
  last_robot_vth_ = 0.0;

  // 创建非实时节点
  non_realtime_node_ = rclcpp::Node::make_shared(info_.name + "_non_realtime_node");
  // 创建车轮信息发布者
  agrobot_info_pub_ = non_realtime_node_->create_publisher<robot_base::msg::AgrobotInfo>(
    "/agrobot_base_info", rclcpp::SystemDefaultsQoS());
  // 创建电池信息发布者
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

    // 此函数增加cmd = 3 支持（slave模式）
    auto applyEnable2Bits = [&](uint16_t shift, uint8_t cmd, const char * name) -> bool {
        if (cmd == 0) {
          return true;
        }
        if (cmd > 3) {
          RCLCPP_ERROR(
            rclcpp::get_logger("AgrobotHardwareInterface"),
            "%s_cmd 取值非法：%u（期望 0/1/2/3）",
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
      response->success = false;
      return;
    }
    if (!applyEnable2Bits(
        LEFT_WHEEL_ENABLE_SHIFT, request->left_wheel_enable_cmd, "left_wheel_enable"))
    {
      response->success = false;
      return;
    }
    if (!applyResetBit(
        RIGHT_WHEEL_RESET_BIT, request->right_wheel_reset_cmd, "right_wheel_reset"))
    {
      response->success = false;
      return;
    }
    if (!applyResetBit(
      LEFT_WHEEL_RESET_BIT, request->left_wheel_reset_cmd, "left_wheel_reset"))
    {
      response->success = false;
      return;
    }
    if (!applyTriStateBit(
      CHARGING_ON_BIT, request->charging_on_cmd, "charging_on"))
    {
      response->success = false;
      return;
    }

    uint16_t expected_mask = this->hw_mode1_.load();
    while (!this->hw_mode1_.compare_exchange_weak(expected_mask, expected_mask | status_mask)) {}

    uint16_t expected_val = this->hw_mode2_.load();
    uint16_t new_val;
    do {
        new_val = (expected_val & ~status_mask) | (status_value & status_mask);
    } while (!this->hw_mode2_.compare_exchange_weak(expected_val, new_val));

    RCLCPP_INFO(
      rclcpp::get_logger("AgrobotHardwareInterface"),
      "SetControlMode: status_mask=0x%04X status_value=0x%04X",
      status_mask, status_value);

    response->success = true;
  };
    
  // 创建设置控制模式的服务
  mode_service_ = non_realtime_node_->create_service<robot_base::srv::SetControlMode>(
    "/set_agrobotbase_ctrlmode", service_callback);

  using OperatePot = robot_base::action::OperatePot;
  using GoalHandleOperatePot = rclcpp_action::ServerGoalHandle<OperatePot>;

  auto handle_goal = [this](const rclcpp_action::GoalUUID & /*uuid*/, std::shared_ptr<const OperatePot::Goal> goal) {
    RCLCPP_INFO(
      rclcpp::get_logger("AgrobotHardwareInterface"), 
      "Action: 收到请求, 动作类型 %d, 架子索引 %d", 
      goal->action_type, goal->rack_index);

    bool expected = false;
    // 原子操作：如果当前 busy 为 false (expected)，则将其瞬间改为 true，并放行
    if (!this->is_action_busy_.compare_exchange_strong(expected, true)) {
      RCLCPP_WARN(
        rclcpp::get_logger("AgrobotHardwareInterface"), 
        "底层正在执行动作，拒绝新的 Action 请求");
      return rclcpp_action::GoalResponse::REJECT;
    }
    
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  };

  auto handle_cancel =[this](const std::shared_ptr<GoalHandleOperatePot> /*goal_handle*/) {
    RCLCPP_INFO(
      rclcpp::get_logger("AgrobotHardwareInterface"), 
      "Action: 收到取消请求，紧急下发失能指令！");

    // 只操作动作使能位，将其置为 0 表示失能/停止
    uint16_t status_mask = (1u << ACTION_ENABLE_BIT);
    uint16_t status_value = 0; 
    
    uint16_t expected_mask = this->hw_mode1_.load();
    while (!this->hw_mode1_.compare_exchange_weak(expected_mask, expected_mask | status_mask)) {}

    uint16_t expected_val = this->hw_mode2_.load();
    uint16_t new_val;
    do {
        new_val = (expected_val & ~status_mask) | (status_value & status_mask);
    } while (!this->hw_mode2_.compare_exchange_weak(expected_val, new_val));
    
    return rclcpp_action::CancelResponse::ACCEPT;
  };

  auto handle_accepted = [this](const std::shared_ptr<GoalHandleOperatePot> goal_handle) {
    {
      std::lock_guard<std::mutex> lock(this->action_thread_mutex_);
      if (this->action_thread_.joinable()) {
        this->action_thread_.join();
      } // 旧线程结束后才启动新线程，确保同一时间只有一个动作线程在运行

      this->action_thread_ = std::thread([this, goal_handle]() {
        
        auto unlock_guard = std::shared_ptr<void>(nullptr, [this](void*) {
          this->is_action_busy_.store(false);
        });

        const auto goal = goal_handle->get_goal();
        const bool is_slave_mode = (goal->action_type == 3);
        
        uint16_t mask = (1u << ACTION_ENABLE_BIT) | (0x03u << ACTION_TYPE_SHIFT);
        uint16_t val  = (1u << ACTION_ENABLE_BIT) | ((goal->action_type & 0x03u) << ACTION_TYPE_SHIFT);
        
        if (!is_slave_mode) {
          this->rack_index_cmd_.store(goal->rack_index);
        }
        
        uint16_t expected_mask = this->hw_mode1_.load();
        while (!this->hw_mode1_.compare_exchange_weak(expected_mask, expected_mask | mask)) {}

        uint16_t expected_val = this->hw_mode2_.load();
        uint16_t new_val;
        do {
            new_val = (expected_val & ~mask) | (val & mask);
        } while (!this->hw_mode2_.compare_exchange_weak(expected_val, new_val));
        
        auto result = std::make_shared<OperatePot::Result>();
        auto feedback = std::make_shared<OperatePot::Feedback>();
        
        rclcpp::Rate loop_rate(10);
        int wait_start_timeout = 50;
        bool has_started = false;

        // 阶段1：启动确认
        while (rclcpp::ok() && !this->shutting_down_.load() && wait_start_timeout > 0 && !is_slave_mode) {
          if (this->current_action_running_flag_.load()) {
            has_started = true;
            break;
          }
          if (goal_handle->is_canceling()) {
            result->success = false;
            result->message = "启动前被取消";
            goal_handle->canceled(result);
            return;
          }
          wait_start_timeout--;
          loop_rate.sleep();
        }

        // Slave 模式不需要等启动反馈，直接进入阶段 2
        if (is_slave_mode) {
          has_started = true;
        }

        if (!has_started) {
          RCLCPP_ERROR(rclcpp::get_logger("AgrobotHardwareInterface"), "Action: 启动超时(5s)，底层无响应");
          result->success = false;
          result->message = "硬件超时无响应";
          goal_handle->abort(result);
          return;
        }

        // 阶段2：执行中
        if (is_slave_mode) {
          // 【关键修复 2】：Slave 模式在循环内持续发送 feedback，保持 Action 活跃
          while (rclcpp::ok() && !this->shutting_down_.load()) {
            if (goal_handle->is_canceling()) {
              // 客户端主动取消 Slave 是预期内的正常行为，所以 success 设为 true
              result->success = true;
              result->message = "slave 模式已主动取消并退出";
              goal_handle->canceled(result);
              return;
            }
            
            feedback->is_running = true;
            goal_handle->publish_feedback(feedback);
            
            loop_rate.sleep();
          }
        } else {
          // 非 slave：等 running 结束
          while (rclcpp::ok() && !this->shutting_down_.load()) {
            if (goal_handle->is_canceling()) {
              // 搬运或卸载中途被取消，算作任务失败
              result->success = false; 
              result->message = "执行中途被取消";
              goal_handle->canceled(result);
              return;
            }
            
            bool is_running = this->current_action_running_flag_.load(); 
            feedback->is_running = is_running;
            goal_handle->publish_feedback(feedback);
            
            if (!is_running) {
              break;
            }
            loop_rate.sleep();
          }
        }
        
        // 节点关闭检查
        if (this->shutting_down_.load()) {
          result->success = false;
          result->message = "节点正在停用，任务中断";
          goal_handle->abort(result);
          return;
        }

        // 正常完成
        if (rclcpp::ok()) {
          result->success = true;
          result->message = is_slave_mode ? "slave 模式运行结束" : "任务执行完毕";
          goal_handle->succeed(result);
          RCLCPP_INFO(rclcpp::get_logger("AgrobotHardwareInterface"), "Action: %s", result->message.c_str());
        }
      });
    }
  };

  action_server_ = rclcpp_action::create_server<OperatePot>(
    non_realtime_node_,
    "operate_pot",
    handle_goal,
    handle_cancel,
    handle_accepted);
    
  // 创建并启动节点线程
  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(non_realtime_node_);
  node_thread_ = std::thread([this]() { this->executor_->spin(); });

  // 打印初始化成功信息
  RCLCPP_INFO(rclcpp::get_logger("AgrobotHardwareInterface"), "on_init 成功");
  return hardware_interface::CallbackReturn::SUCCESS;
}

AgrobotHardwareInterface::~AgrobotHardwareInterface() {
  // 1. 先通知底层循环退出
  shutting_down_.store(true);
  
  // 2. 取消 ROS 2 调度器并等待节点线程结束（阻止新的 Service/Action 进入）
  if (executor_) { executor_->cancel(); }
  if (node_thread_.joinable()) { node_thread_.join(); }

  // 3. 安全加锁，回收遗留的 Action 执行线程
  {
    std::lock_guard<std::mutex> lock(action_thread_mutex_);
    if (action_thread_.joinable()) { 
      action_thread_.join(); 
    }
  }
}

// 暴露左右轮位置、速度状态等接口给控制器
std::vector<hardware_interface::StateInterface> AgrobotHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // 导出左轮的状态接口
  state_interfaces.emplace_back(
    hardware_interface::StateInterface(
      info_.joints[0].name, "position", &hw_position_left_));
  state_interfaces.emplace_back(
    hardware_interface::StateInterface(
      info_.joints[0].name, "velocity", &hw_velocity_left_));

  // 导出右轮的状态接口
  state_interfaces.emplace_back(
    hardware_interface::StateInterface(
      info_.joints[1].name, "position", &hw_position_right_));
  state_interfaces.emplace_back(
    hardware_interface::StateInterface(
      info_.joints[1].name, "velocity", &hw_velocity_right_));

  return state_interfaces;
}

// 暴露左右轮速度等指令接口给控制器
std::vector<hardware_interface::CommandInterface> AgrobotHardwareInterface::
export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // 导出左轮的命令接口
  command_interfaces.emplace_back(
    hardware_interface::CommandInterface(
      info_.joints[0].name, "velocity", &hw_command_velocity_left_));

  // 导出右轮的命令接口
  command_interfaces.emplace_back(
    hardware_interface::CommandInterface(
      info_.joints[1].name, "velocity", &hw_command_velocity_right_));

  return command_interfaces;
}

// 激活阶段：打开串口并准备通信
hardware_interface::CallbackReturn AgrobotHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  hw_position_left_ = 0.0;
  hw_position_right_ = 0.0;

  shutting_down_.store(false);
  is_action_busy_.store(false);
  current_action_running_flag_.store(false);
  resetCommandParams(); 

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

  // 1. 通知底层的 Action 线程尽快退出（中断当前正在执行的任务）
  shutting_down_.store(true);

  // 2. 仅回收 Action 线程。千万不要 cancel executor_，保持 ROS 服务的在线状态！
  {
    std::lock_guard<std::mutex> lock(action_thread_mutex_);
    if (action_thread_.joinable()) {
      action_thread_.join();
    } 
  }

  // 3. 关闭串口
  if (serial_port_.IsOpen()) {
    serial_port_.Close();
    RCLCPP_INFO(rclcpp::get_logger("AgrobotHardwareInterface"), "Successfully close stm32_serial!");
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
            size_t drop_count = 1;
            while (drop_count < rx_buffer_.size() && rx_buffer_[drop_count] != FRAME_HEAD_1) {
              drop_count++;
            }
            rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + drop_count);
            RCLCPP_WARN(
              rclcpp::get_logger("AgrobotHardwareInterface"),
              "帧头对齐失败，一次性丢弃 %zu 个无效字节，重新寻找帧头", drop_count
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
              "PAYLOAD 长度异常：%u（上限 %u），丢弃当前帧头重新同步",
              static_cast<unsigned>(payload_len),
              static_cast<unsigned>(MAX_PAYLOAD_LEN));
            rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + 2);
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
            rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + 2);
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
            // CRC错误说明数据段有错位或干扰，丢弃帧头(0xA0 0x0A)，从后续数据中重新寻头
            rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + 2);
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
            uint16_t health_word = last_health_word_;
            uint16_t alarm_info = last_alarm_info_;
            uint16_t battery_soc_x100 = last_battery_soc_x100_;

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
                  robot_vth = -robot_vth;
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
              msg->speed_z = robot_vth; //ROS协议
              msg->power = static_cast<float>(battery_soc_x100) / 100.0f;
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
                msg->cmd_status_mask = last_tx_info_.status_mask;
                msg->cmd_status_value = last_tx_info_.status_value;
                msg->ack_seq = last_ack_info_.seq;
                msg->ack_result = last_ack_info_.result;
              }
              // bit0: estop_state
              msg->estop_state = (health_word & (1u << 0)) != 0;
              // bit1: right_wheel_enabled
              msg->right_wheel_enabled = (health_word & (1u << 1)) != 0;
              // bit2: right_wheel_alarm
              msg->right_wheel_alarm = (health_word & (1u << 2)) != 0;
              // bit3: left_wheel_enabled
              msg->left_wheel_enabled = (health_word & (1u << 3)) != 0;
              // bit4: left_wheel_alarm
              msg->left_wheel_alarm = (health_word & (1u << 4)) != 0;
              // bit7: action_running
              msg->action_running = (health_word & (1u << 7)) != 0;
              // bit8: battery_comm_fault
              msg->battery_comm_fault = (health_word & (1u << 8)) != 0;
              // bit9: rc_force_ctl
              msg->rc_force_ctl = (health_word & (1u << 9)) != 0;
              // bit12: charging_on
              msg->charging_on = (health_word & (1u << 12)) != 0;

              // 同步给 Action 线程用于判断底层状态
              current_action_running_flag_.store(msg->action_running);

              // 综合电机报警标志
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
              last_health_word_ = health_word;
              last_alarm_info_ = alarm_info;
              last_battery_soc_x100_ = battery_soc_x100;

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

          if (!parsed) {
            RCLCPP_WARN(
              rclcpp::get_logger("AgrobotHardwareInterface"),
              "收到未知帧类型(0x%02X)或 TLV 结构解析失败，但 CRC 校验有效，忽略该帧。", 
              static_cast<unsigned>(type)
            );
          }
          // 因为前面已经通过了 CRC 校验，说明这 `frame_len` 长度的数据是完整且未损坏的包裹。
          // 无论解析是否认识它（parsed状态），都必须一次性把这一整帧移除，绝不能只丢1个字节！
          rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + frame_len);
          
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
  double vth = (hw_command_velocity_right_ - hw_command_velocity_left_) * wheel_radius_ / wheel_separation_;
  vth = -vth;  // ROS 中正值为逆时针旋转，协议中正值为顺时针，需取反转换
  uint16_t status_mask = hw_mode1_.exchange(0);
  uint16_t status_value = hw_mode2_.exchange(0);
  int8_t rack_idx = 0; // 默认值，只有在特定指令下才会被更新

  // 按新协议组装 Command TLV
  std::vector<uint8_t> payload;
  const int32_t v_linear_mm_s = static_cast<int32_t>(std::lround(vx * 1000.0));
  const int32_t w_angular_mrad_s = static_cast<int32_t>(std::lround(vth * 1000.0));
  uart::appendInt32TLV(payload, TAG_V_LINEAR_MM_S, v_linear_mm_s);
  uart::appendInt32TLV(payload, TAG_W_ANGULAR_MRAD_S, w_angular_mrad_s);

  if (status_mask != 0) {
    uart::appendUint16TLV(payload, TAG_STATUS_MASK, status_mask);
    uart::appendUint16TLV(payload, TAG_STATUS_VALUE, status_value);
    
    // 仅当掩码中包含动作指令时，才下发架子索引
    if ((status_mask & (1u << ACTION_ENABLE_BIT)) || 
        (status_mask & (0x03u << ACTION_TYPE_SHIFT))) 
    {
      rack_idx = rack_index_cmd_.exchange(0); 
      uart::appendInt8TLV(payload, TAG_RACK_INDEX, rack_idx);
    }
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
    last_tx_info_.status_mask = status_mask;
    last_tx_info_.status_value = status_value;
    last_tx_info_.rack_index = rack_idx;
    last_tx_info_.seq = tx_seq;
  }

  if (status_mask != 0) {
    RCLCPP_INFO(rclcpp::get_logger("AgrobotHardwareInterface"), "send command: %s", tx_frame_hex.c_str());
  } 

  if (serial_port_.IsOpen()) {
    serial_port_.Write(frame);
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
