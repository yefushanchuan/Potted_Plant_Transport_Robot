# 机器人线速度（m/s，已由协议 mm/s 转换）
float32 speed_x

# 机器人角速度（rad/s，已由协议 mrad/s 转换）
float32 speed_z

# 电池电量（%）
float32 power

#花盆架索引
int8 rack_index

# 预留字段（历史兼容）
uint8 flag_1

# health_word
bool estop_state             # 急停状态 bit0 0:未触发 1:已触发
bool right_wheel_enabled     # 右轮电机使能 bit1 0:未使能 1:使能
bool right_wheel_alarm       # 右轮电机报警 bit2 0:未报警 1:报警
bool left_wheel_enabled      # 左轮电机使能 bit3 0:未使能 1:使能
bool left_wheel_alarm        # 左轮电机报警 bit4 0:未报警 1:报警
bool battery_comm_fault      # 电池电池通信故障 bit8 0:未故障 1:故障
bool remote_connected        # 遥控器连接状态 bit9 0:未连接 1:已连接
bool charging_on             # 充电开关状态 bit12 0:关闭 1:打开

bool motor_alarm             # 电机报警 bit2 | bit4

# alarm_info
string[] alarm_list  # 报警信息（字符串列表）

# ---------------- 协议联调字段（原始值/帧） ----------------

# 协议帧头字段
uint8 protocol_version
uint8 rx_seq
uint8 rx_type
uint16 rx_payload_len

# Telemetry 原始字段
uint16 health_word_raw
uint16 alarm_info_raw
uint16 battery_soc_x100      # 电池电量定标值：0.01%

# 最近一次原始帧（字节用空格分隔的大写十六进制）
string rx_frame_hex
string tx_frame_hex

# 最近一次下发命令（按协议定标后的原始值）
int32 cmd_v_linear_mm_s
int32 cmd_w_angular_mrad_s
uint16 cmd_status_mask
uint16 cmd_status_value

# 最近一次 ACK 信息（若未收到则保持 0）
uint8 ack_seq
uint8 ack_result
