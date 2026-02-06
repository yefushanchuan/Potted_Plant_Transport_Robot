#include <cstdint>
#include <vector>

#include "gtest/gtest.h"
#include "robot_base/uart_protocol.hpp"

namespace robot_base::uart
{
namespace
{

TEST(Crc16Modbus, MatchesProtocolExampleCommand)
{
  // 来源：上位机与STM32通讯协议.md（“下发”示例帧，CRC 字节为 0x26 0xB5）
  const std::vector<uint8_t> frame = {
    0xA0, 0x0A,  // H1 H2
    0x01,        // VER
    0x01,        // SEQ
    0x81,        // TYPE (Command)
    0x14, 0x00,  // LEN_L LEN_H
    // PAYLOAD
    0x01, 0x04, 0x64, 0x00, 0x00, 0x00,  // v_linear = 100 mm/s
    0x02, 0x04, 0x64, 0x00, 0x00, 0x00,  // w_angular = 100 mrad/s
    0x11, 0x02, 0x01, 0x00,              // status_mask
    0x12, 0x02, 0x01, 0x00,              // status_value
  };

  const uint16_t crc = crc16_modbus(frame.data() + 2, static_cast<uint16_t>(frame.size() - 2));
  EXPECT_EQ(crc, 0xB526);
  EXPECT_EQ(static_cast<uint8_t>(crc & 0xFF), 0x26);
  EXPECT_EQ(static_cast<uint8_t>((crc >> 8) & 0xFF), 0xB5);
}

TEST(TlvEncoding, AppendsUint16LittleEndian)
{
  std::vector<uint8_t> payload;
  appendUint16TLV(payload, 0x11, 0x1234);
  const std::vector<uint8_t> expected = {0x11, 0x02, 0x34, 0x12};
  EXPECT_EQ(payload, expected);
}

TEST(TlvEncoding, AppendsInt32LittleEndian)
{
  std::vector<uint8_t> payload;
  appendInt32TLV(payload, 0x01, 0x01020304);
  const std::vector<uint8_t> expected = {0x01, 0x04, 0x04, 0x03, 0x02, 0x01};
  EXPECT_EQ(payload, expected);
}

TEST(LittleEndianRead, ReadsUint16AndInt32)
{
  const uint8_t u16_bytes[] = {0x34, 0x12};
  EXPECT_EQ(readUint16LE(u16_bytes), 0x1234);

  const uint8_t i32_bytes[] = {0x78, 0x56, 0x34, 0x12};
  EXPECT_EQ(readInt32LE(i32_bytes), 0x12345678);
}

}  // namespace
}  // namespace robot_base::uart
