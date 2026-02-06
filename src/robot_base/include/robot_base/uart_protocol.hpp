#pragma once

#include <cstdint>
#include <vector>

namespace robot_base
{
namespace uart
{

inline uint16_t crc16_modbus(const uint8_t * data, uint16_t len)
{
  uint16_t crc = 0xFFFF;
  for (uint16_t i = 0; i < len; i++) {
    crc ^= static_cast<uint16_t>(data[i]);
    for (uint8_t b = 0; b < 8; b++) {
      if (crc & 0x0001) {
        crc = static_cast<uint16_t>((crc >> 1) ^ 0xA001);
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

inline uint16_t readUint16LE(const uint8_t * data)
{
  return static_cast<uint16_t>(static_cast<uint16_t>(data[0]) |
         (static_cast<uint16_t>(data[1]) << 8));
}

inline int32_t readInt32LE(const uint8_t * data)
{
  const uint32_t value = (static_cast<uint32_t>(data[0]) << 0) |
    (static_cast<uint32_t>(data[1]) << 8) |
    (static_cast<uint32_t>(data[2]) << 16) |
    (static_cast<uint32_t>(data[3]) << 24);
  return static_cast<int32_t>(value);
}

inline void appendUint16TLV(std::vector<uint8_t> & payload, uint8_t tag, uint16_t value)
{
  payload.push_back(tag);
  payload.push_back(2);
  payload.push_back(static_cast<uint8_t>(value & 0xFF));
  payload.push_back(static_cast<uint8_t>((value >> 8) & 0xFF));
}

inline void appendInt32TLV(std::vector<uint8_t> & payload, uint8_t tag, int32_t value)
{
  payload.push_back(tag);
  payload.push_back(4);
  const uint32_t uvalue = static_cast<uint32_t>(value);
  payload.push_back(static_cast<uint8_t>((uvalue >> 0) & 0xFF));
  payload.push_back(static_cast<uint8_t>((uvalue >> 8) & 0xFF));
  payload.push_back(static_cast<uint8_t>((uvalue >> 16) & 0xFF));
  payload.push_back(static_cast<uint8_t>((uvalue >> 24) & 0xFF));
}

}  // namespace uart
}  // namespace robot_base
