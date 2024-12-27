// Copyright (c) 2024 SMBU-PolarBear-Robotics-Team
// Licensed under the MIT License.

#ifndef STANDARD_ROBOT_PP_ROS2__CRC8_CRC16_HPP_
#define STANDARD_ROBOT_PP_ROS2__CRC8_CRC16_HPP_

#include <cstdint>
#include <vector>

namespace crc8
{
extern uint8_t get_CRC8_check_sum(uint8_t * pchMessage, unsigned int dwLength, uint8_t ucCRC8);

extern uint32_t verify_CRC8_check_sum(uint8_t * pchMessage, unsigned int dwLength);

extern void append_CRC8_check_sum(uint8_t * pchMessage, unsigned int dwLength);
}  // namespace crc8

namespace crc16
{
extern uint16_t get_CRC16_check_sum(uint8_t * pchMessage, uint32_t dwLength, uint16_t wCRC);

extern uint32_t verify_CRC16_check_sum(uint8_t * pchMessage, uint32_t dwLength);

extern void append_CRC16_check_sum(uint8_t * pchMessage, uint32_t dwLength);

// 对vector重载

extern bool verify_CRC16_check_sum(std::vector<uint8_t> & pchMessage);
}  // namespace crc16
#endif  // STANDARD_ROBOT_PP_ROS2__CRC8_CRC16_HPP_
