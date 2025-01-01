// Copyright 2025 SMBU-PolarBear-Robotics-Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

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
