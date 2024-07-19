/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       crc8_crc16.c/h
  * @brief      crc8 and crc16 calculate function, verify function, append function.
  *             crc8和crc16计算函数,校验函数,添加函数
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-11-2019     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#ifndef CRC8_CRC16_HPP
#define CRC8_CRC16_HPP

#include <cstdint>

namespace crc8
{
extern uint8_t get_CRC8_check_sum(
    uint8_t * pchMessage, unsigned int dwLength, uint8_t ucCRC8);

extern uint32_t verify_CRC8_check_sum(uint8_t * pchMessage, unsigned int dwLength);

extern void append_CRC8_check_sum(uint8_t * pchMessage, unsigned int dwLength);
}  // namespace crc8

namespace crc16
{
extern uint16_t get_CRC16_check_sum(uint8_t * pchMessage, uint32_t dwLength, uint16_t wCRC);

extern uint32_t verify_CRC16_check_sum(uint8_t * pchMessage, uint32_t dwLength);

extern void append_CRC16_check_sum(uint8_t * pchMessage, uint32_t dwLength);
}  // namespace crc16
#endif  //CRC8_CRC16_HPP
