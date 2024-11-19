#ifndef ROS2_STANDARD_ROBOT_PP__PACKET_TYPEDEF_HPP_
#define ROS2_STANDARD_ROBOT_PP__PACKET_TYPEDEF_HPP_

#include <cstdint>
#include <vector>

namespace ros2_standard_robot_pp
{
const uint8_t SOF_RECEIVE = 0x5A;
const uint8_t SOF_SEND = 0x5A;

const uint8_t ID_DEBUG = 0x01;
const uint8_t ID_IMU = 0x02;
const uint8_t ID_ROBOT_INFO = 0x03;
const uint8_t ID_PID_DEBUG = 0x04;
const uint8_t ID_ALL_ROBOT_HP = 0x05;
const uint8_t ID_GAME_STATUS = 0x06;
const uint8_t ID_ROBOT_MOTION = 0x07;

const uint8_t ID_ROBOT_CMD = 0x01;

const uint8_t DEBUG_PACKAGE_NUM = 10;
const uint8_t DEBUG_PACKAGE_NAME_LEN = 10;

struct HeaderFrame
{
  uint8_t sof;  // 数据帧起始字节，固定值为 0x5A
  uint8_t len;  // 数据段长度
  uint8_t id;   // 数据段id
  uint8_t crc;  // 数据帧头的 CRC8 校验
} __attribute__((packed));

/********************************************************/
/* Receive data                                         */
/********************************************************/

// 串口调试数据包
struct ReceiveDebugData
{
  HeaderFrame frame_header;  // id = 0x01

  uint32_t time_stamp;

  struct
  {
    uint8_t name[DEBUG_PACKAGE_NAME_LEN];
    uint8_t type;
    float data;
  } __attribute__((packed)) packages[DEBUG_PACKAGE_NUM];

  uint16_t checksum;
} __attribute__((packed));

// IMU 数据包
struct ReceiveImuData
{
  HeaderFrame frame_header;  // id = 0x02

  uint32_t time_stamp;

  struct
  {
    float yaw;    // rad
    float pitch;  // rad
    float roll;   // rad

    float yaw_vel;    // rad/s
    float pitch_vel;  // rad/s
    float roll_vel;   // rad/s

    // float x_accel;  // m/s^2
    // float y_accel;  // m/s^2
    // float z_accel;  // m/s^2
  } __attribute__((packed)) data;

  uint16_t crc;
} __attribute__((packed));

// 机器人信息数据包
struct ReceiveRobotInfoData
{
  HeaderFrame frame_header;  // id = 0x03

  uint32_t time_stamp;

  struct
  {
    /// @brief 机器人部位类型 2 bytes
    struct
    {
      uint16_t chassis : 3;
      uint16_t gimbal : 3;
      uint16_t shoot : 3;
      uint16_t arm : 3;
      uint16_t custom_controller : 3;
      uint16_t reserve : 1;
    } __attribute__((packed)) type;

    /// @brief 机器人部位状态 1 byte
    /// @note 0: 正常，1: 错误
    struct
    {
      uint8_t chassis : 1;
      uint8_t gimbal : 1;
      uint8_t shoot : 1;
      uint8_t arm : 1;
      uint8_t custom_controller : 1;
      uint8_t reserve : 3;
    } __attribute__((packed)) state;

    /// @brief 机器人裁判系统信息 7 bytes
    struct
    {
      uint8_t id;
      uint8_t color;  // 0-red 1-blue 2-unknown
      bool attacked;
      uint16_t hp;
      uint16_t heat;
    } __attribute__((packed)) referee;

  } __attribute__((packed)) data;

  uint16_t crc;
} __attribute__((packed));

// PID调参数据包
struct ReceivePidDebugData
{
  HeaderFrame frame_header;  // id = 0x04

  uint32_t time_stamp;

  struct
  {
    float fdb;
    float ref;
    float pid_out;
  } __attribute__((packed)) data;

  uint16_t crc;
} __attribute__((packed));

// 全场机器人hp信息数据包
struct ReceiveAllRobotHpData
{
  HeaderFrame frame_header;  // id = 0x05

  uint32_t time_stamp;

  struct
  {
    uint16_t red_1_robot_hp;
    uint16_t red_2_robot_hp;
    uint16_t red_3_robot_hp;
    uint16_t red_4_robot_hp;
    uint16_t red_5_robot_hp;
    uint16_t red_7_robot_hp;
    uint16_t red_outpost_hp;
    uint16_t red_base_hp;
    uint16_t blue_1_robot_hp;
    uint16_t blue_2_robot_hp;
    uint16_t blue_3_robot_hp;
    uint16_t blue_4_robot_hp;
    uint16_t blue_5_robot_hp;
    uint16_t blue_7_robot_hp;
    uint16_t blue_outpost_hp;
    uint16_t blue_base_hp;
  } __attribute__((packed)) data;

  uint16_t crc;
} __attribute__((packed));

// 比赛信息数据包
struct ReceiveGameStatusData
{
  HeaderFrame frame_header;  // id = 0x06

  uint32_t time_stamp;

  struct
  {
    uint8_t game_progress;
    uint16_t stage_remain_time;
  } __attribute__((packed)) data;

  uint16_t crc;
} __attribute__((packed));

// 机器人运动数据包
struct ReceiveRobotMotionData
{
  HeaderFrame frame_header;  // id = 0x07

  uint32_t time_stamp;

  struct
  {
    struct
    {
      float vx;
      float vy;
      float wz;
    } __attribute__((packed)) speed_vector;

  } __attribute__((packed)) data;

  uint16_t crc;
} __attribute__((packed));

/********************************************************/
/* Send data                                            */
/********************************************************/

struct SendRobotCmdData
{
  HeaderFrame frame_header;

  uint32_t time_stamp;

  struct
  {
    struct
    {
      float vx;
      float vy;
      float wz;
    } __attribute__((packed)) speed_vector;

    struct
    {
      float roll;
      float pitch;
      float yaw;
      float leg_lenth;
    } __attribute__((packed)) chassis;

    struct
    {
      float pitch;
      float yaw;
    } __attribute__((packed)) gimbal;

    struct
    {
      uint8_t fire;
      uint8_t fric_on;
    } __attribute__((packed)) shoot;

  } __attribute__((packed)) data;

  uint16_t checksum;
} __attribute__((packed));

/********************************************************/
/* template                                             */
/********************************************************/

template <typename T>
inline T fromVector(const std::vector<uint8_t> & data)
{
  T packet;
  std::copy(data.begin(), data.end(), reinterpret_cast<uint8_t *>(&packet));
  return packet;
}

template <typename T>
inline std::vector<uint8_t> toVector(const T & data)
{
  std::vector<uint8_t> packet(sizeof(T));
  std::copy(
    reinterpret_cast<const uint8_t *>(&data), reinterpret_cast<const uint8_t *>(&data) + sizeof(T),
    packet.begin());
  return packet;
}

}  // namespace ros2_standard_robot_pp

#endif  // ROS2_STANDARD_ROBOT_PP__PACKET_TYPEDEF_HPP_
