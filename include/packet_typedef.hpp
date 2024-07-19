#ifndef ROS2_STANDARD_ROBOT_PP__PACKET_TYPEDEF_HPP_
#define ROS2_STANDARD_ROBOT_PP__PACKET_TYPEDEF_HPP_

#include <algorithm>
#include <cstdint>
#include <vector>

namespace ros2_standard_robot_pp
{

const uint8_t ID_DEBUG = 0x01;
const uint8_t ID_IMU = 0x02;
const uint8_t ID_ROBOT_INFO = 0x03;

struct HeaderFrame
{
  uint8_t sof;
  uint8_t len;
  uint8_t id;
  uint8_t crc;
} __attribute__((packed));

// IMU 数据包
struct ReceiveImuData
{
    HeaderFrame frame_header;

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
