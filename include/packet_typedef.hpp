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

const uint8_t DEBUG_PACKAGE_NUM = 10;
const uint8_t DEBUG_PACKAGE_NAME_LEN = 10;

struct HeaderFrame
{
    uint8_t sof;
    uint8_t len;
    uint8_t id;
    uint8_t crc;
} __attribute__((packed));

/********************************************************/
/* Receive data                                         */
/********************************************************/

// 串口调试数据包
struct ReceiveDebugData
{
    HeaderFrame frame_header;

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

// 机器人信息数据包
struct ReceiveRobotInfoData
{
    HeaderFrame frame_header;

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

        /// @brief 机器人运动状态 12 bytes
        struct
        {
            float vx;  // m/s
            float vy;  // m/s
            float wz;  // rad/s
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
        reinterpret_cast<const uint8_t *>(&data),
        reinterpret_cast<const uint8_t *>(&data) + sizeof(T), packet.begin());
    return packet;
}

}  // namespace ros2_standard_robot_pp

#endif  // ROS2_STANDARD_ROBOT_PP__PACKET_TYPEDEF_HPP_
