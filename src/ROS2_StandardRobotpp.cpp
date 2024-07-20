/**
  ****************************(C) COPYRIGHT 2024 Polarbear*************************
  * @file       ROS2_StandardRobotpp.hpp/cpp
  * @brief      上下位机通信模块
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Jul-24-2024     Penguin         1. done
  @verbatim
  =================================================================================

  =================================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear*************************
  */

#include <tf2/LinearMath/Quaternion.h>

#include <iostream>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>
#include <serial_driver/serial_driver.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// C++ system
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "CRC8_CRC16.hpp"
#include "ROS2_StandardRobotpp.hpp"
#include "packet_typedef.hpp"

namespace ros2_standard_robot_pp
{

#define SOF_RECEIVE 0x5A
#define SOF_SEND 0x5A

ROS2_StandardRobotpp::ROS2_StandardRobotpp(const rclcpp::NodeOptions & options)
: Node("ros2_standard_robot_pp", options),
  owned_ctx_{new IoContext(2)},
  serial_driver_{new drivers::serial_driver::SerialDriver(*owned_ctx_)}
{
    RCLCPP_INFO(get_logger(), "Start ROS2_StandardRobotpp!");
    std::cout << "\033[32m Start ROS2_StandardRobotpp! \033[0m" << std::endl;

    getParams();

    // Create Publisher
    //   latency_pub_ = this->create_publisher<std_msgs::msg::Float64>("/latency", 10);
    //   marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/aiming_point", 10);

    //   try {
    //     serial_driver_->init_port(device_name_, *device_config_);
    //     if (!serial_driver_->port()->is_open()) {
    //       serial_driver_->port()->open();
    //       receive_thread_ = std::thread(&RMSerialDriver::receiveData, this);
    //     }
    //   } catch (const std::exception & ex) {
    //     RCLCPP_ERROR(
    //       get_logger(), "Error creating serial port: %s - %s", device_name_.c_str(), ex.what());
    //     throw ex;
    //   }

    // Create Subscription
    //   target_sub_ = this->create_subscription<auto_aim_interfaces::msg::Target>(
    //     "/tracker/target", rclcpp::SensorDataQoS(),
    //     std::bind(&RMSerialDriver::sendDataVision, this, std::placeholders::_1));
    //   cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    //     "/cmd_vel", 10, std::bind(&RMSerialDriver::sendDataTwist, this, std::placeholders::_1));

    receive_thread_ = std::thread(&ROS2_StandardRobotpp::receiveData, this);
}

ROS2_StandardRobotpp::~ROS2_StandardRobotpp()
{
    if (receive_thread_.joinable()) {
        receive_thread_.join();
    }

    if (serial_driver_->port()->is_open()) {
        serial_driver_->port()->close();
    }

    if (owned_ctx_) {
        owned_ctx_->waitForExit();
    }
}

void ROS2_StandardRobotpp::getParams()
{
    using FlowControl = drivers::serial_driver::FlowControl;
    using Parity = drivers::serial_driver::Parity;
    using StopBits = drivers::serial_driver::StopBits;

    uint32_t baud_rate{};
    auto fc = FlowControl::NONE;
    auto pt = Parity::NONE;
    auto sb = StopBits::ONE;

    try {
        device_name_ = declare_parameter<std::string>("device_name", "");
    } catch (rclcpp::ParameterTypeException & ex) {
        RCLCPP_ERROR(get_logger(), "The device name provided was invalid");
        throw ex;
    }

    try {
        baud_rate = declare_parameter<int>("baud_rate", 0);
    } catch (rclcpp::ParameterTypeException & ex) {
        RCLCPP_ERROR(get_logger(), "The baud_rate provided was invalid");
        throw ex;
    }

    try {
        const auto fc_string = declare_parameter<std::string>("flow_control", "");

        if (fc_string == "none") {
            fc = FlowControl::NONE;
        } else if (fc_string == "hardware") {
            fc = FlowControl::HARDWARE;
        } else if (fc_string == "software") {
            fc = FlowControl::SOFTWARE;
        } else {
            throw std::invalid_argument{
                "The flow_control parameter must be one of: none, software, or hardware."};
        }
    } catch (rclcpp::ParameterTypeException & ex) {
        RCLCPP_ERROR(get_logger(), "The flow_control provided was invalid");
        throw ex;
    }

    try {
        const auto pt_string = declare_parameter<std::string>("parity", "");

        if (pt_string == "none") {
            pt = Parity::NONE;
        } else if (pt_string == "odd") {
            pt = Parity::ODD;
        } else if (pt_string == "even") {
            pt = Parity::EVEN;
        } else {
            throw std::invalid_argument{"The parity parameter must be one of: none, odd, or even."};
        }
    } catch (rclcpp::ParameterTypeException & ex) {
        RCLCPP_ERROR(get_logger(), "The parity provided was invalid");
        throw ex;
    }

    try {
        const auto sb_string = declare_parameter<std::string>("stop_bits", "");

        if (sb_string == "1" || sb_string == "1.0") {
            sb = StopBits::ONE;
        } else if (sb_string == "1.5") {
            sb = StopBits::ONE_POINT_FIVE;
        } else if (sb_string == "2" || sb_string == "2.0") {
            sb = StopBits::TWO;
        } else {
            throw std::invalid_argument{"The stop_bits parameter must be one of: 1, 1.5, or 2."};
        }
    } catch (rclcpp::ParameterTypeException & ex) {
        RCLCPP_ERROR(get_logger(), "The stop_bits provided was invalid");
        throw ex;
    }

    device_config_ =
        std::make_unique<drivers::serial_driver::SerialPortConfig>(baud_rate, fc, pt, sb);
}

void ROS2_StandardRobotpp::receiveData()
{
    RCLCPP_INFO(get_logger(), "Start receiveData!");
    std::cout << "\033[32m Start receiveData! \033[0m" << std::endl;

    std::vector<uint8_t> sof(1);
    std::vector<uint8_t> receive_data;

    try {
        serial_driver_->init_port(device_name_, *device_config_);
        if (!serial_driver_->port()->is_open()) {
            serial_driver_->port()->open();
        }
    } catch (const std::exception & ex) {
        RCLCPP_ERROR(
            get_logger(), "Error creating serial port: %s - %s", device_name_.c_str(), ex.what());
        throw ex;
    }

    while (rclcpp::ok()) {
        try {
            serial_driver_->port()->receive(sof);

            if (sof[0] == SOF_RECEIVE) {
                std::vector<uint8_t> header_frame_buf(3);  // sof在读取完数据后添加

                serial_driver_->port()->receive(header_frame_buf);  // 读取除sof外剩下的数据
                header_frame_buf.insert(header_frame_buf.begin(), sof[0]);  // 添加sof
                HeaderFrame header_frame = fromVector<HeaderFrame>(header_frame_buf);

                // HeaderFrame CRC8 check
                bool crc8_ok = crc8::verify_CRC8_check_sum(
                    reinterpret_cast<uint8_t *>(&header_frame), sizeof(header_frame));
                if (crc8_ok) {
                    // 根据数据段长度读取数据
                    std::vector<uint8_t> data_buf(header_frame.len + 2);  // len + crc
                    serial_driver_->port()->receive(data_buf);
                    // 添加header_frame_buf到data_buf
                    data_buf.insert(
                        data_buf.begin(), header_frame_buf.begin(), header_frame_buf.end());

                    // // 整包数据校验
                    // bool crc16_ok = crc16::verify_CRC16_check_sum(data_buf);
                    // std::cout << "crc16:"
                    //           << uint16_t(
                    //                  (data_buf[data_buf.size() - 2] << 8) +
                    //                  data_buf[data_buf.size() - 1])
                    //           << std::endl;

                    // if (crc16_ok) {
                    //     std::cout << "\033[32m crc16_ok... \033[0m" << std::endl;
                    // } else {
                    //     std::cout << "\033[31m crc16 error... \033[0m" << std::endl;
                    // }

                    // 根据header_frame.id解析数据
                    switch (header_frame.id) {
                        case ID_DEBUG: {
                            ReceiveDebugData debug_data = fromVector<ReceiveDebugData>(data_buf);
                            // 整包数据校验
                            bool crc16_ok = crc16::verify_CRC16_check_sum(
                                reinterpret_cast<uint8_t *>(&debug_data), sizeof(ReceiveDebugData));
                            if (crc16_ok) {
                                std::cout << "\033[32m Decoded debug data. \033[0m" << std::endl;
                            } else {
                                RCLCPP_ERROR(get_logger(), "Debug data crc16 error!");
                            }
                        } break;
                        case ID_IMU: {
                            ReceiveImuData imu_data = fromVector<ReceiveImuData>(data_buf);

                            // 整包数据校验
                            bool crc16_ok = crc16::verify_CRC16_check_sum(
                                reinterpret_cast<uint8_t *>(&imu_data), sizeof(ReceiveImuData));
                            if (crc16_ok) {
                                std::cout << "\033[32m Decoded imu data. \033[0m" << std::endl;
                            } else {
                                RCLCPP_ERROR(get_logger(), "Imu data crc16 error!");
                            }
                        } break;
                        case ID_ROBOT_INFO: {
                            ReceiveRobotInfoData robot_info_data = fromVector<ReceiveRobotInfoData>(data_buf);

                            // 整包数据校验
                            bool crc16_ok = crc16::verify_CRC16_check_sum(
                                reinterpret_cast<uint8_t *>(&robot_info_data), sizeof(ReceiveRobotInfoData));
                            if (crc16_ok) {
                                std::cout << "\033[32m Decoded robot info data. \033[0m" << std::endl;
                            } else {
                                RCLCPP_ERROR(get_logger(), "Robot info data crc16 error!");
                            }
                        } break;
                        default: {
                            RCLCPP_WARN(get_logger(), "Invalid id: %d", header_frame.id);
                        } break;
                    }
                } else {
                    RCLCPP_ERROR(get_logger(), "Header frame CRC8 error!");
                }

            } else {
                continue;
            }
        } catch (const std::exception & ex) {
            RCLCPP_ERROR(get_logger(), "Error receiving data: %s", ex.what());
        }

        // thread sleep
        // std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

}  // namespace ros2_standard_robot_pp

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(ros2_standard_robot_pp::ROS2_StandardRobotpp)