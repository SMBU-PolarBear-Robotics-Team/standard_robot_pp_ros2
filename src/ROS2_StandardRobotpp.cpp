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
    createPublisher();

    // Create Subscription
    //   target_sub_ = this->create_subscription<auto_aim_interfaces::msg::Target>(
    //     "/tracker/target", rclcpp::SensorDataQoS(),
    //     std::bind(&RMSerialDriver::sendDataVision, this, std::placeholders::_1));
    //   cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    //     "/cmd_vel", 10, std::bind(&RMSerialDriver::sendDataTwist, this, std::placeholders::_1));

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

    serial_port_protect_thread_ = std::thread(&ROS2_StandardRobotpp::serialPortProtect, this);

    // 延时10ms
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    receive_thread_ = std::thread(&ROS2_StandardRobotpp::receiveData, this);
    send_thread_ = std::thread(&ROS2_StandardRobotpp::sendData, this);
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

/********************************************************/
/* Serial port protect                                  */
/********************************************************/

void ROS2_StandardRobotpp::serialPortProtect()
{
    RCLCPP_INFO(get_logger(), "Start serialPortProtect!");
    std::cout << "\033[32m Start serialPortProtect! \033[0m" << std::endl;

    ///@todo: 1.保持串口连接 2.串口断开重连 3.串口异常处理

    while (rclcpp::ok()) {
        try {
            std::cout << "protecting..." << std::endl;
            ;
        } catch (const std::exception & ex) {
            RCLCPP_ERROR(get_logger(), "Error receiving data: %s", ex.what());
        }

        // thread sleep
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

/********************************************************/
/* Receive data                                         */
/********************************************************/

void ROS2_StandardRobotpp::receiveData()
{
    RCLCPP_INFO(get_logger(), "Start receiveData!");
    std::cout << "\033[32m Start receiveData! \033[0m" << std::endl;

    std::vector<uint8_t> sof(1);
    std::vector<uint8_t> receive_data;

    int sof_count = 0;

    while (rclcpp::ok()) {
        try {
            serial_driver_->port()->receive(sof);

            if (sof[0] != SOF_RECEIVE) {
                sof_count++;
                std::cout << "Find sof, cnt=" << sof_count << std::endl;
                continue;
            }
            sof_count = 0;

            //### sof[0] == SOF_RECEIVE 后读取剩余 header_frame内容
            std::vector<uint8_t> header_frame_buf(3);  // sof在读取完数据后添加

            serial_driver_->port()->receive(header_frame_buf);  // 读取除sof外剩下的数据
            header_frame_buf.insert(header_frame_buf.begin(), sof[0]);  // 添加sof
            HeaderFrame header_frame = fromVector<HeaderFrame>(header_frame_buf);

            // HeaderFrame CRC8 check
            bool crc8_ok = crc8::verify_CRC8_check_sum(
                reinterpret_cast<uint8_t *>(&header_frame), sizeof(header_frame));
            if (!crc8_ok) {
                RCLCPP_ERROR(get_logger(), "Header frame CRC8 error!");
                continue;
            }

            //### crc8_ok 校验正确后读取数据段
            // 根据数据段长度读取数据
            std::vector<uint8_t> data_buf(header_frame.len + 2);  // len + crc
            int received_len = serial_driver_->port()->receive(data_buf);
            int received_len_sum = received_len;
            // 考虑到一次性读取数据可能存在数据量过大，读取不完整的情况。需要检测是否读取完整
            // 计算剩余未读取的数据长度
            int remain_len = header_frame.len + 2 - received_len;
            while (remain_len > 0) {  // 读取剩余未读取的数据
                std::vector<uint8_t> remain_buf(remain_len);
                received_len = serial_driver_->port()->receive(remain_buf);
                data_buf.insert(
                    data_buf.begin() + received_len_sum, remain_buf.begin(), remain_buf.end());
                received_len_sum += received_len;
                remain_len -= received_len;
            }

            // 数据段读取完成后添加header_frame_buf到data_buf，得到完整数据包
            data_buf.insert(data_buf.begin(), header_frame_buf.begin(), header_frame_buf.end());

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

                    publishDebugData(debug_data);
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

                    std_msgs::msg::Float64 stm32_run_time;
                    stm32_run_time.data = imu_data.time_stamp / 1000.0;
                    stm32_run_time_pub_->publish(stm32_run_time);

                    publishImuData(imu_data);
                } break;
                case ID_ROBOT_INFO: {
                    ReceiveRobotInfoData robot_info_data =
                        fromVector<ReceiveRobotInfoData>(data_buf);

                    // 整包数据校验
                    bool crc16_ok = crc16::verify_CRC16_check_sum(
                        reinterpret_cast<uint8_t *>(&robot_info_data),
                        sizeof(ReceiveRobotInfoData));
                    if (crc16_ok) {
                        std::cout << "\033[32m Decoded robot info data. \033[0m" << std::endl;
                    } else {
                        RCLCPP_ERROR(get_logger(), "Robot info data crc16 error!");
                    }
                    std::cout << "Timestamp:" << robot_info_data.time_stamp / 1000.0f << "s"
                              << std::endl;
                } break;
                default: {
                    RCLCPP_WARN(get_logger(), "Invalid id: %d", header_frame.id);
                } break;
            }

        } catch (const std::exception & ex) {
            RCLCPP_ERROR(get_logger(), "Error receiving data: %s", ex.what());
        }

        // thread sleep
        // std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

void ROS2_StandardRobotpp::createPublisher()
{
    stm32_run_time_pub_ = this->create_publisher<std_msgs::msg::Float64>("/stm32_run_time", 10);
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu", 10);

    imu_tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
}

void ROS2_StandardRobotpp::publishDebugData(ReceiveDebugData & received_debug_data)
{
    static rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr debug_pub;
    for (int i = 0; i < DEBUG_PACKAGE_NUM; i++) {
        // Create a vector to hold the non-zero data
        std::vector<uint8_t> non_zero_data;
        for (size_t j = 0; j < DEBUG_PACKAGE_NAME_LEN; j++) {
            if (received_debug_data.packages[i].name[j] != 0) {
                non_zero_data.push_back(received_debug_data.packages[i].name[j]);
            } else {
                break;
            }
        }
        // Convert the non-zero data to a string
        std::string name(non_zero_data.begin(), non_zero_data.end());

        if (name.empty()) {
            continue;
        }

        if (debug_pub_map_.find(name) != debug_pub_map_.end()) {  // The key is in the map
            debug_pub = debug_pub_map_.at(name);
        } else {  // The key is not in the map
            std::string topic_name = "/debug/" + name;
            // Create a new publisher
            debug_pub = this->create_publisher<std_msgs::msg::Float64>(topic_name, 10);
            debug_pub_map_.insert(std::make_pair(name, debug_pub));  // Create a new key-value pair
        }

        std_msgs::msg::Float64 debug_data;
        debug_data.data = received_debug_data.packages[i].data;
        debug_pub->publish(debug_data);
    }
}

void ROS2_StandardRobotpp::publishImuData(ReceiveImuData & imu_data)
{
    auto imu_msg = sensor_msgs::msg::Imu();
    // Convert Euler angles to quaternion
    tf2::Quaternion q;
    q.setRPY(imu_data.data.roll, imu_data.data.pitch, imu_data.data.yaw);
    // Set the header
    imu_msg.header.stamp.sec = imu_data.time_stamp / 1000;
    imu_msg.header.stamp.nanosec = (imu_data.time_stamp % 1000) * 1e6;
    imu_msg.header.frame_id = "odom";
    // Set the orientation
    imu_msg.orientation.x = q.x();
    imu_msg.orientation.y = q.y();
    imu_msg.orientation.z = q.z();
    imu_msg.orientation.w = q.w();
    // Set the angular velocity
    imu_msg.angular_velocity.x = imu_data.data.roll_vel;
    imu_msg.angular_velocity.y = imu_data.data.pitch_vel;
    imu_msg.angular_velocity.z = imu_data.data.yaw_vel;
    // Set the linear acceleration
    // imu_msg.linear_acceleration.x = imu_data.data.x_accel;
    // imu_msg.linear_acceleration.y = imu_data.data.y_accel;
    // imu_msg.linear_acceleration.z = imu_data.data.z_accel;
    // Publish the message
    imu_pub_->publish(imu_msg);

    // Publish the transform to visualize the IMU in Foxglove Studio
    geometry_msgs::msg::TransformStamped t;
    imu_msg.header.stamp.sec = imu_data.time_stamp / 1000;
    imu_msg.header.stamp.nanosec = (imu_data.time_stamp % 1000) * 1e6;
    t.header.frame_id = "odom";
    t.child_frame_id = "imu";
    t.transform.rotation = tf2::toMsg(q);
    imu_tf_broadcaster_->sendTransform(t);
}

/********************************************************/
/* Send data                                            */
/********************************************************/

void ROS2_StandardRobotpp::sendData()
{
    RCLCPP_INFO(get_logger(), "Start sendData!");
    std::cout << "\033[32m Start sendData! \033[0m" << std::endl;

    while (rclcpp::ok()) {
        try {
            std::cout << "sending..." << std::endl;
            ;
        } catch (const std::exception & ex) {
            RCLCPP_ERROR(get_logger(), "Error receiving data: %s", ex.what());
        }

        // thread sleep
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

}  // namespace ros2_standard_robot_pp

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(ros2_standard_robot_pp::ROS2_StandardRobotpp)