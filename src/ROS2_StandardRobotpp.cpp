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

namespace ros2_standard_robot_pp
{
ROS2_StandardRobotpp::ROS2_StandardRobotpp(const rclcpp::NodeOptions & options)
: Node("ROS2_StandardRobotpp", options)
{
    RCLCPP_INFO(get_logger(), "Start RMSerialDriver!");

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

    //   if (serial_driver_->port()->is_open()) {
    //     serial_driver_->port()->close();
    //   }

    //   if (owned_ctx_) {
    //     owned_ctx_->waitForExit();
    //   }
}

void ROS2_StandardRobotpp::receiveData()
{
    RCLCPP_INFO(get_logger(), "Start receiveData!");
    while (true) {
        RCLCPP_INFO(get_logger(), "receiving...");
        // try {
        //     serial_driver_->port()->receive(header);

        //     switch (header[0]) {
        //         case 0x5A:
        //             receiveDataVision(header);
        //             break;
        //         case 0x5B:
        //             receiveDataAllRobotHP(header);
        //             break;
        //         case 0x5C:
        //             receiveDataGameStatus(header);
        //             break;
        //         case 0x5D:
        //             receiveDataRobotStatus(header);
        //             break;
        //         default:
        //             RCLCPP_WARN(get_logger(), "Unknown header received: %02X", header[0]);
        //             break;
        //     }
        // } catch (const std::exception & ex) {
        //     RCLCPP_ERROR(get_logger(), "Error receiving data: %s", ex.what());
        // }
    }
}

}  // namespace ros2_standard_robot_pp

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(ros2_standard_robot_pp::ROS2_StandardRobotpp)