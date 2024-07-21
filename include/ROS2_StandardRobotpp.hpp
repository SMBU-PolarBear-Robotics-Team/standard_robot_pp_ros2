/**
  ****************************(C) COPYRIGHT 2024 Polarbear*************************
  * @file       ROS2_StandardRobotpp.hpp/cpp
  * @brief      上下位机通信模块
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Jul-18-2024     Penguin         1. done
  @verbatim
  =================================================================================

  =================================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear*************************
  */
#ifndef ROS2_STANDARD_ROBOT_PP__ROS2_STANDARD_ROBOT_HPP_
#define ROS2_STANDARD_ROBOT_PP__ROS2_STANDARD_ROBOT_HPP_

#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <serial_driver/serial_driver.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker.hpp>
// C++ system

#include <future>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "packet_typedef.hpp"

namespace ros2_standard_robot_pp
{
class ROS2_StandardRobotpp : public rclcpp::Node
{
  public:
    explicit ROS2_StandardRobotpp(const rclcpp::NodeOptions & options);

    ~ROS2_StandardRobotpp() override;

  private:
    std::unique_ptr<IoContext> owned_ctx_;

    void getParams();

    // Debug data related
    std::unordered_map<std::string, rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr>
        debug_pub_map_;

    // Serial port
    std::string device_name_;
    std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config_;
    std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_;

    // Publish
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
        stm32_run_time_pub_;  // 发布STM32运行时间，数据基于接收到的imu数据时间戳
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> imu_tf_broadcaster_;  // 发布imu的tf用于可视化
    void createPublisher();
    void publishDebugData(ReceiveDebugData & debug_data);
    void publishImuData(ReceiveImuData & imu_data);

    // receive_thread
    std::thread receive_thread_;
    void receiveData();

    // send thread
    std::thread send_thread_;
    void sendData();

    // Serial port protect thread
    std::thread serial_port_protect_thread_;
    void serialPortProtect();
};
}  // namespace ros2_standard_robot_pp

#endif  // ROS2_STANDARD_ROBOT_PP__ROS2_STANDARD_ROBOT_HPP_