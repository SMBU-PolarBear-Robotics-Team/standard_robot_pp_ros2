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
#include <srpp_interfaces/msg/detail/robot_state_info__struct.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int64_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker.hpp>
// C++ system

#include <future>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "packet_typedef.hpp"
#include "robot_info.hpp"
#include "srpp_interfaces/msg/robot_state_info.hpp"

namespace ros2_standard_robot_pp
{
class ROS2_StandardRobotpp : public rclcpp::Node
{
  public:
    explicit ROS2_StandardRobotpp(const rclcpp::NodeOptions & options);

    ~ROS2_StandardRobotpp() override;

  private:
    rclcpp::Time node_start_time_stamp;
    RobotModels robot_models_;
    bool usb_is_ok_;

    void getParams();

    // Cmmand related
    SendRobotCmdData send_robot_cmd_data_;

    // Debug data related
    std::unordered_map<std::string, rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr>
        debug_pub_map_;

    // Serial port
    std::unique_ptr<IoContext> owned_ctx_;
    std::string device_name_;
    std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config_;
    std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_;

    // Publish
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<srpp_interfaces::msg::RobotStateInfo>::SharedPtr robot_state_info_pub_;
    rclcpp::Publisher<std_msgs::msg::Int64MultiArray>::SharedPtr all_robot_hp_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> imu_tf_broadcaster_;  // 发布imu的tf用于可视化
    void createPublisher();
    void createNewDebugPublisher(const std::string & name);
    void publishDebugData(ReceiveDebugData & debug_data);
    void publishImuData(ReceiveImuData & imu_data);
    void publishRobotStateInfo(ReceiveRobotInfoData & robot_info);
    void publishAllRobotHp(ReceiveAllRobotHpData & all_robot_hp);

    // Subscribe
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    void createSubscription();
    void updateCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg);

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