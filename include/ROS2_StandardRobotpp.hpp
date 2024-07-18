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

namespace ros2_standard_robot_pp
{
class ROS2_StandardRobotpp : public rclcpp::Node
{
  public:
    explicit ROS2_StandardRobotpp(const rclcpp::NodeOptions & options);

    ~ROS2_StandardRobotpp() override;

  private:
    // receive_thread
    std::thread receive_thread_;
    void receiveData();
};
}  // namespace ros2_standard_robot_pp

#endif  // ROS2_STANDARD_ROBOT_PP__ROS2_STANDARD_ROBOT_HPP_