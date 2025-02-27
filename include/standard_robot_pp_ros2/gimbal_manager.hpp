// Copyright 2025 SMBU-PolarBear-Robotics-Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef STANDARD_ROBOT_PP_ROS2__GIMBAL_MANAGER_HPP_
#define STANDARD_ROBOT_PP_ROS2__GIMBAL_MANAGER_HPP_

#include "pb_rm_interfaces/msg/gimbal_cmd.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace standard_robot_pp_ros2
{

enum class ControlMode { POSITION, VELOCITY };

struct AxisControl
{
  ControlMode mode = ControlMode::POSITION;
  float velocity = 0.0f;       // Desired velocity (rad/s)
  float min_range = 0.0f;      // Minimum angle limit (rad)
  float max_range = 0.0f;      // Maximum angle limit (rad)
  bool is_continuous = false;  // Continuous rotation mode
};

class GimbalManagerNode : public rclcpp::Node
{
public:
  explicit GimbalManagerNode(const rclcpp::NodeOptions & options);
  ~GimbalManagerNode() override = default;

private:
  void gimbalCmdCallback(const pb_rm_interfaces::msg::GimbalCmd::SharedPtr msg);

  void updateState(double delta_time);

  void publishJointState();

  double updateAxisPosition(double current, AxisControl & ctrl, double delta);

  struct
  {
    double pitch = 0.0;
    double yaw = 0.0;
    AxisControl pitch_ctrl;
    AxisControl yaw_ctrl;
    rclcpp::Time last_update;
  } state_;

  rclcpp::Subscription<pb_rm_interfaces::msg::GimbalCmd>::SharedPtr cmd_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace standard_robot_pp_ros2

#endif  // STANDARD_ROBOT_PP_ROS2__GIMBAL_MANAGER_HPP_
