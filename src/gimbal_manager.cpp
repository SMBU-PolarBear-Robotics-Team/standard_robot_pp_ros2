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

#include "standard_robot_pp_ros2/gimbal_manager.hpp"

#include <chrono>
#include <cmath>

namespace standard_robot_pp_ros2
{

GimbalManagerNode::GimbalManagerNode(const rclcpp::NodeOptions & options)
: Node("gimbal_manager", options)
{
  RCLCPP_INFO(get_logger(), "Start GimbalManagerNode!");

  cmd_sub_ = this->create_subscription<pb_rm_interfaces::msg::GimbalCmd>(
    "cmd_gimbal", 10,
    std::bind(&GimbalManagerNode::gimbalCmdCallback, this, std::placeholders::_1));

  joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("cmd_gimbal_joint", 10);

  timer_ = create_wall_timer(std::chrono::milliseconds(10), [this]() {
    const auto current_time = now();
    if (state_.last_update.nanoseconds() == 0) {
      state_.last_update = current_time;
      return;
    }
    updateState((current_time - state_.last_update).seconds());
    state_.last_update = current_time;
  });
}

void GimbalManagerNode::gimbalCmdCallback(const pb_rm_interfaces::msg::GimbalCmd::SharedPtr msg)
{
  static auto last_msg = std::make_shared<pb_rm_interfaces::msg::GimbalCmd>();

  if (
    msg->pitch_type == last_msg->pitch_type && msg->yaw_type == last_msg->yaw_type &&
    msg->position == last_msg->position && msg->velocity == last_msg->velocity) {
    return;
  }

  *last_msg = *msg;

  if (msg->pitch_type == pb_rm_interfaces::msg::GimbalCmd::ABSOLUTE_ANGLE) {
    state_.pitch = msg->position.pitch;
    state_.pitch_ctrl.mode = ControlMode::POSITION;
  } else if (msg->pitch_type == pb_rm_interfaces::msg::GimbalCmd::VELOCITY) {
    state_.pitch_ctrl = {
      .mode = ControlMode::VELOCITY,
      .velocity = msg->velocity.pitch,
      .min_range = msg->velocity.pitch_min_range,
      .max_range = msg->velocity.pitch_max_range,
      .is_continuous =
        std::abs((msg->velocity.pitch_max_range - msg->velocity.pitch_min_range) - 2 * M_PI) <
        0.01};
  }

  if (msg->yaw_type == pb_rm_interfaces::msg::GimbalCmd::ABSOLUTE_ANGLE) {
    state_.yaw = msg->position.yaw;
    state_.yaw_ctrl.mode = ControlMode::POSITION;
  } else if (msg->yaw_type == pb_rm_interfaces::msg::GimbalCmd::VELOCITY) {
    state_.yaw_ctrl = {
      .mode = ControlMode::VELOCITY,
      .velocity = msg->velocity.yaw,
      .min_range = msg->velocity.yaw_min_range,
      .max_range = msg->velocity.yaw_max_range,
      .is_continuous =
        std::abs((msg->velocity.yaw_max_range - msg->velocity.yaw_min_range) - 2 * M_PI) < 0.01};
  }
}

void GimbalManagerNode::updateState(double delta_time)
{
  if (state_.pitch_ctrl.mode == ControlMode::VELOCITY) {
    state_.pitch = updateAxisPosition(state_.pitch, state_.pitch_ctrl, delta_time);
  }
  if (state_.yaw_ctrl.mode == ControlMode::VELOCITY) {
    state_.yaw = updateAxisPosition(state_.yaw, state_.yaw_ctrl, delta_time);
  }
  publishJointState();
}

double GimbalManagerNode::updateAxisPosition(double current, AxisControl & ctrl, double delta)
{
  const double displacement = ctrl.velocity * delta;
  double new_pos = current + displacement;

  if (ctrl.is_continuous) {
    const double range = ctrl.max_range - ctrl.min_range;
    new_pos = ctrl.min_range + std::fmod(new_pos - ctrl.min_range, range);
    return (new_pos < ctrl.min_range) ? new_pos + range : new_pos;
  }

  if (new_pos > ctrl.max_range) {
    ctrl.velocity = -ctrl.velocity;
    return ctrl.max_range;
  }
  if (new_pos < ctrl.min_range) {
    ctrl.velocity = -ctrl.velocity;
    return ctrl.min_range;
  }
  return new_pos;
}

void GimbalManagerNode::publishJointState()
{
  auto msg = sensor_msgs::msg::JointState();
  msg.header.stamp = now();
  msg.name = {"gimbal_pitch_joint", "gimbal_yaw_joint"};
  msg.position = {state_.pitch, state_.yaw};
  joint_pub_->publish(msg);
}
}  // namespace standard_robot_pp_ros2

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(standard_robot_pp_ros2::GimbalManagerNode)
