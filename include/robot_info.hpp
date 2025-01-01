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

#ifndef STANDARD_ROBOT_PP_ROS2__ROBOT_INFO_HPP_
#define STANDARD_ROBOT_PP_ROS2__ROBOT_INFO_HPP_

#include <map>
#include <string>

namespace standard_robot_pp_ros2
{
const int CHASSIS_MODEL_NUM = 5;
const int GIMBAL_MODEL_NUM = 2;
const int SHOOT_MODEL_NUM = 3;
const int ARM_MODEL_NUM = 2;
const int CUSTOM_CONTROLLER_MODEL_NUM = 2;

struct RobotModels
{
  std::map<uint8_t, std::string> chassis;
  std::map<uint8_t, std::string> gimbal;
  std::map<uint8_t, std::string> shoot;
  std::map<uint8_t, std::string> arm;
  std::map<uint8_t, std::string> custom_controller;
};

}  // namespace standard_robot_pp_ros2

#endif  // STANDARD_ROBOT_PP_ROS2__ROBOT_INFO_HPP_
