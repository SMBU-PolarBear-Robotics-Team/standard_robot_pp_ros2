#ifndef ROS2_STANDARD_ROBOT_PP__ROBOT_INFO_HPP_
#define ROS2_STANDARD_ROBOT_PP__ROBOT_INFO_HPP_

#include <string>

namespace ros2_standard_robot_pp
{
const int CHASSIS_MODEL_NUM = 5;
const int GIMBAL_MODEL_NUM = 2;
const int SHOOT_MODEL_NUM = 3;
const int ARM_MODEL_NUM = 2;
const int CUSTOM_CONTROLLER_MODEL_NUM = 2;

struct Models
{
    // clang-format off
    std::string chassis[CHASSIS_MODEL_NUM] = {
        "无底盘", 
        "麦轮底盘", 
        "全向轮底盘", 
        "舵轮底盘", 
        "平衡底盘"
    };
    std::string gimbal[GIMBAL_MODEL_NUM] = {
        "无云台", 
        "yaw_pitch直连云台"
    };
    std::string shoot[SHOOT_MODEL_NUM] = {
        "无发射机构", 
        "摩擦轮+拨弹盘", 
        "气动+拨弹盘"
    };
    std::string arm[ARM_MODEL_NUM] = {
        "无机械臂", 
        "mini机械臂"
    };
    std::string custom_controller[CUSTOM_CONTROLLER_MODEL_NUM] = {
        "无自定义控制器", 
        "mini自定义控制器"
    };
    // clang-format on
};

}  // namespace ros2_standard_robot_pp

#endif  // ROS2_STANDARD_ROBOT_PP__ROBOT_INFO_HPP_
