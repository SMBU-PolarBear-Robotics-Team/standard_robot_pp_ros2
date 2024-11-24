# pb_rm_interfaces

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

ROS2 interfaces (.msg, .srv, .action) used in the StandardRobot++ project.

## msg

云台和射击使用自定义消息类型，

* GimbalCmd.msg：云台控制命令，使用绝对位置，单位为弧度
* ShootCmd.msg：射击命令，包含射击子弹数
* 底盘控制命令使用 ROS2 的 `geometry_msgs/msg/Twist`。
* referee

    当前对应串口协议版本：[V1.6.4（20240715）](https://terra-1-g.djicdn.com/b2a076471c6c4b72b574a977334d3e05/RM2024/RoboMaster%20%E8%A3%81%E5%88%A4%E7%B3%BB%E7%BB%9F%E4%B8%B2%E5%8F%A3%E5%8D%8F%E8%AE%AE%E9%99%84%E5%BD%95%20V1.6.4%EF%BC%8820240715%EF%BC%89.pdf)
