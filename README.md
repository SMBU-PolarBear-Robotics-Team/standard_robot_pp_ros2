# ROS2_StandardRobot++

<div align=center>

![Logo](./.doc/logo.png)

[![Author](https://img.shields.io/badge/Author-小企鹅-orange.svg)](https://gitee.com/Ljw0401)
[![Maintainer](https://img.shields.io/badge/Maintainer-LihanChen2004-orange.svg)](https://gitee.com/LihanChen2004)

![C++ Badge](https://img.shields.io/badge/-C%2B%2B-blue?style=flat&logo=c%2B%2B&logoColor=white)
![license](https://img.shields.io/badge/license-MIT-green.svg)

[![commit](https://svg.hamm.cn/gitee.svg?user=SMBU-POLARBEAR&project=standard_robot_pp_ros2&type=commit)](https://gitee.com/SMBU-POLARBEAR/standard_robot_pp_ros2)
[![fork](https://gitee.com/SMBU-POLARBEAR/standard_robot_pp_ros2/badge/fork.svg?theme=dark)](https://gitee.com/SMBU-POLARBEAR/standard_robot_pp_ros2)
[![star](https://gitee.com/SMBU-POLARBEAR/standard_robot_pp_ros2/badge/star.svg?theme=dark)](https://gitee.com/SMBU-POLARBEAR/standard_robot_pp_ros2)
[![release](https://svg.hamm.cn/gitee.svg?user=SMBU-POLARBEAR&project=standard_robot_pp_ros2&type=release)](https://gitee.com/SMBU-POLARBEAR/standard_robot_pp_ros2)

</div>

## 1. 介绍

standard_robot_pp_ros2 是配合 [StandardRobot++](https://gitee.com/SMBU-POLARBEAR/StandardRobotpp.git) 下位机控制使用的机器人驱动，提供了机器人的控制接口、数据接口。

## 2. 协议结构

### 2.1 数据帧构成

|字段|长度 (Byte)|备注|
|:-:|:-:|:-:|
|frame_header|4|帧头|
|time_stamp|4|时间戳（基于下位机运行时间）|
|data|n|数据段|
|checksum|2|校验码|

### 2.2 帧头构成

|字段|长度 (Byte)|备注|
|:-:|:-:|:-:|
|sof|1|数据帧起始字节，固定值为 0x5A|
|len|1|数据段长度|
|id|1|数据段id|
|crc|1|数据帧头的 CRC8 校验|

### 2.3 data 数据段内容

详见飞书文档 [上下位机串口通信数据包](https://aafxu50hc35.feishu.cn/docx/HRh5dOjrMor4maxi3Xscvff6nCh?from=from_copylink)

## 3. 依赖

- Ubuntu: 22.04
- ROS2: Humble

- rosdep 安装依赖

    ```bash
    rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
    ```

- 自定义消息类型：[pb_rm_interfaces](https://gitee.com/SMBU-POLARBEAR/pb_rm_interfaces.git)

    ```sh
    git clone https://gitee.com/SMBU-POLARBEAR/pb_rm_interfaces.git
    ```

## 4. 使用方式

1. 配置 udev，用来定向下位机 RoboMaster C 型开发板串口硬件并给予串口权限

    ```bash
    ./create_udev_rules.sh
    ```

2. 构建程序

    ```bash
    colcon build --symlink-install
    ```

3. 运行程序

    ```bash
    ros2 launch standard_robot_pp_ros2 standard_robot_pp_ros2.launch.py
    ```

## 5. 致谢

串口通信部分参考了 [rm_vision - serial_driver](https://github.com/chenjunnn/rm_serial_driver.git)，通信协议参考 DJI 裁判系统通信协议。
