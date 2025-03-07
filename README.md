# standard_robot_pp_ros2

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![Build and Test](https://github.com/SMBU-PolarBear-Robotics-Team/standard_robot_pp_ros2/actions/workflows/ci.yml/badge.svg)](https://github.com/SMBU-PolarBear-Robotics-Team/standard_robot_pp_ros2/actions/workflows/ci.yml)

![PolarBear Logo](https://raw.githubusercontent.com/SMBU-PolarBear-Robotics-Team/.github/main/.docs/image/polarbear_logo_text.png)

## 1. Introduction

standard_robot_pp_ros2 是配合 [StandardRobot++](https://gitee.com/SMBU-POLARBEAR/StandardRobotpp.git) 下位机控制使用的机器人驱动，提供了机器人的控制接口、数据接口。

本项目获取下位机的 packet 并发布为 topic，并将下位机处理后的动态关节信息数据发布到 `joint_states` 话题，通过 [joint_state_publisher](https://github.com/ros/joint_state_publisher/tree/ros2/joint_state_publisher) 和 [robot_state_publisher](https://github.com/ros/robot_state_publisher/tree/humble) 建立整车 TF 树（包含 static 和 dynamic）。

![frames](https://raw.githubusercontent.com/LihanChen2004/picx-images-hosting/master/frames.5xaq4wriyy.webp)

## 2. Quick Start

### 2.1 Setup Environment

- Ubuntu 22.04
- ROS: [Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)

### 2.2 Create Workspace

```bash
sudo pip install vcstool2
pip install xmacro
```

```bash
mkdir -p ~/ros_ws
cd ~/ros_ws
```

```bash
git clone https://github.com/SMBU-PolarBear-Robotics-Team/standard_robot_pp_ros2.git src/standard_robot_pp_ros2
```

```bash
vcs import src < src/standard_robot_pp_ros2/dependencies.repos
vcs import src < src/pb2025_robot_description/dependencies.repos
```

### 2.3 Build

```bash
rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
```

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### 2.4 Running

1. 配置 udev，用来定向下位机 RoboMaster C 型开发板串口硬件并给予串口权限

    > 本命令在一台主机中只需要运行一次，无需重复运行。

    ```bash
    ./script/create_udev_rules.sh
    ```

2. 构建程序

    ```bash
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=release
    ```

3. 运行上下位机通讯

    Tips: 如需开启 RViz 可视化，请添加 `use_rviz:=True` 参数。

    ```bash
    ros2 launch standard_robot_pp_ros2 standard_robot_pp_ros2.launch.py
    ```

### 2.5 Launch Arguments

| 参数 | 描述 | 类型 | 默认值 |
|-|-|-|-|
| `namespace` | 顶级命名空间 | string | "" |
| `params_file` | 用于所有启动节点的 ROS2 参数文件的完整路径 | string | [vision_params.yaml](./config/standard_robot_pp_ros2.yaml) |
| `robot_name` | 要使用的机器人 xmacro 文件名 | string | "pb2025_sentry_robot" |
| `use_rviz` | 是否启动 RViz | bool | True |
| `use_respawn` | 如果节点崩溃，是否重新启动。本参数仅 `use_composition:=False` 时有效 | bool | False |
| `log_level` | 日志级别 | string | "info" |

## 3. 协议结构

### 3.1 数据帧构成

|字段|长度 (Byte)|备注|
|:-:|:-:|:-:|
|frame_header|4|帧头|
|time_stamp|4|时间戳（基于下位机运行时间）|
|data|n|数据段|
|checksum|2|校验码|

### 3.2 帧头构成

|字段|长度 (Byte)|备注|
|:-:|:-:|:-:|
|sof|1|数据帧起始字节，固定值为 0x5A|
|len|1|数据段长度|
|id|1|数据段id|
|crc|1|数据帧头的 CRC8 校验|

### 3.3 data 数据段内容

详见飞书文档 [上下位机串口通信数据包](https://aafxu50hc35.feishu.cn/docx/HRh5dOjrMor4maxi3Xscvff6nCh?from=from_copylink)

## 4. 致谢

串口通信部分参考了 [rm_vision - serial_driver](https://github.com/chenjunnn/rm_serial_driver.git)，通信协议参考 DJI 裁判系统通信协议。
