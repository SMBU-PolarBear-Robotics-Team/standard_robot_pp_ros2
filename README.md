# ROS2_StandardRobot++

<div align=center>

![Logo](https://gitee.com/SMBU-POLARBEAR/Organization_Information/blob/master/.pic/Logo6.png)

[![Author](https://img.shields.io/badge/Author-小企鹅-orange.svg)](https://gitee.com/Ljw0401)

![C++ Badge](https://img.shields.io/badge/-C%2B%2B-blue?style=flat&logo=c%2B%2B&logoColor=white)
![license](https://img.shields.io/badge/license-MIT-green.svg)

[![commit](https://svg.hamm.cn/gitee.svg?user=SMBU-POLARBEAR&project=ros2_standard_robot_pp&type=commit)](https://gitee.com/SMBU-POLARBEAR/ros2_standard_robot_pp)
[![fork](https://gitee.com/SMBU-POLARBEAR/ros2_standard_robot_pp/badge/fork.svg?theme=dark)](https://gitee.com/SMBU-POLARBEAR/ros2_standard_robot_pp)
[![star](https://gitee.com/SMBU-POLARBEAR/ros2_standard_robot_pp/badge/star.svg?theme=dark)](https://gitee.com/SMBU-POLARBEAR/ros2_standard_robot_pp)
[![release](https://svg.hamm.cn/gitee.svg?user=SMBU-POLARBEAR&project=ros2_standard_robot_pp&type=release)](https://gitee.com/SMBU-POLARBEAR/ros2_standard_robot_pp)

</div>

## 介绍

ROS2 StandardRobot++ 是配合 StandardRobot++ 下位机控制使用的机器人驱动，提供了机器人的控制接口、数据接口。

- **机器人控制接口：**
  - 运动的速度向量 vx vy wz
  - 云台的朝向 pitch yaw
  - 底盘的姿态 roll pitch yaw leg_length
  - 发射机构状态 fire fric_on
  - 话题:`\cmd_vel` : vx vy wz
- **机器人数据接口：**
  - ...

## 依赖

## 如何使用

1. [配置udev](./doc/appendix.md/#配置udev规则)，用来定向下位机串口硬件和给予串口权限
2. 构建程序（工作空间中打开命令行）

    ```shell
    colcon build --symlink-install
    ```

3. 运行程序

    ```shell
    source install/setup.sh
    ros2 launch ros2_standard_robot_pp ros2_standard_robot_pp.launch.py
    ```

## 更多玩法

### 可视化

配合 [foxglove](https://foxglove.dev/download) 可以实现机器人相关数据的可视化。

使用foxglove可视化：

  ```shell
  source install/setup.sh
  ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765
  ```

> layout文件夹中提供了一些foxglove的页面布局，可以导入直接使用。

### 遥控车

配合 [teleop_twist_keyboard](https://index.ros.org/p/teleop_twist_keyboard/github-ros2-teleop_twist_keyboard/#humble-overview) 使用键盘进行远程遥控。

安装 `teleop_twist_keyboard` :

```shell
sudo apt update
sudo apt install ros-humble-teleop-twist-keyboard
```

运行 `teleop_twist_keyboard` :

```shell
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## 附录

[appendix](./doc/appendix.md)

## 致谢

串口通信部分参考了rv serial_driver，通信协议参考dji裁判系统通信协议。
