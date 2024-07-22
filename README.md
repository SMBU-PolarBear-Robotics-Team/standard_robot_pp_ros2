# ROS2_StandardRobot++

![Logo]()

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

1. [配置udev](./doc/appendix.md/#配置udev规则)，用来定向下位机串口硬件
2. 构建程序（工作空间中打开命令行）

    ```shell
    colcon build --symlink-install
    ```

3. 开放串口使用权限

    ```shell
    sudo chmod 777 /dev/ttyACM0
    ```

4. 运行程序

    ```shell
    source install/setup.sh
    ros2 launch ros2_standard_robot_pp ros2_standard_robot_pp.launch.py
    ```
5. 使用foxglove可视化（打开一个新命令行）

    ```shell
    source install/setup.sh
    ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765
    ```

## 更多玩法

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
