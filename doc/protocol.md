# 上下位机通信协议

> v1.0.0
>
> update 2024-11-19

- [上下位机通信协议](#上下位机通信协议)
  - [协议结构](#协议结构)
    - [数据帧构成](#数据帧构成)
    - [帧头构成](#帧头构成)
  - [上行数据](#上行数据)
    - [串口调试数据包](#串口调试数据包)
    - [IMU 数据包](#imu-数据包)
    - [机器人信息数据包](#机器人信息数据包)
    - [PID调参数据包](#pid调参数据包)
    - [全场机器人hp信息数据包](#全场机器人hp信息数据包)
    - [比赛信息数据包](#比赛信息数据包)
    - [机器人运动数据包](#机器人运动数据包)
  - [下行数据](#下行数据)
    - [机器人控制数据包](#机器人控制数据包)
    - [PID调参数据包\_](#pid调参数据包_)
    - [虚拟遥控器数据包](#虚拟遥控器数据包)

## 协议结构

### 数据帧构成

|字段|长度 (Byte)|备注|
|:-:|:-:|:-:|
|frame_header|4|帧头|
|time_stamp|4|时间戳(基于下位机运行时间)|
|data|n|数据段|
|checksum|2|校验码|

### 帧头构成

|字段|长度 (Byte)|备注|
|:-:|:-:|:-:|
|sof|1|数据帧起始字节，固定值为 0x5A|
|len|1|数据段长度|
|id|1|数据段id|
|crc|1|数据帧头的 CRC8 校验|

> 下文中仅讲述帧头部分的 id 和 数据段 data 的具体内容

## 上行数据

### 串口调试数据包

- id = 0x01
- 发送频率： 200 Hz

> data数据段内容为10个packages，每个package的内容如下：

|字段|类型|备注|
|:-:|:-:|:-:|
|name[10]|uint8_t|数据帧起始字节，固定值为 0x5A|
|type|uint8_t|数据类型 0-int,1-float|
|data|float|数据|

### IMU 数据包

- id = 0x02
- 发送频率： 200 Hz

> 坐标系为右手系
>
> - x轴对应roll轴
> - y轴对应pitch轴
> - z轴对应yaw轴

|字段|类型|备注|
|:-:|:-:|:-:|
|yaw|float|(rad)yaw轴角度|
|pitch|float|(rad)pitch轴角度|
|roll|float|(rad)roll轴角度|
|yaw_vel|float|(rad/s)yaw轴角速度|
|pitch_vel|float|(rad/s)pitch轴角速度|
|roll_vel|float|(rad/s)roll轴角速度|
<!-- |x_accel|float|(m/s^2)x轴加速度|
|y_accel|float|(m/s^2)y轴加速度|
|z_accel|float|(m/s^2)z轴加速度| -->

### 机器人信息数据包

- id = 0x03
- 发送频率： 10 Hz

|字段|长度 (Byte)|备注|
|:-:|:-:|:-:|
|type|2|机器人各个部位的类型 <br><br> chassis : 3 <br> gimbal : 3 <br> shoot : 3 <br> arm : 3 <br> custom_controller : 3 <br> reserve : 1|
|state|1|机器人各个部位的状态 <br> 0: 正常，1: 错误 <br><br> chassis : 1 <br> gimbal : 1 <br> shoot : 1 <br> arm : 1 <br> custom_controller : 1 <br> reserve : 3|
|referee|7| color 定义为 <br> 0-red 1-blue 2-unknown <br><br> uint8_t id <br> uint8_t color <br> bool attacked  <br> uint16_t hp <br> uint16_t heat|

### PID调参数据包

- id = 0x04
- 发送频率： 100 Hz

|字段|类型|备注|
|:-:|:-:|:-:|
|fdb|float|反馈值|
|ref|float|目标值|
|pid_out|float|pid输出值|

### 全场机器人hp信息数据包

- id = 0x05
- 发送频率： 10 Hz

|字段|类型|备注|
|:-:|:-:|:-:|
|red_1_robot_hp|uint16_t||
|red_2_robot_hp|uint16_t||
|red_3_robot_hp|uint16_t||
|red_4_robot_hp|uint16_t||
|red_5_robot_hp|uint16_t||
|red_7_robot_hp|uint16_t||
|red_outpost_hp|uint16_t||
|red_base_hp|uint16_t||
|blue_1_robot_hp|uint16_t||
|blue_2_robot_hp|uint16_t||
|blue_3_robot_hp|uint16_t||
|blue_4_robot_hp|uint16_t||
|blue_5_robot_hp|uint16_t||
|blue_7_robot_hp|uint16_t||
|blue_outpost_hp|uint16_t||
|blue_base_hp|uint16_t||

### 比赛信息数据包

- id = 0x06
- 发送频率： 10 Hz

|字段|类型|备注|
|:-:|:-:|:-:|
|game_progress|uint8_t||
|stage_remain_time|uint16_t||

### 机器人运动数据包

- id = 0x07
- 发送频率： 10 Hz

|字段|类型|备注|
|:-:|:-:|:-:|
|vx|float||
|vy|float||
|wz|float||

## 下行数据

### 机器人控制数据包

- id = 0x01
- 发送频率： 60 Hz

|字段|备注|
|:-:|:-:|
|speed_vector|速度向量|
|chassis|底盘姿态|
|gimbal|云台姿态|
|shoot|射击指令|

- speed_vector

  |字段|类型|备注|
  |:-:|:-:|:-:|
  |vx|float|速度向量|
  |vy|float|底盘姿态|
  |wz|float|云台姿态|

- chassis

  |字段|类型|备注|
  |:-:|:-:|:-:|
  |roll|float|roll角|
  |pitch|float|pitch角|
  |yaw|float|yaw角|
  |leg_lenth|float|腿长|

- gimbal

  |字段|类型|备注|
  |:-:|:-:|:-:|
  |pitch|float|pitch角|
  |yaw|float|yaw角|

- shoot

  |字段|类型|备注|
  |:-:|:-:|:-:|
  |fire|uint8_t|开火|
  |fric_on|uint8_t|摩擦轮启动|

### PID调参数据包_

- id = 0x02
- 发送频率： 60 Hz

|字段|类型|备注|
|:-:|:-:|:-:|
|kp|float|kp|
|ki|float|ki|
|kd|float|kd|
|max_out|float|max_out|
|max_iout|float|max_iout|

### 虚拟遥控器数据包

- id = 0x03
- 发送频率： 60 Hz

|字段|类型|备注|
|:-:|:-:|:-:|
|ch[5]|int16_t|遥控器通道值|
|s[2]|char|拨杆值|
|x|int16_t|鼠标x|
|y|int16_t|鼠标y|
|z|int16_t|鼠标z|
|press_l|uint8_t|鼠标左键按下|
|press_r|uint8_t|鼠标右键按下|
|v|uint16_t|键盘按键|
