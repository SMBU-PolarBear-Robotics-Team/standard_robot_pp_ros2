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

#include "standard_robot_pp_ros2/standard_robot_pp_ros2.hpp"

#include <memory>

#include "standard_robot_pp_ros2/crc8_crc16.hpp"
#include "standard_robot_pp_ros2/packet_typedef.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#define USB_NOT_OK_SLEEP_TIME 1000   // (ms)
#define USB_PROTECT_SLEEP_TIME 1000  // (ms)

using namespace std::chrono_literals;

namespace standard_robot_pp_ros2
{

StandardRobotPpRos2Node::StandardRobotPpRos2Node(const rclcpp::NodeOptions & options)
: Node("StandardRobotPpRos2Node", options),
  owned_ctx_{new IoContext(2)},
  serial_driver_{new drivers::serial_driver::SerialDriver(*owned_ctx_)}
{
  RCLCPP_INFO(get_logger(), "Start StandardRobotPpRos2Node!");

  getParams();
  createPublisher();
  createSubscription();

  robot_models_.chassis = {
    {0, "无底盘"}, {1, "麦轮底盘"}, {2, "全向轮底盘"}, {3, "舵轮底盘"}, {4, "平衡底盘"}};
  robot_models_.gimbal = {{0, "无云台"}, {1, "yaw_pitch直连云台"}};
  robot_models_.shoot = {{0, "无发射机构"}, {1, "摩擦轮+拨弹盘"}, {2, "气动+拨弹盘"}};
  robot_models_.arm = {{0, "无机械臂"}, {1, "mini机械臂"}};
  robot_models_.custom_controller = {{0, "无自定义控制器"}, {1, "mini自定义控制器"}};

  serial_port_protect_thread_ = std::thread(&StandardRobotPpRos2Node::serialPortProtect, this);
  receive_thread_ = std::thread(&StandardRobotPpRos2Node::receiveData, this);
  send_thread_ = std::thread(&StandardRobotPpRos2Node::sendData, this);
}

StandardRobotPpRos2Node::~StandardRobotPpRos2Node()
{
  if (send_thread_.joinable()) {
    send_thread_.join();
  }

  if (receive_thread_.joinable()) {
    receive_thread_.join();
  }

  if (serial_port_protect_thread_.joinable()) {
    serial_port_protect_thread_.join();
  }

  if (serial_driver_->port()->is_open()) {
    serial_driver_->port()->close();
  }

  if (owned_ctx_) {
    owned_ctx_->waitForExit();
  }
}

void StandardRobotPpRos2Node::createPublisher()
{
  imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("serial/imu", 10);
  robot_state_info_pub_ =
    this->create_publisher<pb_rm_interfaces::msg::RobotStateInfo>("serial/robot_state_info", 10);
  joint_state_pub_ =
    this->create_publisher<sensor_msgs::msg::JointState>("serial/gimbal_joint_state", 10);
  robot_motion_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("serial/robot_motion", 10);
  event_data_pub_ =
    this->create_publisher<pb_rm_interfaces::msg::EventData>("referee/event_data", 10);
  all_robot_hp_pub_ =
    this->create_publisher<pb_rm_interfaces::msg::GameRobotHP>("referee/all_robot_hp", 10);
  game_status_pub_ =
    this->create_publisher<pb_rm_interfaces::msg::GameStatus>("referee/game_status", 10);
  ground_robot_position_pub_ = this->create_publisher<pb_rm_interfaces::msg::GroundRobotPosition>(
    "referee/ground_robot_position", 10);
  rfid_status_pub_ =
    this->create_publisher<pb_rm_interfaces::msg::RfidStatus>("referee/rfid_status", 10);
  robot_status_pub_ =
    this->create_publisher<pb_rm_interfaces::msg::RobotStatus>("referee/robot_status", 10);
  buff_pub_ = this->create_publisher<pb_rm_interfaces::msg::Buff>("referee/buff", 10);
}

void StandardRobotPpRos2Node::createNewDebugPublisher(const std::string & name)
{
  RCLCPP_INFO(get_logger(), "Create new debug publisher: %s", name.c_str());
  std::string topic_name = "serial/debug/" + name;
  auto debug_pub = this->create_publisher<example_interfaces::msg::Float64>(topic_name, 10);
  debug_pub_map_.insert(std::make_pair(name, debug_pub));
}

void StandardRobotPpRos2Node::createSubscription()
{
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 10,
    std::bind(&StandardRobotPpRos2Node::cmdVelCallback, this, std::placeholders::_1));

  cmd_gimbal_joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "cmd_gimbal_joint", 10,
    std::bind(&StandardRobotPpRos2Node::cmdGimbalJointCallback, this, std::placeholders::_1));

  cmd_shoot_sub_ = this->create_subscription<example_interfaces::msg::UInt8>(
    "cmd_shoot", 10,
    std::bind(&StandardRobotPpRos2Node::cmdShootCallback, this, std::placeholders::_1));
  cmd_tracking_sub_ = this->create_subscription<auto_aim_interfaces::msg::Target>(
    "tracker/target", 10,
    std::bind(&StandardRobotPpRos2Node::visionTargetCallback, this, std::placeholders::_1));
}

void StandardRobotPpRos2Node::getParams()
{
  using FlowControl = drivers::serial_driver::FlowControl;
  using Parity = drivers::serial_driver::Parity;
  using StopBits = drivers::serial_driver::StopBits;

  uint32_t baud_rate{};
  auto fc = FlowControl::NONE;
  auto pt = Parity::NONE;
  auto sb = StopBits::ONE;

  try {
    device_name_ = declare_parameter<std::string>("device_name", "");
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The device name provided was invalid");
    throw ex;
  }

  try {
    baud_rate = declare_parameter<int>("baud_rate", 0);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The baud_rate provided was invalid");
    throw ex;
  }

  try {
    const auto fc_string = declare_parameter<std::string>("flow_control", "");

    if (fc_string == "none") {
      fc = FlowControl::NONE;
    } else if (fc_string == "hardware") {
      fc = FlowControl::HARDWARE;
    } else if (fc_string == "software") {
      fc = FlowControl::SOFTWARE;
    } else {
      throw std::invalid_argument{
        "The flow_control parameter must be one of: none, software, or hardware."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The flow_control provided was invalid");
    throw ex;
  }

  try {
    const auto pt_string = declare_parameter<std::string>("parity", "");

    if (pt_string == "none") {
      pt = Parity::NONE;
    } else if (pt_string == "odd") {
      pt = Parity::ODD;
    } else if (pt_string == "even") {
      pt = Parity::EVEN;
    } else {
      throw std::invalid_argument{"The parity parameter must be one of: none, odd, or even."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The parity provided was invalid");
    throw ex;
  }

  try {
    const auto sb_string = declare_parameter<std::string>("stop_bits", "");

    if (sb_string == "1" || sb_string == "1.0") {
      sb = StopBits::ONE;
    } else if (sb_string == "1.5") {
      sb = StopBits::ONE_POINT_FIVE;
    } else if (sb_string == "2" || sb_string == "2.0") {
      sb = StopBits::TWO;
    } else {
      throw std::invalid_argument{"The stop_bits parameter must be one of: 1, 1.5, or 2."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The stop_bits provided was invalid");
    throw ex;
  }

  device_config_ =
    std::make_unique<drivers::serial_driver::SerialPortConfig>(baud_rate, fc, pt, sb);

  record_rosbag_ = declare_parameter("record_rosbag", false);
  set_detector_color_ = declare_parameter("set_detector_color", false);
  debug_ = declare_parameter("debug", false);
}

/********************************************************/
/* Serial port protect                                  */
/********************************************************/
void StandardRobotPpRos2Node::serialPortProtect()
{
  RCLCPP_INFO(get_logger(), "Start serialPortProtect!");

  // @TODO: 1.保持串口连接 2.串口断开重连 3.串口异常处理

  // 初始化串口
  serial_driver_->init_port(device_name_, *device_config_);
  // 尝试打开串口
  try {
    if (!serial_driver_->port()->is_open()) {
      serial_driver_->port()->open();
      RCLCPP_INFO(get_logger(), "Serial port opened!");
      is_usb_ok_ = true;
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Open serial port failed : %s", ex.what());
    is_usb_ok_ = false;
  }

  is_usb_ok_ = true;
  std::this_thread::sleep_for(std::chrono::milliseconds(USB_PROTECT_SLEEP_TIME));

  while (rclcpp::ok()) {
    if (!is_usb_ok_) {
      try {
        if (serial_driver_->port()->is_open()) {
          serial_driver_->port()->close();
        }

        serial_driver_->port()->open();

        if (serial_driver_->port()->is_open()) {
          RCLCPP_INFO(get_logger(), "Serial port opened!");
          is_usb_ok_ = true;
        }
      } catch (const std::exception & ex) {
        is_usb_ok_ = false;
        RCLCPP_ERROR(get_logger(), "Open serial port failed : %s", ex.what());
      }
    }

    // thread sleep
    std::this_thread::sleep_for(std::chrono::milliseconds(USB_PROTECT_SLEEP_TIME));
  }
}

/********************************************************/
/* Receive data                                         */
/********************************************************/

void StandardRobotPpRos2Node::receiveData()
{
  RCLCPP_INFO(get_logger(), "Start receiveData!");

  std::vector<uint8_t> sof(1);
  std::vector<uint8_t> receive_data;

  int sof_count = 0;
  int retry_count = 0;

  while (rclcpp::ok()) {
    if (!is_usb_ok_) {
      RCLCPP_WARN(get_logger(), "receive: usb is not ok! Retry count: %d", retry_count++);
      std::this_thread::sleep_for(std::chrono::milliseconds(USB_NOT_OK_SLEEP_TIME));
      continue;
    }

    try {
      serial_driver_->port()->receive(sof);

      if (sof[0] != SOF_RECEIVE) {
        sof_count++;
        RCLCPP_INFO(get_logger(), "Find sof, cnt=%d", sof_count);
        continue;
      }

      // Reset sof_count when SOF_RECEIVE is found
      sof_count = 0;

      // sof[0] == SOF_RECEIVE 后读取剩余 header_frame 内容
      std::vector<uint8_t> header_frame_buf(3);  // sof 在读取完数据后添加

      serial_driver_->port()->receive(header_frame_buf);  // 读取除 sof 外剩下的数据
      header_frame_buf.insert(header_frame_buf.begin(), sof[0]);  // 添加 sof
      HeaderFrame header_frame = fromVector<HeaderFrame>(header_frame_buf);

      // HeaderFrame CRC8 check
      bool crc8_ok = crc8::verify_CRC8_check_sum(
        reinterpret_cast<uint8_t *>(&header_frame), sizeof(header_frame));
      if (!crc8_ok) {
        RCLCPP_ERROR(get_logger(), "Header frame CRC8 error!");
        continue;
      }

      // crc8_ok 校验正确后读取数据段
      // 根据数据段长度读取数据
      std::vector<uint8_t> data_buf(header_frame.len + 2);  // len + crc
      int received_len = serial_driver_->port()->receive(data_buf);
      int received_len_sum = received_len;
      // 考虑到一次性读取数据可能存在数据量过大，读取不完整的情况。需要检测是否读取完整
      // 计算剩余未读取的数据长度
      int remain_len = header_frame.len + 2 - received_len;
      while (remain_len > 0) {  // 读取剩余未读取的数据
        std::vector<uint8_t> remain_buf(remain_len);
        received_len = serial_driver_->port()->receive(remain_buf);
        data_buf.insert(data_buf.begin() + received_len_sum, remain_buf.begin(), remain_buf.end());
        received_len_sum += received_len;
        remain_len -= received_len;
      }

      // 数据段读取完成后添加 header_frame_buf 到 data_buf，得到完整数据包
      data_buf.insert(data_buf.begin(), header_frame_buf.begin(), header_frame_buf.end());

      if (!debug_ && header_frame.id == ID_DEBUG) {
        continue;
      }

      // 整包数据校验
      bool crc16_ok = crc16::verify_CRC16_check_sum(data_buf);
      if (!crc16_ok) {
        RCLCPP_ERROR(get_logger(), "Data segment CRC16 error!");
        continue;
      }

      // crc16_ok 校验正确后根据 header_frame.id 解析数据
      switch (header_frame.id) {
        case ID_DEBUG: {
          ReceiveDebugData debug_data = fromVector<ReceiveDebugData>(data_buf);
          publishDebugData(debug_data);
        } break;
        case ID_IMU: {
          ReceiveImuData imu_data = fromVector<ReceiveImuData>(data_buf);
          publishImuData(imu_data);
        } break;
        case ID_ROBOT_STATE_INFO: {
          ReceiveRobotInfoData robot_info_data = fromVector<ReceiveRobotInfoData>(data_buf);
          publishRobotInfo(robot_info_data);
        } break;
        case ID_EVENT_DATA: {
          ReceiveEventData event_data = fromVector<ReceiveEventData>(data_buf);
          publishEventData(event_data);
        } break;
        case ID_PID_DEBUG: {
          RCLCPP_WARN(get_logger(), "Not implemented yet!");
        } break;
        case ID_ALL_ROBOT_HP: {
          ReceiveAllRobotHpData all_robot_hp_data = fromVector<ReceiveAllRobotHpData>(data_buf);
          publishAllRobotHp(all_robot_hp_data);
        } break;
        case ID_GAME_STATUS: {
          ReceiveGameStatusData game_status_data = fromVector<ReceiveGameStatusData>(data_buf);
          publishGameStatus(game_status_data);
        } break;
        case ID_ROBOT_MOTION: {
          ReceiveRobotMotionData robot_motion_data = fromVector<ReceiveRobotMotionData>(data_buf);
          publishRobotMotion(robot_motion_data);
        } break;
        case ID_GROUND_ROBOT_POSITION: {
          ReceiveGroundRobotPosition ground_robot_position_data =
            fromVector<ReceiveGroundRobotPosition>(data_buf);
          publishGroundRobotPosition(ground_robot_position_data);
        } break;
        case ID_RFID_STATUS: {
          ReceiveRfidStatus rfid_status_data = fromVector<ReceiveRfidStatus>(data_buf);
          publishRfidStatus(rfid_status_data);
        } break;
        case ID_ROBOT_STATUS: {
          ReceiveRobotStatus robot_status_data = fromVector<ReceiveRobotStatus>(data_buf);
          publishRobotStatus(robot_status_data);
        } break;
        case ID_JOINT_STATE: {
          ReceiveJointState joint_state_data = fromVector<ReceiveJointState>(data_buf);
          publishJointState(joint_state_data);
        } break;
        case ID_BUFF: {
          ReceiveBuff buff = fromVector<ReceiveBuff>(data_buf);
          publishBuff(buff);
        } break;
        default: {
          RCLCPP_WARN(get_logger(), "Invalid id: %d", header_frame.id);
        } break;
      }
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(get_logger(), "Error receiving data: %s", ex.what());
      is_usb_ok_ = false;
    }
  }
}

void StandardRobotPpRos2Node::publishDebugData(ReceiveDebugData & received_debug_data)
{
  static rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr debug_pub;
  for (auto & package : received_debug_data.packages) {
    // Create a vector to hold the non-zero data
    std::vector<uint8_t> non_zero_data;
    for (unsigned char name : package.name) {
      if (name != 0) {
        non_zero_data.push_back(name);
      } else {
        break;
      }
    }
    // Convert the non-zero data to a string
    std::string name(non_zero_data.begin(), non_zero_data.end());

    if (name.empty()) {
      continue;
    }

    if (debug_pub_map_.find(name) == debug_pub_map_.end()) {
      createNewDebugPublisher(name);
    }
    debug_pub = debug_pub_map_.at(name);

    example_interfaces::msg::Float64 msg;
    msg.data = package.data;
    debug_pub->publish(msg);
  }
}

void StandardRobotPpRos2Node::publishImuData(ReceiveImuData & imu_data)
{
  sensor_msgs::msg::JointState joint_msg;
  sensor_msgs::msg::Imu imu_msg;
  imu_msg.header.stamp = joint_msg.header.stamp = now();
  imu_msg.header.frame_id = "gimbal_pitch_odom";

  // Convert Euler angles to quaternion
  tf2::Quaternion q;
  q.setRPY(imu_data.data.roll, imu_data.data.pitch, imu_data.data.yaw);
  imu_msg.orientation = tf2::toMsg(q);
  imu_msg.angular_velocity.x = imu_data.data.roll_vel;
  imu_msg.angular_velocity.y = imu_data.data.pitch_vel;
  imu_msg.angular_velocity.z = imu_data.data.yaw_vel;
  imu_pub_->publish(imu_msg);

  joint_msg.name = {
    "gimbal_pitch_joint",
    "gimbal_yaw_joint",
    "gimbal_pitch_odom_joint",
    "gimbal_yaw_odom_joint",
  };
  joint_msg.position = {
    imu_data.data.pitch,
    imu_data.data.yaw,
    last_gimbal_pitch_odom_joint_,
    last_gimbal_yaw_odom_joint_,
  };
  joint_state_pub_->publish(joint_msg);
}

void StandardRobotPpRos2Node::publishRobotInfo(ReceiveRobotInfoData & robot_info)
{
  pb_rm_interfaces::msg::RobotStateInfo msg;

  msg.header.stamp.sec = robot_info.time_stamp / 1000;
  msg.header.stamp.nanosec = (robot_info.time_stamp % 1000) * 1e6;
  msg.header.frame_id = "odom";

  msg.models.chassis = robot_models_.chassis.at(robot_info.data.type.chassis);
  msg.models.gimbal = robot_models_.gimbal.at(robot_info.data.type.gimbal);
  msg.models.shoot = robot_models_.shoot.at(robot_info.data.type.shoot);
  msg.models.arm = robot_models_.arm.at(robot_info.data.type.arm);
  msg.models.custom_controller =
    robot_models_.custom_controller.at(robot_info.data.type.custom_controller);

  robot_state_info_pub_->publish(msg);
}

void StandardRobotPpRos2Node::publishEventData(ReceiveEventData & event_data)
{
  pb_rm_interfaces::msg::EventData msg;

  msg.non_overlapping_supply_zone = event_data.data.non_overlapping_supply_zone;
  msg.overlapping_supply_zone = event_data.data.overlapping_supply_zone;
  msg.supply_zone = event_data.data.supply_zone;

  msg.small_energy = event_data.data.small_energy;
  msg.big_energy = event_data.data.big_energy;

  msg.central_highland = event_data.data.central_highland;
  msg.trapezoidal_highland = event_data.data.trapezoidal_highland;

  msg.center_gain_zone = event_data.data.center_gain_zone;

  event_data_pub_->publish(msg);
}

void StandardRobotPpRos2Node::publishAllRobotHp(ReceiveAllRobotHpData & all_robot_hp)
{
  pb_rm_interfaces::msg::GameRobotHP msg;

  msg.red_1_robot_hp = all_robot_hp.data.red_1_robot_hp;
  msg.red_2_robot_hp = all_robot_hp.data.red_2_robot_hp;
  msg.red_3_robot_hp = all_robot_hp.data.red_3_robot_hp;
  msg.red_4_robot_hp = all_robot_hp.data.red_4_robot_hp;
  msg.red_7_robot_hp = all_robot_hp.data.red_7_robot_hp;
  msg.red_outpost_hp = all_robot_hp.data.red_outpost_hp;
  msg.red_base_hp = all_robot_hp.data.red_base_hp;

  msg.blue_1_robot_hp = all_robot_hp.data.blue_1_robot_hp;
  msg.blue_2_robot_hp = all_robot_hp.data.blue_2_robot_hp;
  msg.blue_3_robot_hp = all_robot_hp.data.blue_3_robot_hp;
  msg.blue_4_robot_hp = all_robot_hp.data.blue_4_robot_hp;
  msg.blue_7_robot_hp = all_robot_hp.data.blue_7_robot_hp;
  msg.blue_outpost_hp = all_robot_hp.data.blue_outpost_hp;
  msg.blue_base_hp = all_robot_hp.data.blue_base_hp;

  all_robot_hp_pub_->publish(msg);
}

void StandardRobotPpRos2Node::publishGameStatus(ReceiveGameStatusData & game_status)
{
  pb_rm_interfaces::msg::GameStatus msg;
  msg.game_progress = game_status.data.game_progress;
  msg.stage_remain_time = game_status.data.stage_remain_time;
  game_status_pub_->publish(msg);

  if (record_rosbag_ && game_status.data.game_progress != previous_game_progress_) {
    previous_game_progress_ = game_status.data.game_progress;
    RCLCPP_INFO(get_logger(), "Game progress: %d", game_status.data.game_progress);

    std::string service_name;
    switch (game_status.data.game_progress) {
      case pb_rm_interfaces::msg::GameStatus::COUNT_DOWN:
        service_name = "start_recording";
        break;
      case pb_rm_interfaces::msg::GameStatus::GAME_OVER:
        service_name = "stop_recording";
        break;
      default:
        return;
    }

    if (!callTriggerService(service_name)) {
      RCLCPP_ERROR(get_logger(), "Failed to call service: %s", service_name.c_str());
    }
  }
}

void StandardRobotPpRos2Node::publishRobotMotion(ReceiveRobotMotionData & robot_motion)
{
  geometry_msgs::msg::Twist msg;

  msg.linear.x = robot_motion.data.speed_vector.vx;
  msg.linear.y = robot_motion.data.speed_vector.vy;
  msg.angular.z = robot_motion.data.speed_vector.wz;

  robot_motion_pub_->publish(msg);
}

void StandardRobotPpRos2Node::publishGroundRobotPosition(
  ReceiveGroundRobotPosition & ground_robot_position)
{
  pb_rm_interfaces::msg::GroundRobotPosition msg;

  msg.hero_position.x = ground_robot_position.data.hero_x;
  msg.hero_position.y = ground_robot_position.data.hero_y;

  msg.engineer_position.x = ground_robot_position.data.engineer_x;
  msg.engineer_position.y = ground_robot_position.data.engineer_y;

  msg.standard_3_position.x = ground_robot_position.data.standard_3_x;
  msg.standard_3_position.y = ground_robot_position.data.standard_3_y;

  msg.standard_4_position.x = ground_robot_position.data.standard_4_x;
  msg.standard_4_position.y = ground_robot_position.data.standard_4_y;

  ground_robot_position_pub_->publish(msg);
}

void StandardRobotPpRos2Node::publishRfidStatus(ReceiveRfidStatus & rfid_status)
{
  pb_rm_interfaces::msg::RfidStatus msg;

  msg.base_gain_point = rfid_status.data.base_gain_point;
  msg.central_highland_gain_point = rfid_status.data.central_highland_gain_point;
  msg.enemy_central_highland_gain_point = rfid_status.data.enemy_central_highland_gain_point;
  msg.friendly_trapezoidal_highland_gain_point =
    rfid_status.data.friendly_trapezoidal_highland_gain_point;
  msg.enemy_trapezoidal_highland_gain_point =
    rfid_status.data.enemy_trapezoidal_highland_gain_point;
  msg.friendly_fly_ramp_front_gain_point = rfid_status.data.friendly_fly_ramp_front_gain_point;
  msg.friendly_fly_ramp_back_gain_point = rfid_status.data.friendly_fly_ramp_back_gain_point;
  msg.enemy_fly_ramp_front_gain_point = rfid_status.data.enemy_fly_ramp_front_gain_point;
  msg.enemy_fly_ramp_back_gain_point = rfid_status.data.enemy_fly_ramp_back_gain_point;
  msg.friendly_central_highland_lower_gain_point =
    rfid_status.data.friendly_central_highland_lower_gain_point;
  msg.friendly_central_highland_upper_gain_point =
    rfid_status.data.friendly_central_highland_upper_gain_point;
  msg.enemy_central_highland_lower_gain_point =
    rfid_status.data.enemy_central_highland_lower_gain_point;
  msg.enemy_central_highland_upper_gain_point =
    rfid_status.data.enemy_central_highland_upper_gain_point;
  msg.friendly_highway_lower_gain_point = rfid_status.data.friendly_highway_lower_gain_point;
  msg.friendly_highway_upper_gain_point = rfid_status.data.friendly_highway_upper_gain_point;
  msg.enemy_highway_lower_gain_point = rfid_status.data.enemy_highway_lower_gain_point;
  msg.enemy_highway_upper_gain_point = rfid_status.data.enemy_highway_upper_gain_point;
  msg.friendly_fortress_gain_point = rfid_status.data.friendly_fortress_gain_point;
  msg.friendly_outpost_gain_point = rfid_status.data.friendly_outpost_gain_point;
  msg.friendly_supply_zone_non_exchange = rfid_status.data.friendly_supply_zone_non_exchange;
  msg.friendly_supply_zone_exchange = rfid_status.data.friendly_supply_zone_exchange;
  msg.friendly_big_resource_island = rfid_status.data.friendly_big_resource_island;
  msg.enemy_big_resource_island = rfid_status.data.enemy_big_resource_island;
  msg.center_gain_point = rfid_status.data.center_gain_point;

  rfid_status_pub_->publish(msg);
}

void StandardRobotPpRos2Node::publishRobotStatus(ReceiveRobotStatus & robot_status)
{
  pb_rm_interfaces::msg::RobotStatus msg;

  msg.robot_id = robot_status.data.robot_id;
  msg.robot_level = robot_status.data.robot_level;
  msg.current_hp = robot_status.data.current_up;
  msg.maximum_hp = robot_status.data.maximum_hp;
  msg.shooter_barrel_cooling_value = robot_status.data.shooter_barrel_cooling_value;
  msg.shooter_barrel_heat_limit = robot_status.data.shooter_barrel_heat_limit;
  msg.shooter_17mm_1_barrel_heat = robot_status.data.shooter_17mm_1_barrel_heat;
  msg.robot_pos.position.x = robot_status.data.robot_pos_x;
  msg.robot_pos.position.y = robot_status.data.robot_pos_y;
  msg.robot_pos.orientation =
    tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), robot_status.data.robot_pos_angle));
  msg.armor_id = robot_status.data.armor_id;
  msg.hp_deduction_reason = robot_status.data.hp_deduction_reason;
  msg.projectile_allowance_17mm = robot_status.data.projectile_allowance_17mm;
  msg.remaining_gold_coin = robot_status.data.remaining_gold_coin;

  if (last_hp_ - msg.current_hp > 0) {
    msg.is_hp_deduced = true;
  }
  last_hp_ = robot_status.data.current_up;

  robot_status_pub_->publish(msg);

  if (set_detector_color_) {
    uint8_t detect_color;
    if (getDetectColor(robot_status.data.robot_id, detect_color)) {
      if (!initial_set_param_ || detect_color != previous_receive_color_) {
        previous_receive_color_ = detect_color;
        setParam(rclcpp::Parameter("detect_color", detect_color));
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
      }
    }
  }
}

void StandardRobotPpRos2Node::publishJointState(ReceiveJointState & packet)
{
  last_gimbal_pitch_odom_joint_ = packet.data.pitch;
  last_gimbal_yaw_odom_joint_ = packet.data.yaw;
}

void StandardRobotPpRos2Node::publishBuff(ReceiveBuff & buff)
{
  pb_rm_interfaces::msg::Buff msg;
  msg.recovery_buff = buff.data.recovery_buff;
  msg.cooling_buff = buff.data.cooling_buff;
  msg.defence_buff = buff.data.defence_buff;
  msg.vulnerability_buff = buff.data.vulnerability_buff;
  msg.attack_buff = buff.data.attack_buff;
  msg.remaining_energy = buff.data.remaining_energy;
  buff_pub_->publish(msg);
}

/********************************************************/
/* Send data                                            */
/********************************************************/
void StandardRobotPpRos2Node::sendData()
{
  RCLCPP_INFO(get_logger(), "Start sendData!");

  send_robot_cmd_data_.frame_header.sof = SOF_SEND;
  send_robot_cmd_data_.frame_header.id = ID_ROBOT_CMD;
  send_robot_cmd_data_.frame_header.len = sizeof(SendRobotCmdData) - 6;
  send_robot_cmd_data_.data.speed_vector.vx = 0;
  send_robot_cmd_data_.data.speed_vector.vy = 0;
  send_robot_cmd_data_.data.speed_vector.wz = 0;
  // 添加帧头crc8校验
  crc8::append_CRC8_check_sum(
    reinterpret_cast<uint8_t *>(&send_robot_cmd_data_), sizeof(HeaderFrame));

  int retry_count = 0;

  while (rclcpp::ok()) {
    if (!is_usb_ok_) {
      RCLCPP_WARN(get_logger(), "send: usb is not ok! Retry count: %d", retry_count++);
      std::this_thread::sleep_for(std::chrono::milliseconds(USB_NOT_OK_SLEEP_TIME));
      continue;
    }

    try {
      // 整包数据校验
      // 添加数据段crc16校验
      crc16::append_CRC16_check_sum(
        reinterpret_cast<uint8_t *>(&send_robot_cmd_data_), sizeof(SendRobotCmdData));

      // 发送数据
      std::vector<uint8_t> send_data = toVector(send_robot_cmd_data_);
      serial_driver_->port()->send(send_data);
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(get_logger(), "Error sending data: %s", ex.what());
      is_usb_ok_ = false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
}

void StandardRobotPpRos2Node::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  send_robot_cmd_data_.data.speed_vector.vx = msg->linear.x;
  send_robot_cmd_data_.data.speed_vector.vy = msg->linear.y;
  send_robot_cmd_data_.data.speed_vector.wz = msg->angular.z;
}

void StandardRobotPpRos2Node::cmdGimbalJointCallback(
  const sensor_msgs::msg::JointState::SharedPtr msg)
{
  if (msg->name.size() != msg->position.size()) {
    RCLCPP_ERROR(
      get_logger(), "JointState message name and position arrays are of different sizes");
    return;
  }

  for (size_t i = 0; i < msg->name.size(); ++i) {
    if (msg->name[i] == "gimbal_pitch_joint") {
      send_robot_cmd_data_.data.gimbal.pitch = msg->position[i];
    } else if (msg->name[i] == "gimbal_yaw_joint") {
      send_robot_cmd_data_.data.gimbal.yaw = msg->position[i];
    }
  }
}

void StandardRobotPpRos2Node::visionTargetCallback(
  const auto_aim_interfaces::msg::Target::SharedPtr msg)
{
  send_robot_cmd_data_.data.tracking.tracking = msg->tracking;
}

void StandardRobotPpRos2Node::cmdShootCallback(const example_interfaces::msg::UInt8::SharedPtr msg)
{
  send_robot_cmd_data_.data.shoot.fric_on = true;
  send_robot_cmd_data_.data.shoot.fire = msg->data;
}

void StandardRobotPpRos2Node::setParam(const rclcpp::Parameter & param)
{
  if (!initial_set_param_) {
    auto node_graph = this->get_node_graph_interface();
    auto node_names = node_graph->get_node_names();
    std::vector<std::string> possible_detectors = {
      "armor_detector_openvino", "armor_detector_opencv"};

    for (const auto & name : possible_detectors) {
      for (const auto & node_name : node_names) {
        if (node_name.find(name) != std::string::npos) {
          detector_node_name_ = node_name;
          break;
        }
      }
      if (!detector_node_name_.empty()) {
        break;
      }
    }

    if (detector_node_name_.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 1000, "No detector node found!");
      return;
    }

    detector_param_client_ =
      std::make_shared<rclcpp::AsyncParametersClient>(this, detector_node_name_);
    if (!detector_param_client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *this->get_clock(), 1000, "Service not ready, skipping parameter set");
      return;
    }
  }

  if (
    !set_param_future_.valid() ||
    set_param_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
    RCLCPP_INFO(get_logger(), "Setting detect_color to %ld...", param.as_int());
    set_param_future_ = detector_param_client_->set_parameters(
      {param}, [this, param](const ResultFuturePtr & results) {
        for (const auto & result : results.get()) {
          if (!result.successful) {
            RCLCPP_ERROR(get_logger(), "Failed to set parameter: %s", result.reason.c_str());
            return;
          }
        }
        RCLCPP_INFO(get_logger(), "Successfully set detect_color to %ld!", param.as_int());
        initial_set_param_ = true;
      });
  }
}

bool StandardRobotPpRos2Node::getDetectColor(uint8_t robot_id, uint8_t & color)
{
  if (robot_id == 0 || (robot_id > 11 && robot_id < 101)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *this->get_clock(), 1000, "Invalid robot ID: %d. Color not set.", robot_id);
    return false;
  }
  color = (robot_id >= 100) ? 0 : 1;
  return true;
}

bool StandardRobotPpRos2Node::callTriggerService(const std::string & service_name)
{
  auto client = this->create_client<std_srvs::srv::Trigger>(service_name);
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

  auto start_time = std::chrono::steady_clock::now();
  while (!client->wait_for_service(0.1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(
        get_logger(), "Interrupted while waiting for the service: %s", service_name.c_str());
      return false;
    }
    auto elapsed_time = std::chrono::steady_clock::now() - start_time;
    if (elapsed_time > std::chrono::seconds(5)) {
      RCLCPP_ERROR(
        get_logger(), "Service %s not available after 5 seconds, giving up.", service_name.c_str());
      return false;
    }
    RCLCPP_INFO(get_logger(), "Service %s not available, waiting again...", service_name.c_str());
  }

  auto result = client->async_send_request(request);
  if (
    rclcpp::spin_until_future_complete(this->shared_from_this(), result) ==
    rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(
      get_logger(), "Service %s call succeeded: %s", service_name.c_str(),
      result.get()->success ? "true" : "false");
    return result.get()->success;
  }

  RCLCPP_ERROR(get_logger(), "Service %s call failed", service_name.c_str());
  return false;
}

}  // namespace standard_robot_pp_ros2

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(standard_robot_pp_ros2::StandardRobotPpRos2Node)
