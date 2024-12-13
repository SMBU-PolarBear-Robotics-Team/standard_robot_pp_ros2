/**
  ****************************(C) COPYRIGHT 2024 Polarbear*************************
  * @file       StandardRobotPpRos2Node.hpp/cpp
  * @brief      上下位机通信模块
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Jul-24-2024     Penguin         1. done
  @verbatim
  =================================================================================

  =================================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear*************************
  */
#include "standard_robot_pp_ros2.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "crc8_crc16.hpp"
#include "debug_for_pb_rm.hpp"
#include "packet_typedef.hpp"

#define USB_NOT_OK_SLEEP_TIME 1000   // (ms)
#define USB_PROTECT_SLEEP_TIME 1000  // (ms)

namespace standard_robot_pp_ros2
{

StandardRobotPpRos2Node::StandardRobotPpRos2Node(const rclcpp::NodeOptions & options)
: Node("StandardRobotPpRos2Node", options),
  owned_ctx_{new IoContext(2)},
  serial_driver_{new drivers::serial_driver::SerialDriver(*owned_ctx_)}
{
  RCLCPP_INFO(get_logger(), "Start StandardRobotPpRos2Node!");
  debug_for_pb_rm::PrintGreenString("Start StandardRobotPpRos2Node!");

  node_start_time_stamp_ = now();
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
  imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/pb_rm/imu", 10);
  robot_state_info_pub_ =
    this->create_publisher<pb_rm_interfaces::msg::RobotStateInfo>("/pb_rm/robot_info", 10);
  event_data_pub_ =
    this->create_publisher<pb_rm_interfaces::msg::EventData>("/pb_rm/event_data", 10);
  all_robot_hp_pub_ =
    this->create_publisher<pb_rm_interfaces::msg::GameRobotHP>("/pb_rm/all_robot_hp", 10);
  game_progress_pub_ =
    this->create_publisher<pb_rm_interfaces::msg::GameStatus>("/pb_rm/game_progress", 10);
  robot_motion_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/pb_rm/robot_motion", 10);
  ground_robot_position_pub_ = this->create_publisher<pb_rm_interfaces::msg::GroundRobotPosition>(
    "/pb_rm/ground_robot_position", 10);
  rfid_status_pub_ =
    this->create_publisher<pb_rm_interfaces::msg::RfidStatus>("/pb_rm/rfid_status", 10);
  robot_status_pub_ =
    this->create_publisher<pb_rm_interfaces::msg::RobotStatus>("/pb_rm/robot_status", 10);
  joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/pb_rm/joint_state", 10);
  imu_tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
}

void StandardRobotPpRos2Node::createNewDebugPublisher(const std::string & name)
{
  std::string topic_name = "/pb_rm/debug/" + name;
  auto debug_pub = this->create_publisher<std_msgs::msg::Float64>(topic_name, 10);
  debug_pub_map_.insert(std::make_pair(name, debug_pub));
}

void StandardRobotPpRos2Node::createSubscription()
{
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel", 10, std::bind(&StandardRobotPpRos2Node::updateCmdVel, this, std::placeholders::_1));
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
}

/********************************************************/
/* Serial port protect                                  */
/********************************************************/

void StandardRobotPpRos2Node::serialPortProtect()
{
  RCLCPP_INFO(get_logger(), "Start serialPortProtect!");
  debug_for_pb_rm::PrintGreenString("Start serialPortProtect!");

  ///@todo: 1.保持串口连接 2.串口断开重连 3.串口异常处理

  // 初始化串口
  serial_driver_->init_port(device_name_, *device_config_);
  // 尝试打开串口
  try {
    if (!serial_driver_->port()->is_open()) {
      serial_driver_->port()->open();
      debug_for_pb_rm::PrintGreenString("Serial port opened!");
      usb_is_ok_ = true;
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Open serial port failed : %s", ex.what());
    usb_is_ok_ = false;
  }

  usb_is_ok_ = true;
  std::this_thread::sleep_for(std::chrono::milliseconds(USB_PROTECT_SLEEP_TIME));

  while (rclcpp::ok()) {
    if (!usb_is_ok_) {
      try {
        if (serial_driver_->port()->is_open()) {
          serial_driver_->port()->close();
        }

        serial_driver_->port()->open();

        if (serial_driver_->port()->is_open()) {
          std::cout << "\033[32m Serial port opened! \033[0m" << std::endl;
          usb_is_ok_ = true;
        }
      } catch (const std::exception & ex) {
        usb_is_ok_ = false;
        RCLCPP_ERROR(get_logger(), "Open serial port failed : %s", ex.what());
      }
    };

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
  debug_for_pb_rm::PrintGreenString("Start receiveData!");

  std::vector<uint8_t> sof(1);
  std::vector<uint8_t> receive_data;

  int sof_count = 0;

  while (rclcpp::ok()) {
    if (!usb_is_ok_) {
      std::cout << "reveive: usb is not ok!" << std::endl;
      // thread sleep
      std::this_thread::sleep_for(std::chrono::milliseconds(USB_NOT_OK_SLEEP_TIME));
      continue;
    }

    try {
      serial_driver_->port()->receive(sof);

      if (sof[0] != SOF_RECEIVE) {
        sof_count++;
        std::cout << "Find sof, cnt=" << sof_count << std::endl;
        continue;
      }
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
        case ID_ROBOT_INFO: {
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
        case ID_RFID_STASTUS: {
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
        default: {
          RCLCPP_WARN(get_logger(), "Invalid id: %d", header_frame.id);
        } break;
      }
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(get_logger(), "Error receiving data: %s", ex.what());
      usb_is_ok_ = false;
    }
  }
}

void StandardRobotPpRos2Node::publishDebugData(ReceiveDebugData & received_debug_data)
{
  static rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr debug_pub;
  for (int i = 0; i < DEBUG_PACKAGE_NUM; i++) {
    // Create a vector to hold the non-zero data
    std::vector<uint8_t> non_zero_data;
    for (size_t j = 0; j < DEBUG_PACKAGE_NAME_LEN; j++) {
      if (received_debug_data.packages[i].name[j] != 0) {
        non_zero_data.push_back(received_debug_data.packages[i].name[j]);
      } else {
        break;
      }
    }
    // Convert the non-zero data to a string
    std::string name(non_zero_data.begin(), non_zero_data.end());

    if (name.empty()) {
      continue;
    }

    if (debug_pub_map_.find(name) == debug_pub_map_.end()) {  // The key is not in the map
      createNewDebugPublisher(name);
    }
    debug_pub = debug_pub_map_.at(name);

    std_msgs::msg::Float64 msg;
    msg.data = received_debug_data.packages[i].data;
    debug_pub->publish(msg);
  }
}

void StandardRobotPpRos2Node::publishImuData(ReceiveImuData & imu_data)
{
  sensor_msgs::msg::Imu msg;
  // Convert Euler angles to quaternion
  tf2::Quaternion q;
  q.setRPY(imu_data.data.roll, imu_data.data.pitch, imu_data.data.yaw);
  // Set the header
  msg.header.stamp.sec = imu_data.time_stamp / 1000;
  msg.header.stamp.nanosec = (imu_data.time_stamp % 1000) * 1e6;
  msg.header.frame_id = "odom";
  // Set the orientation
  msg.orientation.x = q.x();
  msg.orientation.y = q.y();
  msg.orientation.z = q.z();
  msg.orientation.w = q.w();
  // Set the angular velocity
  msg.angular_velocity.x = imu_data.data.roll_vel;
  msg.angular_velocity.y = imu_data.data.pitch_vel;
  msg.angular_velocity.z = imu_data.data.yaw_vel;
  // Set the linear acceleration
  // msg.linear_acceleration.x = imu_data.data.x_accel;
  // msg.linear_acceleration.y = imu_data.data.y_accel;
  // msg.linear_acceleration.z = imu_data.data.z_accel;
  // Publish the message
  imu_pub_->publish(msg);

  // Publish the transform to visualize the IMU in Foxglove Studio
  geometry_msgs::msg::TransformStamped t;
  msg.header.stamp.sec = imu_data.time_stamp / 1000;
  msg.header.stamp.nanosec = (imu_data.time_stamp % 1000) * 1e6;
  t.header.frame_id = "odom";
  t.child_frame_id = "imu";
  t.transform.rotation = tf2::toMsg(q);
  imu_tf_broadcaster_->sendTransform(t);
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

  msg.supply_station_front = event_data.supply_station_front;
  msg.supply_station_internal = event_data.supply_station_internal;
  msg.supply_zone = event_data.supply_zone;
  msg.center_gain_zone = event_data.center_gain_zone;

  msg.small_energy = event_data.small_energy;
  msg.big_energy = event_data.big_energy;

  msg.circular_highland = event_data.circular_highland;
  msg.trapezoidal_highland_3 = event_data.trapezoidal_highland_3;
  msg.trapezoidal_highland_4 = event_data.trapezoidal_highland_4;

  msg.base_virtual_shield_remaining = event_data.base_virtual_shield_remaining;

  event_data_pub_->publish(msg);
}

void StandardRobotPpRos2Node::publishAllRobotHp(ReceiveAllRobotHpData & all_robot_hp)
{
  pb_rm_interfaces::msg::GameRobotHP msg;

  msg.red_1_robot_hp = all_robot_hp.data.red_1_robot_hp;
  msg.red_2_robot_hp = all_robot_hp.data.red_2_robot_hp;
  msg.red_3_robot_hp = all_robot_hp.data.red_3_robot_hp;
  msg.red_4_robot_hp = all_robot_hp.data.red_4_robot_hp;
  msg.red_5_robot_hp = all_robot_hp.data.red_5_robot_hp;
  msg.red_7_robot_hp = all_robot_hp.data.red_7_robot_hp;
  msg.red_outpost_hp = all_robot_hp.data.red_outpost_hp;
  msg.red_base_hp = all_robot_hp.data.red_base_hp;

  msg.blue_1_robot_hp = all_robot_hp.data.blue_1_robot_hp;
  msg.blue_2_robot_hp = all_robot_hp.data.blue_2_robot_hp;
  msg.blue_3_robot_hp = all_robot_hp.data.blue_3_robot_hp;
  msg.blue_4_robot_hp = all_robot_hp.data.blue_4_robot_hp;
  msg.blue_5_robot_hp = all_robot_hp.data.blue_5_robot_hp;
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

  game_progress_pub_->publish(msg);
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

  msg.hero_x = ground_robot_position.hero_x;
  msg.hero_y = ground_robot_position.hero_y;

  msg.engineer_x = ground_robot_position.engineer_x;
  msg.engineer_y = ground_robot_position.engineer_y;

  msg.standard_3_x = ground_robot_position.standard_3_x;
  msg.standard_3_y = ground_robot_position.standard_3_y;

  msg.standard_4_x = ground_robot_position.standard_4_x;
  msg.standard_4_y = ground_robot_position.standard_4_y;

  msg.standard_5_x = ground_robot_position.standard_5_x;
  msg.standard_5_y = ground_robot_position.standard_5_y;

  ground_robot_position_pub_->publish(msg);
}

void StandardRobotPpRos2Node::publishRfidStatus(ReceiveRfidStatus & rfid_status)
{
  pb_rm_interfaces::msg::RfidStatus msg;

  msg.base_gain_point = rfid_status.base_gain_point;
  msg.circular_highland_gain_point = rfid_status.circular_highland_gain_point;
  msg.enemy_circular_highland_gain_point = rfid_status.enemy_circular_highland_gain_point;
  msg.friendly_r3_b3_gain_point = rfid_status.friendly_r3_b3_gain_point;
  msg.enemy_r3_b3_gain_point = rfid_status.enemy_r3_b3_gain_point;
  msg.friendly_r4_b4_gain_point = rfid_status.friendly_r4_b4_gain_point;
  msg.enemy_r4_b4_gain_point = rfid_status.enemy_r4_b4_gain_point;
  msg.energy_mechanism_gain_point = rfid_status.energy_mechanism_gain_point;
  msg.friendly_fly_ramp_front_gain_point = rfid_status.friendly_fly_ramp_front_gain_point;
  msg.friendly_fly_ramp_back_gain_point = rfid_status.friendly_fly_ramp_back_gain_point;
  msg.enemy_fly_ramp_front_gain_point = rfid_status.enemy_fly_ramp_front_gain_point;
  msg.enemy_fly_ramp_back_gain_point = rfid_status.enemy_fly_ramp_back_gain_point;
  msg.friendly_outpost_gain_point = rfid_status.friendly_outpost_gain_point;
  msg.friendly_healing_point = rfid_status.friendly_healing_point;
  msg.friendly_sentry_patrol_area = rfid_status.friendly_sentry_patrol_area;
  msg.enemy_sentry_patrol_area = rfid_status.enemy_sentry_patrol_area;
  msg.friendly_big_resource_island = rfid_status.friendly_big_resource_island;
  msg.enemy_big_resource_island = rfid_status.enemy_big_resource_island;
  msg.friendly_exchange_area = rfid_status.friendly_exchange_area;
  msg.center_gain_point = rfid_status.center_gain_point;

  rfid_status_pub_->publish(msg);
}

void StandardRobotPpRos2Node::publishRobotStatus(ReceiveRobotStatus & robot_status)
{
  pb_rm_interfaces::msg::RobotStatus msg;

  msg.robot_id = robot_status.robot_id;
  msg.robot_level = robot_status.robot_level;
  msg.current_hp = robot_status.current_up;
  msg.maximum_hp = robot_status.maximum_hp;
  msg.shooter_barrel_cooling_value = robot_status.shooter_barrel_cooling_value;
  msg.shooter_barrel_heat_limit = robot_status.shooter_barrel_heat_limit;
  msg.shooter_17mm_1_barrel_heat = robot_status.shooter_17mm_1_barrel_heat;
  msg.robot_pos_x = robot_status.robot_pos_x;
  msg.robot_pos_y = robot_status.robot_pos_y;
  msg.robot_pos_angle = robot_status.robot_pos_angle;
  msg.armor_id = robot_status.armor_id;
  msg.hp_deduction_reason = robot_status.hp_deduction_reason;
  msg.projectile_allowance_17mm_1 = robot_status.projectile_allowance_17mm_1;
  msg.remaining_gold_coin = robot_status.remaining_gold_coin;

  robot_status_pub_->publish(msg);
}

void StandardRobotPpRos2Node::publishJointState(ReceiveJointState & joint_state)
{
  sensor_msgs::msg::JointState msg;

  msg.position.resize(2);
  msg.name.resize(2);
  msg.header.stamp = now();

  msg.name[0] = "gimbal_pitch_joint";
  msg.position[0] = joint_state.data.pitch;

  msg.name[1] = "gimbal_yaw_joint";
  msg.position[1] = joint_state.data.yaw;

  joint_state_pub_->publish(msg);
}

/********************************************************/
/* Send data                                            */
/********************************************************/
void StandardRobotPpRos2Node::sendData()
{
  RCLCPP_INFO(get_logger(), "Start sendData!");
  debug_for_pb_rm::PrintGreenString("Start sendData!");

  send_robot_cmd_data_.frame_header.sof = SOF_SEND;
  send_robot_cmd_data_.frame_header.id = ID_ROBOT_CMD;
  send_robot_cmd_data_.frame_header.len = sizeof(SendRobotCmdData) - 6;
  // 添加帧头crc8校验
  crc8::append_CRC8_check_sum(
    reinterpret_cast<uint8_t *>(&send_robot_cmd_data_), sizeof(HeaderFrame));

  while (rclcpp::ok()) {
    if (!usb_is_ok_) {
      std::cout << "send: usb is not ok!" << std::endl;
      // thread sleep
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
      usb_is_ok_ = false;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
}

void StandardRobotPpRos2Node::updateCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  send_robot_cmd_data_.data.speed_vector.vx = msg->linear.x;
  send_robot_cmd_data_.data.speed_vector.vy = msg->linear.y;
  send_robot_cmd_data_.data.speed_vector.wz = msg->angular.z;
}

}  // namespace standard_robot_pp_ros2

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(standard_robot_pp_ros2::StandardRobotPpRos2Node)