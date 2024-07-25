/**
  ****************************(C) COPYRIGHT 2024 Polarbear*************************
  * @file       ROS2_StandardRobotpp.hpp/cpp
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

#include <tf2/LinearMath/Quaternion.h>

#include <iostream>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>
#include <serial_driver/serial_driver.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// C++ system
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "CRC8_CRC16.hpp"
#include "ROS2_StandardRobotpp.hpp"
#include "debug_for_srpp.hpp"
#include "packet_typedef.hpp"

#define USB_NOT_OK_SLEEP_TIME 1000   // (ms)
#define USB_PROTECT_SLEEP_TIME 1000  // (ms)

namespace ros2_standard_robot_pp
{

ROS2_StandardRobotpp::ROS2_StandardRobotpp(const rclcpp::NodeOptions & options)
: Node("ros2_standard_robot_pp", options),
  owned_ctx_{new IoContext(2)},
  serial_driver_{new drivers::serial_driver::SerialDriver(*owned_ctx_)}
{
    RCLCPP_INFO(get_logger(), "Start ROS2_StandardRobotpp!");
    debug_for_srpp::PrintGreenString("Start ROS2_StandardRobotpp!");

    node_start_time_stamp = now();
    getParams();
    createPublisher();
    createSubscription();

    // create robot models map
    robot_models_.chassis = {
        {0, "无底盘"}, {1, "麦轮底盘"}, {2, "全向轮底盘"}, {3, "舵轮底盘"}, {4, "平衡底盘"}};
    robot_models_.gimbal = {{0, "无云台"}, {1, "yaw_pitch直连云台"}};
    robot_models_.shoot = {{0, "无发射机构"}, {1, "摩擦轮+拨弹盘"}, {2, "气动+拨弹盘"}};
    robot_models_.arm = {{0, "无机械臂"}, {1, "mini机械臂"}};
    robot_models_.custom_controller = {{0, "无自定义控制器"}, {1, "mini自定义控制器"}};

    // 启动线程
    serial_port_protect_thread_ = std::thread(&ROS2_StandardRobotpp::serialPortProtect, this);
    receive_thread_ = std::thread(&ROS2_StandardRobotpp::receiveData, this);
    send_thread_ = std::thread(&ROS2_StandardRobotpp::sendData, this);
}

ROS2_StandardRobotpp::~ROS2_StandardRobotpp()
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

void ROS2_StandardRobotpp::createPublisher()
{
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/srpp/imu", 10);
    robot_state_info_pub_ =
        this->create_publisher<srpp_interfaces::msg::RobotStateInfo>("/srpp/robot_state_info", 10);

    imu_tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
}

void ROS2_StandardRobotpp::createNewDebugPublisher(const std::string & name)
{
    std::string topic_name = "/srpp/debug/" + name;
    auto debug_pub = this->create_publisher<std_msgs::msg::Float64>(topic_name, 10);
    debug_pub_map_.insert(std::make_pair(name, debug_pub));
}

void ROS2_StandardRobotpp::createSubscription()
{
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10,
        std::bind(&ROS2_StandardRobotpp::updateCmdVel, this, std::placeholders::_1));
}

void ROS2_StandardRobotpp::getParams()
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

void ROS2_StandardRobotpp::serialPortProtect()
{
    RCLCPP_INFO(get_logger(), "Start serialPortProtect!");
    debug_for_srpp::PrintGreenString("Start serialPortProtect!");

    ///@todo: 1.保持串口连接 2.串口断开重连 3.串口异常处理

    // 初始化串口
    serial_driver_->init_port(device_name_, *device_config_);

    //尝试打开串口
    try {
        if (!serial_driver_->port()->is_open()) {
            serial_driver_->port()->open();
            debug_for_srpp::PrintGreenString("Serial port opened!");
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

void ROS2_StandardRobotpp::receiveData()
{
    RCLCPP_INFO(get_logger(), "Start receiveData!");
    debug_for_srpp::PrintGreenString("Start receiveData!");

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

            //### sof[0] == SOF_RECEIVE 后读取剩余 header_frame内容
            std::vector<uint8_t> header_frame_buf(3);  // sof在读取完数据后添加

            serial_driver_->port()->receive(header_frame_buf);  // 读取除sof外剩下的数据
            header_frame_buf.insert(header_frame_buf.begin(), sof[0]);  // 添加sof
            HeaderFrame header_frame = fromVector<HeaderFrame>(header_frame_buf);

            // HeaderFrame CRC8 check
            bool crc8_ok = crc8::verify_CRC8_check_sum(
                reinterpret_cast<uint8_t *>(&header_frame), sizeof(header_frame));
            if (!crc8_ok) {
                RCLCPP_ERROR(get_logger(), "Header frame CRC8 error!");
                continue;
            }

            //### crc8_ok 校验正确后读取数据段
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
                data_buf.insert(
                    data_buf.begin() + received_len_sum, remain_buf.begin(), remain_buf.end());
                received_len_sum += received_len;
                remain_len -= received_len;
            }

            // 数据段读取完成后添加header_frame_buf到data_buf，得到完整数据包
            data_buf.insert(data_buf.begin(), header_frame_buf.begin(), header_frame_buf.end());

            // 根据header_frame.id解析数据
            switch (header_frame.id) {
                case ID_DEBUG: {
                    ReceiveDebugData debug_data = fromVector<ReceiveDebugData>(data_buf);
                    // 整包数据校验
                    bool crc16_ok = crc16::verify_CRC16_check_sum(
                        reinterpret_cast<uint8_t *>(&debug_data), sizeof(ReceiveDebugData));
                    if (crc16_ok) {
                        publishDebugData(debug_data);
                    } else {
                        RCLCPP_ERROR(get_logger(), "Debug data crc16 error!");
                    }
                } break;
                case ID_IMU: {
                    ReceiveImuData imu_data = fromVector<ReceiveImuData>(data_buf);

                    // 整包数据校验
                    bool crc16_ok = crc16::verify_CRC16_check_sum(
                        reinterpret_cast<uint8_t *>(&imu_data), sizeof(ReceiveImuData));
                    if (crc16_ok) {
                        publishImuData(imu_data);
                    } else {
                        RCLCPP_ERROR(get_logger(), "Imu data crc16 error!");
                    }
                } break;
                case ID_ROBOT_INFO: {
                    ReceiveRobotInfoData robot_info_data =
                        fromVector<ReceiveRobotInfoData>(data_buf);

                    // 整包数据校验
                    bool crc16_ok = crc16::verify_CRC16_check_sum(
                        reinterpret_cast<uint8_t *>(&robot_info_data),
                        sizeof(ReceiveRobotInfoData));
                    if (crc16_ok) {
                        publishRobotStateInfo(robot_info_data);
                    } else {
                        RCLCPP_ERROR(get_logger(), "Robot info data crc16 error!");
                    }
                } break;
                case ID_PID_DEBUG: {
                    RCLCPP_WARN(get_logger(), "Not implemented yet!");
                } break;
                case ID_ALL_ROBOT_HP: {
                    ReceiveAllRobotHpData all_robot_hp_data =
                        fromVector<ReceiveAllRobotHpData>(data_buf);

                    // 整包数据校验
                    bool crc16_ok = crc16::verify_CRC16_check_sum(
                        reinterpret_cast<uint8_t *>(&all_robot_hp_data),
                        sizeof(ReceiveAllRobotHpData));
                    if (crc16_ok) {
                        ;
                    } else {
                        RCLCPP_ERROR(get_logger(), "All robot hp data crc16 error!");
                    }
                } break;
                case ID_GAME_STATUS: {
                    ReceiveGameStatusData game_status_data =
                        fromVector<ReceiveGameStatusData>(data_buf);

                    // 整包数据校验
                    bool crc16_ok = crc16::verify_CRC16_check_sum(
                        reinterpret_cast<uint8_t *>(&game_status_data),
                        sizeof(ReceiveGameStatusData));
                    if (crc16_ok) {
                        ;
                    } else {
                        RCLCPP_ERROR(get_logger(), "All robot hp data crc16 error!");
                    }
                } break;
                case ID_ROBOT_MOTION: {
                    ReceiveRobotMotionData robot_motion_data =
                        fromVector<ReceiveRobotMotionData>(data_buf);

                    // 整包数据校验
                    bool crc16_ok = crc16::verify_CRC16_check_sum(
                        reinterpret_cast<uint8_t *>(&robot_motion_data),
                        sizeof(ReceiveRobotMotionData));
                    if (crc16_ok) {
                        ;
                    } else {
                        RCLCPP_ERROR(get_logger(), "Robot motion data crc16 error!");
                    }
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

void ROS2_StandardRobotpp::publishDebugData(ReceiveDebugData & received_debug_data)
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

        std_msgs::msg::Float64 debug_data;
        debug_data.data = received_debug_data.packages[i].data;
        debug_pub->publish(debug_data);
    }
}

void ROS2_StandardRobotpp::publishImuData(ReceiveImuData & imu_data)
{
    auto imu_msg = sensor_msgs::msg::Imu();
    // Convert Euler angles to quaternion
    tf2::Quaternion q;
    q.setRPY(imu_data.data.roll, imu_data.data.pitch, imu_data.data.yaw);
    // Set the header
    imu_msg.header.stamp.sec = imu_data.time_stamp / 1000;
    imu_msg.header.stamp.nanosec = (imu_data.time_stamp % 1000) * 1e6;
    imu_msg.header.frame_id = "odom";
    // Set the orientation
    imu_msg.orientation.x = q.x();
    imu_msg.orientation.y = q.y();
    imu_msg.orientation.z = q.z();
    imu_msg.orientation.w = q.w();
    // Set the angular velocity
    imu_msg.angular_velocity.x = imu_data.data.roll_vel;
    imu_msg.angular_velocity.y = imu_data.data.pitch_vel;
    imu_msg.angular_velocity.z = imu_data.data.yaw_vel;
    // Set the linear acceleration
    // imu_msg.linear_acceleration.x = imu_data.data.x_accel;
    // imu_msg.linear_acceleration.y = imu_data.data.y_accel;
    // imu_msg.linear_acceleration.z = imu_data.data.z_accel;
    // Publish the message
    imu_pub_->publish(imu_msg);

    // Publish the transform to visualize the IMU in Foxglove Studio
    geometry_msgs::msg::TransformStamped t;
    imu_msg.header.stamp.sec = imu_data.time_stamp / 1000;
    imu_msg.header.stamp.nanosec = (imu_data.time_stamp % 1000) * 1e6;
    t.header.frame_id = "odom";
    t.child_frame_id = "imu";
    t.transform.rotation = tf2::toMsg(q);
    imu_tf_broadcaster_->sendTransform(t);
}

void ROS2_StandardRobotpp::publishRobotStateInfo(ReceiveRobotInfoData & robot_info)
{
    auto robot_state_info_msg = srpp_interfaces::msg::RobotStateInfo();
    robot_state_info_msg.header.stamp.sec = robot_info.time_stamp / 1000;
    robot_state_info_msg.header.stamp.nanosec = (robot_info.time_stamp % 1000) * 1e6;
    robot_state_info_msg.header.frame_id = "odom";

    robot_state_info_msg.models.chassis = robot_models_.chassis.at(robot_info.data.type.chassis);
    robot_state_info_msg.models.gimbal = robot_models_.gimbal.at(robot_info.data.type.gimbal);
    robot_state_info_msg.models.shoot = robot_models_.shoot.at(robot_info.data.type.shoot);
    robot_state_info_msg.models.arm = robot_models_.arm.at(robot_info.data.type.arm);
    robot_state_info_msg.models.custom_controller =
        robot_models_.custom_controller.at(robot_info.data.type.custom_controller);

    robot_state_info_msg.referee.type = "步兵";
    robot_state_info_msg.referee.color = "red";
    robot_state_info_msg.referee.attacked = robot_info.data.referee.attacked;
    robot_state_info_msg.referee.hp = robot_info.data.referee.hp;
    robot_state_info_msg.referee.heat = robot_info.data.referee.heat;

    robot_state_info_pub_->publish(robot_state_info_msg);
}

/********************************************************/
/* Send data                                            */
/********************************************************/

void ROS2_StandardRobotpp::sendData()
{
    RCLCPP_INFO(get_logger(), "Start sendData!");
    debug_for_srpp::PrintGreenString("Start sendData!");

    send_robot_cmd_data_.frame_header.sof = SOF_SEND;
    send_robot_cmd_data_.frame_header.id = ID_ROBOT_CMD;
    send_robot_cmd_data_.frame_header.len = sizeof(SendRobotCmdData) - 6;
    crc8::append_CRC8_check_sum(  //添加帧头crc8校验
        reinterpret_cast<uint8_t *>(&send_robot_cmd_data_), sizeof(HeaderFrame));

    while (rclcpp::ok()) {
        if (!usb_is_ok_) {
            std::cout << "send: usb is not ok!" << std::endl;
            // thread sleep
            std::this_thread::sleep_for(std::chrono::milliseconds(USB_NOT_OK_SLEEP_TIME));
            continue;
        }

        try {
            // use for test
            // rclcpp::Duration run_time = now() - node_start_time_stamp;
            // std::cout << "time_stamp_ms.seconds: " << run_time.seconds() << std::endl;
            // std::cout << "time_stamp_ms.nanoseconds: " << run_time.nanoseconds() << std::endl;
            // double sin_value = std::sin(run_time.seconds());  // 计算sin值
            // std::cout << "sin_value: " << sin_value << std::endl;
            // send_robot_cmd_data_.data.speed_vector.vx = sin_value - 1;
            // send_robot_cmd_data_.data.speed_vector.vy = sin_value;
            // send_robot_cmd_data_.data.speed_vector.wz = sin_value + 1;
            // send_robot_cmd_data_.data.chassis.yaw = sin_value * 2 + 2;
            // send_robot_cmd_data_.data.chassis.pitch = sin_value * 2 + 3;
            // send_robot_cmd_data_.data.chassis.roll = sin_value * 2 + 4;
            // send_robot_cmd_data_.data.chassis.leg_lenth = sin_value * 2 + 5;
            // send_robot_cmd_data_.data.gimbal.yaw = sin_value * 3 + 6;
            // send_robot_cmd_data_.data.gimbal.pitch = sin_value * 3 + 7;

            // 整包数据校验
            crc16::append_CRC16_check_sum(  //添加数据段crc16校验
                reinterpret_cast<uint8_t *>(&send_robot_cmd_data_), sizeof(SendRobotCmdData));

            // 发送数据
            std::vector<uint8_t> send_data = toVector(send_robot_cmd_data_);
            serial_driver_->port()->send(send_data);

        } catch (const std::exception & ex) {
            RCLCPP_ERROR(get_logger(), "Error sending data: %s", ex.what());
            usb_is_ok_ = false;
        }

        // thread sleep
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

void ROS2_StandardRobotpp::updateCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    // 更新发送数据
    send_robot_cmd_data_.data.speed_vector.vx = msg->linear.x;
    send_robot_cmd_data_.data.speed_vector.vy = msg->linear.y;
    send_robot_cmd_data_.data.speed_vector.wz = msg->angular.z;
}

}  // namespace ros2_standard_robot_pp

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(ros2_standard_robot_pp::ROS2_StandardRobotpp)