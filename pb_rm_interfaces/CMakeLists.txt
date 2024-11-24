cmake_minimum_required(VERSION 3.5)

project(pb_rm_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/referee/EventData.msg"
  "msg/referee/GameRobotHP.msg"
  "msg/referee/GameStatus.msg"
  "msg/referee/GroundRobotPosition.msg"
  "msg/referee/RfidStatus.msg"
  "msg/referee/RobotStatus.msg"

  "msg/GimbalCmd.msg"
  "msg/ShootCmd.msg"

  DEPENDENCIES
    geometry_msgs
)


ament_package()
