#ifndef STANDARD_ROBOT_PP_ROS2__DEBUG_FOR_pb_rm_HPP_
#define STANDARD_ROBOT_PP_ROS2__DEBUG_FOR_pb_rm_HPP_

#include <iostream>
#include <string>
#include <vector>

namespace debug_for_pb_rm
{
inline void OutputByByte(std::string str, std::vector<uint8_t> data)
{
  std::cout << str;
  for (size_t i = 0; i < data.size(); i++) {
    std::cout << std::hex << (int)data[i] << " ";
  }
  std::cout << std::endl;
}
inline void OutputByByte(std::string str, uint8_t * data, size_t size)
{
  std::cout << str;
  for (size_t i = 0; i < size; i++) {
    std::cout << std::hex << (int)data[i] << " ";
  }
  std::cout << std::endl;
}

inline void PrintGreenString(std::string str)
{
  std::cout << "\033[32m" << str << "\033[0m" << std::endl;
}
inline void PrintRedString(std::string str)
{
  std::cout << "\033[31m" << str << "\033[0m" << std::endl;
}
inline void PrintYellowString(std::string str)
{
  std::cout << "\033[33m" << str << "\033[0m" << std::endl;
}

}  // namespace debug_for_pb_rm
#endif  // STANDARD_ROBOT_PP_ROS2__DEBUG_FOR_pb_rm_HPP_