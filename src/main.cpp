/**
 * @file main.cpp
 * @author Vedant Ranade
 * @brief Executable entry point for HuskyInspector
 * @version 0.1
 * @date 2023-12-20
 *
 * @copyright Copyright (c) 2023
 *
 */
#include "HuskyInspector.hpp"
//@brief main entry point for the husky inspector
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<husky::Inspector>());
  rclcpp::shutdown();
  return 0;
}
