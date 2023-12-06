// @file integration_test.cpp
// @brief Dummy file for enabling level 2 tests of this package
// @Modified by Vedant Ranade
// @note This file is derived from
// https://github.com/TommyChangUMD/minimal-integration-test/blob/main/test/basic_test.cpp
// @copyright Copyright (c) 2023

#include <gtest/gtest.h>
#include <stdlib.h>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  std::cout << "DONE SHUTTING DOWN ROS" << std::endl;
  return result;
}