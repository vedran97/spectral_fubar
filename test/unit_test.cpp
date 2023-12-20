#include <gtest/gtest.h>
#include <rclcpp/utilities.hpp>
#include "HuskyInspector.hpp"



TEST(test_1, is_right_check) {
  auto inspector = std::make_shared<husky::Inspector>();
  EXPECT_EQ(false, inspector->isRight());
 }
 TEST(test_2, isObjectDetected) {
  auto inspector = std::make_shared<husky::Inspector>();
  EXPECT_EQ(false, inspector->isObjectDetected());
 }

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  auto ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}