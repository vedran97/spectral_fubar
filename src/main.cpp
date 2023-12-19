#include "HuskyInspector.hpp"
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<husky::Inspector>());
  rclcpp::shutdown();
  return 0;
}