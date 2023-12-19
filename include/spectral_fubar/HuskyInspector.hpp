#ifndef HUSKY_INSPECTOR_HPP
#define HUSKY_INSPECTOR_HPP
#include <geometry_msgs/msg/twist.hpp>
#include <mutex>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
namespace husky {
class Inspector : public rclcpp::Node {
 public:
  Inspector();

 private:
  void cmdVelPublisher();
  geometry_msgs::msg::Twist cmdVel_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr commandVelPublisher_;
  rclcpp::TimerBase::SharedPtr cmdVelTimer_;
};
};  // namespace husky
#endif