#ifndef HUSKY_INSPECTOR_HPP
#define HUSKY_INSPECTOR_HPP
#include <geometry_msgs/msg/twist.hpp>
#include <mutex>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <gazebo_msgs/msg/model_states.hpp>
namespace husky {
class Inspector : public rclcpp::Node {
 public:
  Inspector();

 private:
  void cmdVelPublisher();
  geometry_msgs::msg::Twist cmdVel_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr commandVelPublisher_;
  rclcpp::TimerBase::SharedPtr cmdVelTimer_;
   rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr subscription_;
   void modelStatesCallback(const gazebo_msgs::msg::ModelStates::SharedPtr msg);
   geometry_msgs::msg::Pose robotPose_;
    void quaternionToRPY(const geometry_msgs::msg::Quaternion& quaternion, double& roll, double& pitch, double& yaw);
};
};  // namespace husky
#endif