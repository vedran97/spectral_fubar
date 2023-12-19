#ifndef HUSKY_INSPECTOR_HPP
#define HUSKY_INSPECTOR_HPP
#include <gazebo_msgs/msg/model_states.hpp>
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
  geometry_msgs::msg::Pose robotPose_;
  geometry_msgs::msg::Twist cmdVel_;
  rclcpp::TimerBase::SharedPtr cmdVelTimer_;
  void cmdVelPublisher();

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr commandVelPublisher_;

  rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr subscription_;
  void modelStatesCallback(const gazebo_msgs::msg::ModelStates::SharedPtr msg);

  void quaternionToRPY(const geometry_msgs::msg::Quaternion& quaternion,
                       double& roll, double& pitch, double& yaw);
  // @brief: Publishes on cmd_vel topic to move the robot forward in x direction
  inline void forward();
  // @brief: Publishes on cmd_vel topic to turn the robot in z direction
  inline void turn(int right);
  // @brief: Publishes on cmd_vel topic to stop the robot
  inline void stop();
};
};  // namespace husky
#endif