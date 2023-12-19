#include "HuskyInspector.hpp"

#include <tf2/LinearMath/Quaternion.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
namespace husky {
Inspector::Inspector() : Node("inspector") {
  cmdVel_ = geometry_msgs::msg::Twist();
  commandVelPublisher_ =
      this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  // timer callback function is called every 100ms
  cmdVelTimer_ =
      this->create_wall_timer(std::chrono::milliseconds(100),
                              std::bind(&Inspector::cmdVelPublisher, this));
  subscription_ = create_subscription<gazebo_msgs::msg::ModelStates>(
      "/gazebo/model_states", 10,
      std::bind(&Inspector::modelStatesCallback, this, std::placeholders::_1));
    // subscribe to depth image topic
    depthImgSubscriber_ = this->create_subscription<image>(
        "/front_realsense_depth/depth/image_raw", 10, [this](const image& msg) {
          auto recvimg=msg;
          const auto data = reinterpret_cast<float*>(recvimg.data.data());
        const auto& maxHeight = recvimg.height;
        const auto& maxWidth = recvimg.width;
          const auto idx = (maxHeight/2) * maxWidth + maxWidth/2;
           const auto& pixel = data[idx];
            RCLCPP_INFO(this->get_logger(),"Depth %f",pixel);
        });
}
void Inspector::cmdVelPublisher() {
  this->commandVelPublisher_->publish(cmdVel_);
}
void Inspector::modelStatesCallback(
    const gazebo_msgs::msg::ModelStates::SharedPtr msg) {
  for (size_t i = 0; i < msg->name.size(); ++i) {
    if (msg->name[i] == "husky") {
      robotPose_ = msg->pose[i];
      RCLCPP_DEBUG(get_logger(), "Husky Pose: x=%f, y=%f, z=%f",
                   robotPose_.position.x, robotPose_.position.y,
                   robotPose_.position.z);
      break;  // Exit loop once "husky" is found
    }
  }
}
void Inspector::quaternionToRPY(
    const geometry_msgs::msg::Quaternion& quaternion, double& roll,
    double& pitch, double& yaw) {
  tf2::Quaternion tf_quaternion(quaternion.x, quaternion.y, quaternion.z,
                                quaternion.w);
  tf2::Matrix3x3(tf_quaternion).getRPY(roll, pitch, yaw);
}

// @brief: Publishes on cmd_vel topic to move the robot forward in x direction
inline void Inspector::forward() {
  RCLCPP_INFO_STREAM(this->get_logger(), "Moving forward");

  this->cmdVel_ = geometry_msgs::msg::Twist();
  this->cmdVel_.linear.x = 0.5;
}
// @brief: Publishes on cmd_vel topic to turn the robot in z direction
inline void Inspector::turn(bool right) {
  RCLCPP_INFO_STREAM(this->get_logger(), "Turning");
  this->cmdVel_ = geometry_msgs::msg::Twist();
  cmdVel_.angular.z = 0.2* (right?-1.0:0.0);
  commandVelPublisher_->publish(cmdVel_);
}
// @brief: Publishes on cmd_vel topic to stop the robot
inline void Inspector::stop() {
  RCLCPP_INFO_STREAM(this->get_logger(), "Stopping");
  this->cmdVel_ = geometry_msgs::msg::Twist();
}

}  // namespace husky
