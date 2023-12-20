/**
 * @file HuskyInspector.cpp
 * @author Vedant Ranade
 * @brief ROS2 husky inspector node node's class definitions
 * @version 0.1
 * @date 2023-12-20
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "HuskyInspector.hpp"

#include <tf2/LinearMath/Quaternion.h>

#include <geometry_msgs/msg/detail/point__struct.hpp>
#include <iomanip>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
namespace husky {
Inspector::Inspector() : Node("inspector") {
  cmdVel_ = geometry_msgs::msg::Twist();
  commandVelPublisher_ =
      this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  obstaclePointPublisher_ =
      this->create_publisher<geometry_msgs::msg::Point>("obstacle_loc", 10);
  // timer callback function is called every 100ms
  cmdVelTimer_ =
      this->create_wall_timer(std::chrono::milliseconds(20),
                              std::bind(&Inspector::cmdVelPublisher, this));
  // this is to subscribe to the last known robot pose
  subscription_ = create_subscription<gazebo_msgs::msg::ModelStates>(
      "/gazebo/model_states", 10,
      std::bind(&Inspector::modelStatesCallback, this, std::placeholders::_1));
  // subscribe to depth image topic
  depthImgSubscriber_ = this->create_subscription<image>(
      "/front_realsense_depth/depth/image_raw", 10,
      std::bind(&Inspector::imageSubscriber, this, std::placeholders::_1));
  // this timer updates motion and processes received images at 10hz
  processTimer_ =
      this->create_wall_timer(std::chrono::milliseconds(100),
                              std::bind(&Inspector::motionProcessor, this));
  motionState_ = MotionState::DO_NOTHING;
}
void Inspector::motionProcessor() {
  if (lastDepth_.header.stamp.nanosec > 0 &&
      motionState_ == MotionState::DO_NOTHING) {
    motionState_ = MotionState::START;
  }
  updateMotionState();
}
void Inspector::updateMotionState() {
  switch (motionState_) {
    case MotionState::START:
      motionState_ = MotionState::FORWARD;
      break;
    case MotionState::FORWARD:
      imageProcessor(true);
      if (isObjectDetected_) {
        stop();
        // set state to turn
        RCLCPP_INFO(this->get_logger(), "Switching state to turn");
        motionState_ = MotionState::TURN;
      } else {
        // move forward
        forward();
      }
      break;
    case MotionState::TURN:
      imageProcessor(false);
      if (!isObjectDetected_) {
        stop();
        stop();
        stop();
        motionState_ = MotionState::FORWARD;
      } else {
        turn(isRight_);
      }
      break;
    case MotionState::DO_NOTHING:
      stop();
      break;
  }
}
void Inspector::cmdVelPublisher() {
  this->commandVelPublisher_->publish(cmdVel_);
}
void Inspector::imageProcessor(bool checkForObjectSize) {
  std::unique_lock<std::mutex> lock(dataMutex_);
  const auto data = reinterpret_cast<float*>(lastDepth_.data.data());
  const auto& maxHeight = lastDepth_.height;
  const auto& maxWidth = lastDepth_.width;
  const auto& step = lastDepth_.step;
  // depth threshold is 2m, robot stops here
  static const constexpr float depthThreshold = 2.0;
  // check 5 pixels below
  static const constexpr size_t heightDrop = 1;
  for (size_t height = 0; height < (maxHeight / 2); height++) {
    for (size_t width = 0; width < maxWidth; width++) {
      const auto idx = height * maxWidth + width;
      assert((width) <= step / sizeof(float));
      const auto& depth = data[idx];
      if (depth < depthThreshold) {
        // RCLCPP_INFO(this->get_logger(),"Depth %f", depth);
        isObjectDetected_ = true;
        if (checkForObjectSize) {
          size_t extentOfObject;
          for (extentOfObject = width; extentOfObject < maxWidth;
               extentOfObject++) {
            const auto internalIdx =
                (height + heightDrop) * maxWidth + extentOfObject;
            const auto& internalDepth = data[internalIdx];
            if ((internalDepth > depthThreshold + 0.5)) {
              break;
            }
          }
          this->center_.column = static_cast<float>(extentOfObject + width) / 2;
          this->center_.row = height;
          this->center_.depth =
              data[height * maxWidth + ((extentOfObject + width) / 2)];
          this->isRight_ = (center_.column > static_cast<float>(maxWidth / 2));
          RCLCPP_INFO(this->get_logger(),
                      "Publishing Detected objected centerPoint height %ld "
                      "width %f extend %ld on /obstacle_loc",
                      height, this->center_.column, extentOfObject);
          auto pt = geometry_msgs::msg::Point();
          // @brief publishes the pixel location of center of obstacle, and its
          // depth
          pt.set__x(this->center_.column);
          pt.set__y(this->center_.row);
          pt.set__z(this->center_.depth);
          obstaclePointPublisher_->publish(pt);
        }
        break;
      } else {
        isObjectDetected_ = false;
      }
    }
    if (isObjectDetected_) {
      break;
    }
  }
}
void Inspector::imageSubscriber(const image msg) {
  std::unique_lock<std::mutex> lock(dataMutex_);
  lastDepth_ = msg;
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
  RCLCPP_DEBUG(this->get_logger(), "Moving forward");

  this->cmdVel_ = geometry_msgs::msg::Twist();
  this->cmdVel_.linear.x = 1.0;
  commandVelPublisher_->publish(cmdVel_);
}
// @brief: Publishes on cmd_vel topic to turn the robot in z direction
inline void Inspector::turn(bool right) {
  this->cmdVel_ = geometry_msgs::msg::Twist();
  cmdVel_.linear.x = 0.5;
  cmdVel_.angular.z = 0.4 * (right ? 1.0 : -1.0);
  RCLCPP_INFO(this->get_logger(), "object to %s", right ? "right" : "left");
  commandVelPublisher_->publish(cmdVel_);
}
// @brief: Publishes on cmd_vel topic to stop the robot
inline void Inspector::stop() {
  RCLCPP_DEBUG(this->get_logger(), "Stopping");
  this->cmdVel_ = geometry_msgs::msg::Twist();
  this->commandVelPublisher_->publish(cmdVel_);
}

}  // namespace husky
