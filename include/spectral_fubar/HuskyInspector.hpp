/**
 * @file HuskyInspector.hpp
 * @author Vedant Ranade
 * @brief ROS2 husky inspector node node's class declarations
 * @version 0.1
 * @date 2023-12-20
 *
 * @copyright Copyright (c) 2023
 *
 */
#ifndef HUSKY_INSPECTOR_HPP
#define HUSKY_INSPECTOR_HPP
#include <gazebo_msgs/msg/model_states.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <mutex>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <sensor_msgs/msg/image.hpp>
// @brief namespace for husky inspector code
namespace husky {
//@brief Motion state the robot will always be in one of them
enum class MotionState { START, FORWARD, TURN, DO_NOTHING };
//@brief Point class stores pixel position, and depth at that pixel
struct Point {
  double row;
  double column;
  double depth;
  Point(double inRow, double inCol) : row(inRow), column(inCol) { ; }
  Point() {
    row = 0;
    column = 0;
    depth = 0;
  }
};
using image = sensor_msgs::msg::Image;
class Inspector : public rclcpp::Node {
 public:
  Inspector();
  // @brief: Publishes cmd_vel
  void cmdVelPublisher();
  // @brief: Publishes on cmd_vel topic to move the robot forward in x direction
  inline void forward();
  // @brief: Publishes on cmd_vel topic to turn the robot in z direction
  inline void turn(bool right);
  // @brief: Publishes on cmd_vel topic to stop the robot
  inline void stop();
  // @brief : Retrieves the object detected or not flag
  bool isObjectDetected(){
    return isObjectDetected_;
  }
  // @brief : Retrieves the flag for is the detected object is on the right or let
  bool isRight(){
    return isRight_;
  }
 private:
  image lastDepth_;
  geometry_msgs::msg::Pose robotPose_;
  geometry_msgs::msg::Twist cmdVel_;
  rclcpp::TimerBase::SharedPtr cmdVelTimer_;
  rclcpp::Subscription<image>::SharedPtr depthImgSubscriber_;
  rclcpp::TimerBase::SharedPtr processTimer_;
  volatile bool isObjectDetected_ = false;
  volatile bool isRight_ = false;
  Point center_;
  std::mutex dataMutex_;
  MotionState motionState_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr
      obstaclePointPublisher_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr commandVelPublisher_;
  rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr subscription_;

  // @brief: callback to get robots current pose
  void modelStatesCallback(const gazebo_msgs::msg::ModelStates::SharedPtr msg);

  // @brief Subscribes to depth images
  void imageSubscriber(const image msg);
  // @brief processes images
  void imageProcessor(bool checkForObjectSize);
  // @brief checks the image and changes motion state of the husky robot
  void motionProcessor();
  // @brief updates motion state wrt to events generated by imageProcessor
  void updateMotionState();
};
};  // namespace husky
#endif