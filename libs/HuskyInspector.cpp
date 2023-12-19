#include "HuskyInspector.hpp"

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
}
void Inspector::cmdVelPublisher(){
    this->commandVelPublisher_->publish(cmdVel_);
}
  void Inspector::modelStatesCallback(const gazebo_msgs::msg::ModelStates::SharedPtr msg) {
    for (size_t i = 0; i < msg->name.size(); ++i) {
      if (msg->name[i] == "husky") {
         robotPose_ = msg->pose[i];
        RCLCPP_INFO(get_logger(), "Husky Pose: x=%f, y=%f, z=%f",
                    robotPose_.position.x, robotPose_.position.y, robotPose_.position.z);
        break;  // Exit loop once "husky" is found
      }
    }
  }
}  // namespace husky
