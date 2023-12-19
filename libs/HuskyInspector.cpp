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
}
void Inspector::cmdVelPublisher(){
    this->commandVelPublisher_->publish(cmdVel_);
}
}  // namespace husky
