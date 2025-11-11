

#include "beginner_tutorials/talker.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

MyTalker::MyTalker(): Node("minimal_publisher"), count_(0) 
{
  publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);

  this->declare_parameter("publishing_flag", false);

  this->get_parameter("publishing_flag", publishing_flag_);

  timer_ = this->create_wall_timer(
      500ms, std::bind(&MyTalker::timer_callback, this));
}

void MyTalker::timer_callback() {
  if (!publishing_flag_)
    return;
  auto message = std_msgs::msg::String();
  message.data = "Marcus' custom string message " + std::to_string(count_++);
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  publisher_->publish(message);
}
