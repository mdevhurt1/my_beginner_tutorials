

#include "beginner_tutorials/talker.hpp"
#include "rclcpp/rclcpp.hpp"
#include <string>

using namespace std::chrono_literals;

MyTalker::MyTalker(): Node("minimal_publisher"), count_(0) 
{
  publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);

  this->declare_parameter("publishing_flag", false);

  this->get_parameter("publishing_flag", publishing_flag_);

  service_ = this->create_service<std_srvs::srv::SetBool>(
      "set_flag",
      std::bind(&MyTalker::handle_set_flag_service, this,
                std::placeholders::_1, std::placeholders::_2));

  timer_ = this->create_wall_timer(
      500ms, std::bind(&MyTalker::timer_callback, this));
}

void MyTalker::timer_callback() {
  if (!publishing_flag_)
    return;

  auto message = std_msgs::msg::String();
  if (service_flag_) {
    message.data = "Marcus' custom service flag true " + std::to_string(count_++);
  } else {
    message.data = "Marcus' custom service flag false " + std::to_string(count_++);
  }
  RCLCPP_INFO_STREAM(this->get_logger(), "Publishing: " << message.data.c_str());
  publisher_->publish(message);
}

void MyTalker::handle_set_flag_service(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
  service_flag_ = request->data;
  response->success = true;
  response->message = service_flag_ ? "Service started." : "Service stopped.";
  RCLCPP_INFO(this->get_logger(), response->message.c_str());
}
