

#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"

class MyTalker : public rclcpp::Node {
 public:
  MyTalker();

 private:
  void timer_callback();
  void handle_set_flag_service(
      const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
      std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  bool publishing_flag_;
  bool service_flag_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;
  size_t count_;
};