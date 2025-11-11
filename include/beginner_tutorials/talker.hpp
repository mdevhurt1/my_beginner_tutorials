/**
 * @file talker.hpp
 * @author Marcus Hurt (mhurt@umd.edu)
 * @brief This is a small talker node that publishes messages to the "/topic"
 * topic.
 * @version 0.1
 * @date 2025-11-11
 * @details talker
 * See the LICENSE file for license information.
 *
 * @copyright Copyright (c) 2025
 *
 */

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
  /**
   * @brief Construct a new My Talker object
   *
   */
  MyTalker();

 private:
  /**
   * @brief Timer callback function for publishing messages
   *
   */
  void timer_callback();

  /**
   * @brief Handle the set_flag service request
   *
   * @param request - service request containing a boolean to set the flag
   * @param response - service response
   */
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